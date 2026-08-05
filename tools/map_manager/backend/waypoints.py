from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml


WAYPOINT_ID_RE = re.compile(r"^wp-[a-z0-9-]{8,64}$")
ACTIVE_MISSION_STATES = {"queued", "waiting_server", "running", "cancelling"}
TERMINAL_MISSION_STATES = {"succeeded", "partial", "failed", "cancelled"}


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def make_waypoint_id() -> str:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return f"wp-{stamp}-{os.urandom(3).hex()}"


def write_json_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temporary, path)


def write_yaml_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(value, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def normalize_waypoint_pose(pose: dict[str, Any]) -> dict[str, Any]:
    position_source = pose.get("position") or {}
    orientation_source = pose.get("orientation") or {}
    try:
        position_values = {
            key: float(position_source.get(key, 0.0)) for key in ("x", "y", "z")
        }
        orientation_values = {
            key: float(orientation_source.get(key, 0.0)) for key in ("x", "y", "z", "w")
        }
    except (TypeError, ValueError) as exc:
        raise ValueError("位姿必须是有效数字") from exc
    if not all(math.isfinite(value) for value in (*position_values.values(), *orientation_values.values())):
        raise ValueError("位姿包含无效数字")
    norm = math.sqrt(sum(value * value for value in orientation_values.values()))
    if norm < 1.0e-9:
        raise ValueError("位姿四元数长度为零")
    orientation_values = {
        key: value / norm for key, value in orientation_values.items()
    }
    return {
        "position": position_values,
        "orientation": orientation_values,
    }


def waypoint_yaw(waypoint: dict[str, Any]) -> float:
    q = waypoint["orientation"]
    siny = 2.0 * (q["w"] * q["z"] + q["x"] * q["y"])
    cosy = 1.0 - 2.0 * (q["y"] * q["y"] + q["z"] * q["z"])
    return math.atan2(siny, cosy)


class WaypointStore:
    """Map-scoped, atomic waypoint storage."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.lock = threading.RLock()

    def _load(self) -> dict[str, Any]:
        default: dict[str, Any] = {"schema": 1, "maps": {}}
        if not self.path.is_file():
            return default
        try:
            value = yaml.safe_load(self.path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            return default
        if not isinstance(value, dict) or not isinstance(value.get("maps", {}), dict):
            return default
        return {"schema": 1, "maps": value.get("maps", {})}

    def _items_from(self, data: dict[str, Any], map_id: str) -> list[dict[str, Any]]:
        scope = data.get("maps", {}).get(map_id, {})
        source = scope.get("waypoints", []) if isinstance(scope, dict) else []
        items: list[dict[str, Any]] = []
        for raw in source if isinstance(source, list) else []:
            if not isinstance(raw, dict) or not WAYPOINT_ID_RE.fullmatch(str(raw.get("id", ""))):
                continue
            name = str(raw.get("name", "")).strip()
            if not name:
                continue
            try:
                pose = normalize_waypoint_pose(raw)
            except ValueError:
                continue
            item = {
                "id": str(raw["id"]),
                "name": name[:80],
                **pose,
                "created_at": raw.get("created_at"),
                "updated_at": raw.get("updated_at"),
            }
            item["yaw"] = waypoint_yaw(item)
            items.append(item)
        return items

    def list(self, map_id: str) -> list[dict[str, Any]]:
        with self.lock:
            return self._items_from(self._load(), map_id)

    def _save_items(self, data: dict[str, Any], map_id: str, items: list[dict[str, Any]]) -> None:
        stored = []
        for item in items:
            stored.append({key: value for key, value in item.items() if key != "yaw"})
        data.setdefault("maps", {})[map_id] = {
            "updated_at": now_iso(),
            "waypoints": stored,
        }
        write_yaml_atomic(self.path, data)

    def add(self, map_id: str, pose: dict[str, Any], name: str | None = None) -> dict[str, Any]:
        clean_name = str(name or "").strip()
        with self.lock:
            data = self._load()
            items = self._items_from(data, map_id)
            if len(items) >= 100:
                raise ValueError("每张地图最多保存 100 个目标点")
            if not clean_name:
                used = {item["name"] for item in items}
                index = 1
                while f"目标点 {index}" in used:
                    index += 1
                clean_name = f"目标点 {index}"
            if len(clean_name) > 80:
                raise ValueError("目标点名称不能超过 80 个字符")
            normalized = normalize_waypoint_pose(pose)
            timestamp = now_iso()
            waypoint = {
                "id": make_waypoint_id(),
                "name": clean_name,
                **normalized,
                "created_at": timestamp,
                "updated_at": timestamp,
            }
            items.append(waypoint)
            self._save_items(data, map_id, items)
            waypoint["yaw"] = waypoint_yaw(waypoint)
            return waypoint

    def rename(self, map_id: str, waypoint_id: str, name: str) -> dict[str, Any]:
        clean_name = name.strip()
        if not clean_name:
            raise ValueError("目标点名称不能为空")
        if len(clean_name) > 80:
            raise ValueError("目标点名称不能超过 80 个字符")
        with self.lock:
            data = self._load()
            items = self._items_from(data, map_id)
            waypoint = next((item for item in items if item["id"] == waypoint_id), None)
            if waypoint is None:
                raise KeyError(waypoint_id)
            waypoint["name"] = clean_name
            waypoint["updated_at"] = now_iso()
            self._save_items(data, map_id, items)
            return waypoint

    def delete(self, map_id: str, waypoint_id: str) -> dict[str, Any]:
        with self.lock:
            data = self._load()
            items = self._items_from(data, map_id)
            waypoint = next((item for item in items if item["id"] == waypoint_id), None)
            if waypoint is None:
                raise KeyError(waypoint_id)
            self._save_items(data, map_id, [item for item in items if item["id"] != waypoint_id])
            return waypoint

    def reorder(self, map_id: str, waypoint_ids: list[str]) -> list[dict[str, Any]]:
        if len(waypoint_ids) != len(set(waypoint_ids)):
            raise ValueError("目标点顺序中存在重复项")
        with self.lock:
            data = self._load()
            items = self._items_from(data, map_id)
            current = {item["id"]: item for item in items}
            if set(waypoint_ids) != set(current):
                raise ValueError("新的顺序必须包含当前地图的全部目标点")
            ordered = [current[waypoint_id] for waypoint_id in waypoint_ids]
            self._save_items(data, map_id, ordered)
            return ordered


class WaypointMissionManager:
    def __init__(
        self,
        repo_root: Path,
        root: Path,
        executor_path: Path | None = None,
    ) -> None:
        self.repo_root = repo_root.resolve()
        self.root = root
        self.missions_root = root / "missions"
        self.status_path = root / "status.json"
        self.executor_path = executor_path or Path(__file__).with_name("waypoint_executor.py")
        self.lock = threading.RLock()
        self.process: subprocess.Popen[str] | None = None
        self.reader: threading.Thread | None = None
        self.logs: deque[str] = deque(maxlen=160)
        self.cancel_requested = False
        self.root.mkdir(parents=True, exist_ok=True)
        self.missions_root.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _idle_state() -> dict[str, Any]:
        return {
            "status": "idle",
            "mission_id": None,
            "map_id": None,
            "pid": None,
            "total": 0,
            "processed": 0,
            "completed": 0,
            "failed_count": 0,
            "results": [],
            "current_index": None,
            "current_waypoint_id": None,
            "current_waypoint_name": None,
            "progress_percent": 0.0,
            "leg_progress_percent": 0.0,
            "distance_remaining": None,
            "estimated_time_remaining_sec": None,
            "navigation_time_sec": None,
            "number_of_recoveries": 0,
            "started_at": None,
            "finished_at": None,
            "elapsed_sec": 0.0,
            "message": "尚未开始导航任务",
            "error": None,
        }

    def _read_status(self) -> dict[str, Any]:
        if not self.status_path.is_file():
            return self._idle_state()
        try:
            value = json.loads(self.status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return self._idle_state()
        return {**self._idle_state(), **(value if isinstance(value, dict) else {})}

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            state = self._read_status()
            process = self.process
            if process and process.poll() is None:
                state["pid"] = process.pid
                if self.cancel_requested:
                    state["status"] = "cancelling"
                    state["message"] = "正在通知 Nav2 取消当前目标…"
            state["active"] = state.get("status") in ACTIVE_MISSION_STATES
            state["can_cancel"] = bool(process and process.poll() is None and state["active"])
            state["logs"] = list(self.logs)
            return state

    def _ros_environment(self) -> dict[str, str]:
        environment = dict(os.environ)
        environment["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        if not environment.get("CYCLONEDDS_URI"):
            interface = environment.get("GO2_IFACE", "enx6c1ff7bc241e")
            environment["CYCLONEDDS_URI"] = (
                "<CycloneDDS><Domain><General><Interfaces>"
                f'<NetworkInterface name="{interface}" priority="default" multicast="default" />'
                "</Interfaces></General></Domain></CycloneDDS>"
            )
        return environment

    def capture_pose(self, timeout_seconds: float = 5.0) -> dict[str, Any]:
        with tempfile.TemporaryDirectory(prefix="pose-", dir=self.root) as temporary:
            output = Path(temporary) / "pose.json"
            result = subprocess.run(
                [
                    sys.executable,
                    str(self.executor_path),
                    "pose",
                    "--output",
                    str(output),
                    "--timeout",
                    str(timeout_seconds),
                ],
                cwd=self.repo_root,
                env=self._ros_environment(),
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=timeout_seconds + 5.0,
                check=False,
            )
            if result.returncode != 0 or not output.is_file():
                detail = result.stdout.strip().splitlines()
                raise RuntimeError(detail[-1] if detail else "无法读取机器狗当前位姿")
            try:
                value = json.loads(output.read_text(encoding="utf-8"))
                return {**value, **normalize_waypoint_pose(value)}
            except (OSError, json.JSONDecodeError, ValueError) as exc:
                raise RuntimeError(f"当前位姿数据无效：{exc}") from exc

    def start(
        self,
        mission_id: str,
        map_id: str,
        waypoints: list[dict[str, Any]],
        *,
        stop_on_failure: bool,
        waypoint_timeout_sec: float,
        pause_between_sec: float,
    ) -> dict[str, Any]:
        with self.lock:
            if self.process and self.process.poll() is None:
                raise RuntimeError("已有多目标导航任务正在运行")
            previous = self._read_status()
            if previous.get("status") in ACTIVE_MISSION_STATES:
                raise RuntimeError("检测到未清理的导航任务，请刷新页面后重试")
            self.logs.clear()
            self.cancel_requested = False
            mission = {
                "schema": 1,
                "mission_id": mission_id,
                "map_id": map_id,
                "created_at": now_iso(),
                "stop_on_failure": stop_on_failure,
                "waypoint_timeout_sec": waypoint_timeout_sec,
                "pause_between_sec": pause_between_sec,
                "server_timeout_sec": 20.0,
                "waypoints": [
                    {key: value for key, value in waypoint.items() if key != "yaw"}
                    for waypoint in waypoints
                ],
            }
            mission_path = self.missions_root / f"{mission_id}.json"
            write_json_atomic(mission_path, mission)
            write_json_atomic(self.status_path, {
                **self._idle_state(),
                "status": "queued",
                "mission_id": mission_id,
                "map_id": map_id,
                "total": len(waypoints),
                "started_at": now_iso(),
                "message": "正在启动多目标导航…",
            })
            try:
                process = subprocess.Popen(
                    [
                        sys.executable,
                        str(self.executor_path),
                        "navigate",
                        "--mission",
                        str(mission_path),
                        "--status",
                        str(self.status_path),
                    ],
                    cwd=self.repo_root,
                    env=self._ros_environment(),
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    bufsize=1,
                    start_new_session=True,
                )
            except OSError:
                write_json_atomic(self.status_path, {
                    **self._read_status(),
                    "status": "failed",
                    "finished_at": now_iso(),
                    "message": "导航任务进程启动失败",
                })
                raise
            self.process = process
            self.reader = threading.Thread(target=self._watch, args=(process,), daemon=True)
            self.reader.start()
            return self.snapshot()

    def _watch(self, process: subprocess.Popen[str]) -> None:
        if process.stdout is not None:
            with process.stdout:
                for raw_line in process.stdout:
                    line = raw_line.rstrip()
                    if line:
                        stamp = datetime.now().strftime("%H:%M:%S")
                        with self.lock:
                            self.logs.append(f"[{stamp}] {line}")
        exit_code = process.wait()
        with self.lock:
            if self.process is not process:
                return
            state = self._read_status()
            if state.get("status") not in TERMINAL_MISSION_STATES:
                cancelled = self.cancel_requested
                state.update({
                    "status": "cancelled" if cancelled else "failed",
                    "pid": None,
                    "finished_at": now_iso(),
                    "message": "导航任务已取消" if cancelled else "导航任务进程异常退出",
                    "error": None if cancelled else f"任务进程退出码 {exit_code}",
                })
                write_json_atomic(self.status_path, state)
            self.process = None
            self.cancel_requested = False

    def cancel(self) -> dict[str, Any]:
        with self.lock:
            process = self.process
            if not process or process.poll() is not None:
                state = self._read_status()
                if state.get("status") in ACTIVE_MISSION_STATES:
                    state.update({
                        "status": "cancelled",
                        "pid": None,
                        "finished_at": now_iso(),
                        "message": "导航任务已取消",
                    })
                    write_json_atomic(self.status_path, state)
                return self.snapshot()
            first_request = not self.cancel_requested
            self.cancel_requested = True
            try:
                os.kill(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            if first_request:
                threading.Thread(
                    target=self._cancel_watchdog,
                    args=(process,),
                    daemon=True,
                ).start()
            return self.snapshot()

    def _cancel_watchdog(self, process: subprocess.Popen[str]) -> None:
        """Escalate only if a ROS action cancellation cannot finish cleanly."""
        try:
            process.wait(timeout=8.0)
            return
        except subprocess.TimeoutExpired:
            pass
        for stop_signal, timeout_seconds in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            try:
                os.killpg(process.pid, stop_signal)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=timeout_seconds)
                return
            except subprocess.TimeoutExpired:
                continue

    def _stop_process(self, process: subprocess.Popen[str]) -> None:
        self.cancel_requested = True
        for stop_signal, timeout_seconds in (
            (signal.SIGINT, 6.0),
            (signal.SIGTERM, 2.0),
            (signal.SIGKILL, 1.0),
        ):
            try:
                if stop_signal == signal.SIGINT:
                    os.kill(process.pid, stop_signal)
                else:
                    os.killpg(process.pid, stop_signal)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=timeout_seconds)
                return
            except subprocess.TimeoutExpired:
                continue

    def recover_stale_process(self) -> None:
        state = self._read_status()
        if state.get("status") not in ACTIVE_MISSION_STATES:
            return
        try:
            pid = int(state.get("pid") or 0)
        except (TypeError, ValueError):
            pid = 0
        if pid > 1:
            try:
                command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
                if str(self.executor_path) in command and " navigate " in f" {command} ":
                    for stop_signal, wait_seconds in (
                        (signal.SIGINT, 6.0),
                        (signal.SIGTERM, 2.0),
                        (signal.SIGKILL, 1.0),
                    ):
                        try:
                            os.killpg(pid, stop_signal)
                        except ProcessLookupError:
                            break
                        deadline = time.monotonic() + wait_seconds
                        while time.monotonic() < deadline and Path(f"/proc/{pid}").exists():
                            time.sleep(0.05)
                        if not Path(f"/proc/{pid}").exists():
                            break
            except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
                pass
        state.update({
            "status": "cancelled",
            "pid": None,
            "finished_at": now_iso(),
            "message": "Web 服务重启，上一任务已安全终止",
            "error": None,
        })
        write_json_atomic(self.status_path, state)

    def close(self) -> None:
        with self.lock:
            process = self.process
        if process and process.poll() is None:
            self._stop_process(process)
        if self.reader and self.reader is not threading.current_thread():
            self.reader.join(timeout=1.0)
        with self.lock:
            if self.process is process:
                self.process = None
            self.cancel_requested = False

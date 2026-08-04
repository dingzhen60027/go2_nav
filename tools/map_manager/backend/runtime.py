from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any


def _now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


class RuntimeManager:
    MODES = {
        "localization": "start_localization.sh",
        "navigation": "start_navigation.sh",
    }
    MAPPING_ALGORITHMS = {
        "faster_lio": "start_mapping.sh",
        "fastlio2": "start_mapping_fastlio2.sh",
    }

    def __init__(self, repo_root: Path, state_path: Path) -> None:
        self.repo_root = repo_root.resolve()
        self.state_path = state_path
        self.lock = threading.RLock()
        self.process: subprocess.Popen[str] | None = None
        self.reader: threading.Thread | None = None
        self.logs: deque[str] = deque(maxlen=240)
        self.state: dict[str, Any] = {
            "status": "idle",
            "mode": None,
            "algorithm": None,
            "pid": None,
            "started_at": None,
            "stopped_at": None,
            "exit_code": None,
            "error": None,
            "run_id": None,
            "capture_path": None,
            "capture_dir": None,
            "capture_baseline": None,
            "capture_status": None,
            "capture_session_id": None,
        }

    def _write_state(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        temp = self.state_path.with_suffix(".tmp")
        temp.write_text(json.dumps(self.state, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temp, self.state_path)

    def recover_stale_process(self) -> None:
        if not self.state_path.exists():
            return
        try:
            previous = json.loads(self.state_path.read_text(encoding="utf-8"))
            pid = int(previous.get("pid") or 0)
            mode = previous.get("mode")
            algorithm = previous.get("algorithm")
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return
        with self.lock:
            self.state = {**self.state, **previous}
        script = self._script_for(mode, algorithm)
        if previous.get("status") not in {"running", "stopping"}:
            return
        if pid > 1 and script:
            try:
                command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
                if str(self.repo_root / script) in command:
                    for stop_signal, timeout in (
                        (signal.SIGINT, 8.0),
                        (signal.SIGTERM, 3.0),
                        (signal.SIGKILL, 1.0),
                    ):
                        try:
                            os.killpg(pid, stop_signal)
                        except ProcessLookupError:
                            break
                        deadline = time.monotonic() + timeout
                        while time.monotonic() < deadline and Path(f"/proc/{pid}").exists():
                            time.sleep(0.05)
                        if not Path(f"/proc/{pid}").exists():
                            break
            except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
                pass
        self.state.update({
            "status": "idle",
            "pid": None,
            "stopped_at": _now_iso(),
            "error": None,
        })
        self._write_state()

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {**self.state, "logs": list(self.logs)}

    def _script_for(self, mode: str, algorithm: str | None = None) -> str | None:
        if mode == "mapping":
            return self.MAPPING_ALGORITHMS.get(algorithm or "faster_lio")
        return self.MODES.get(mode)

    def start(
        self,
        mode: str,
        algorithm: str | None = None,
        *,
        environment: dict[str, str] | None = None,
        run_id: str | None = None,
        capture_path: Path | None = None,
        capture_dir: Path | None = None,
        capture_baseline: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        selected_algorithm = (algorithm or "faster_lio") if mode == "mapping" else None
        script_name = self._script_for(mode, selected_algorithm)
        if not script_name:
            raise ValueError("不支持的运行模式或建图算法")
        script = self.repo_root / script_name
        if not script.is_file():
            raise FileNotFoundError(f"启动脚本不存在: {script_name}")

        with self.lock:
            if self.process and self.process.poll() is None:
                raise RuntimeError("已有流程正在运行，请先停止")
            self.logs.clear()
            runner = Path(__file__).with_name("workflow_runner.py")
            self.process = subprocess.Popen(
                [sys.executable, str(runner), str(script)],
                cwd=self.repo_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env={**os.environ, **(environment or {})},
            )
            self.state = {
                "status": "running",
                "mode": mode,
                "algorithm": selected_algorithm,
                "pid": self.process.pid,
                "started_at": _now_iso(),
                "stopped_at": None,
                "exit_code": None,
                "error": None,
                "run_id": run_id,
                "capture_path": str(capture_path) if capture_path else None,
                "capture_dir": str(capture_dir) if capture_dir else None,
                "capture_baseline": capture_baseline,
                "capture_status": "pending" if mode == "mapping" else None,
                "capture_session_id": None,
            }
            self._write_state()
            process = self.process
            self.reader = threading.Thread(target=self._watch, args=(process,), daemon=True)
            self.reader.start()
            return self.snapshot()

    def mark_capture(self, status: str, session_id: str | None = None) -> dict[str, Any]:
        if status not in {"archived", "discarded"}:
            raise ValueError("invalid capture status")
        with self.lock:
            if self.state.get("mode") != "mapping" or not self.state.get("run_id"):
                raise RuntimeError("没有可处理的建图运行")
            self.state["capture_status"] = status
            self.state["capture_session_id"] = session_id
            self._write_state()
            return self.snapshot()

    def _watch(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
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
            was_stopping = self.state["status"] == "stopping"
            self.state.update({
                "status": "idle" if was_stopping or exit_code == 0 else "failed",
                "pid": None,
                "stopped_at": _now_iso(),
                "exit_code": exit_code,
                "error": None if was_stopping or exit_code == 0 else f"流程异常退出，退出码 {exit_code}",
            })
            self.process = None
            self._write_state()

    @staticmethod
    def _wait(process: subprocess.Popen[str], timeout: float) -> bool:
        try:
            process.wait(timeout=timeout)
            return True
        except subprocess.TimeoutExpired:
            return False

    def stop(self, timeout: float = 12.0) -> dict[str, Any]:
        with self.lock:
            process = self.process
            if not process or process.poll() is not None:
                self.process = None
                self.state.update({"status": "idle", "pid": None})
                self._write_state()
                return self.snapshot()
            self.state["status"] = "stopping"
            self._write_state()

        for stop_signal, wait_seconds in (
            (signal.SIGINT, timeout),
            (signal.SIGTERM, 4.0),
            (signal.SIGKILL, 2.0),
        ):
            try:
                os.killpg(process.pid, stop_signal)
            except ProcessLookupError:
                break
            if self._wait(process, wait_seconds):
                break

        if self.reader and self.reader is not threading.current_thread():
            self.reader.join(timeout=1.0)
        with self.lock:
            if self.process is process:
                exit_code = process.poll()
                self.process = None
                self.state.update({
                    "status": "idle",
                    "pid": None,
                    "stopped_at": _now_iso(),
                    "exit_code": exit_code,
                    "error": None,
                })
                self._write_state()
            return self.snapshot()

    def close(self) -> None:
        self.stop(timeout=6.0)

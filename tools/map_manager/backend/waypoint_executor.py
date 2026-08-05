#!/usr/bin/env python3
"""Small ROS 2 worker used by the map manager for waypoint navigation.

The Web process intentionally does not own a ROS node.  Pose capture and an
entire multi-goal mission each run in their own process, so a Web restart or a
cancel request cannot leave an action client in an unknown in-process state.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


TERMINAL_STATES = {"succeeded", "partial", "failed", "cancelled"}


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def write_json_atomic(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def normalize_quaternion(value: dict[str, Any]) -> dict[str, float]:
    values = [float(value.get(key, 0.0)) for key in ("x", "y", "z", "w")]
    if not all(math.isfinite(item) for item in values):
        raise ValueError("目标点四元数包含无效数值")
    norm = math.sqrt(sum(item * item for item in values))
    if norm < 1.0e-9:
        raise ValueError("目标点四元数长度为零")
    return dict(zip(("x", "y", "z", "w"), (item / norm for item in values)))


def lookup_pose(
    node: Node,
    tf_buffer: Buffer,
    timeout_seconds: float,
    cancel_event: threading.Event | None = None,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_seconds
    last_error = ""
    while time.monotonic() < deadline:
        if cancel_event and cancel_event.is_set():
            raise InterruptedError("操作已取消")
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.15),
            ).transform
            values = (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            )
            if not all(math.isfinite(float(item)) for item in values):
                raise ValueError("TF 位姿包含无效数值")
            orientation = normalize_quaternion({
                "x": transform.rotation.x,
                "y": transform.rotation.y,
                "z": transform.rotation.z,
                "w": transform.rotation.w,
            })
            return {
                "frame_id": "map",
                "child_frame_id": "base_link",
                "position": {
                    "x": float(transform.translation.x),
                    "y": float(transform.translation.y),
                    "z": float(transform.translation.z),
                },
                "orientation": orientation,
                "captured_at": now_iso(),
            }
        except (TransformException, ValueError) as exc:
            last_error = str(exc)
    suffix = f"：{last_error}" if last_error else ""
    raise TimeoutError(f"{timeout_seconds:.1f} 秒内未收到 map → base_link TF{suffix}")


class MissionRunner:
    def __init__(self, mission_path: Path, status_path: Path) -> None:
        self.mission_path = mission_path
        self.status_path = status_path
        self.mission = json.loads(mission_path.read_text(encoding="utf-8"))
        self.waypoints = list(self.mission.get("waypoints") or [])
        self.cancel_event = threading.Event()
        self.node: Node | None = None
        self.client: ActionClient | None = None
        self.tf_buffer: Buffer | None = None
        self.tf_listener: TransformListener | None = None
        self.goal_handle: Any = None
        self.started_monotonic = time.monotonic()
        self.last_write = 0.0
        self.last_progress = 0.0
        self.processed = 0
        self.completed = 0
        self.failed_count = 0
        self.results: list[dict[str, Any]] = []
        self.current_index: int | None = None
        self.current_waypoint: dict[str, Any] | None = None
        self.leg_progress = 0.0
        self.leg_baseline = 0.0
        self.feedback: dict[str, Any] = {}
        self.state: dict[str, Any] = {
            "schema": 1,
            "mission_id": self.mission.get("mission_id"),
            "map_id": self.mission.get("map_id"),
            "status": "queued",
            "pid": os.getpid(),
            "total": len(self.waypoints),
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
            "started_at": now_iso(),
            "finished_at": None,
            "elapsed_sec": 0.0,
            "message": "任务排队中",
            "error": None,
        }
        self._write(force=True)

    def request_cancel(self, *_: Any) -> None:
        self.cancel_event.set()

    @staticmethod
    def _duration_seconds(value: Any) -> float | None:
        if value is None:
            return None
        try:
            return float(value.sec) + float(value.nanosec) / 1.0e9
        except (AttributeError, TypeError, ValueError):
            return None

    def _overall_progress(self) -> float:
        total = len(self.waypoints)
        if not total:
            return 0.0
        value = (self.processed + self.leg_progress) * 100.0 / total
        self.last_progress = max(self.last_progress, min(value, 99.9))
        return self.last_progress

    def _write(self, *, force: bool = False, **updates: Any) -> None:
        self.state.update(updates)
        self.state.update({
            "pid": os.getpid(),
            "processed": self.processed,
            "completed": self.completed,
            "failed_count": self.failed_count,
            "results": list(self.results),
            "current_index": self.current_index,
            "current_waypoint_id": (
                self.current_waypoint.get("id") if self.current_waypoint else None
            ),
            "current_waypoint_name": (
                self.current_waypoint.get("name") if self.current_waypoint else None
            ),
            "progress_percent": round(self._overall_progress(), 1),
            "leg_progress_percent": round(self.leg_progress * 100.0, 1),
            "elapsed_sec": round(time.monotonic() - self.started_monotonic, 1),
            **self.feedback,
        })
        current = time.monotonic()
        terminal = self.state.get("status") in TERMINAL_STATES
        if force or terminal or current - self.last_write >= 0.2:
            write_json_atomic(self.status_path, self.state)
            self.last_write = current

    def _finish(self, status: str, message: str, error: str | None = None) -> int:
        if status in {"succeeded", "partial"}:
            self.last_progress = 100.0
            self.leg_progress = 0.0
        self.current_waypoint = None
        self.current_index = None
        self._write(
            force=True,
            status=status,
            finished_at=now_iso(),
            message=message,
            error=error,
            progress_percent=100.0 if status in {"succeeded", "partial"} else self.last_progress,
        )
        return 0 if status in {"succeeded", "partial", "cancelled"} else 1

    def _wait_server(self, timeout_seconds: float) -> bool:
        assert self.node is not None and self.client is not None
        deadline = time.monotonic() + timeout_seconds
        self._write(force=True, status="waiting_server", message="正在连接 Nav2 导航服务…")
        while time.monotonic() < deadline:
            if self.cancel_event.is_set():
                return False
            if self.client.server_is_ready():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self._write()
        return False

    def _feedback_callback(self, message: Any) -> None:
        feedback = message.feedback
        try:
            remaining = max(0.0, float(feedback.distance_remaining))
        except (AttributeError, TypeError, ValueError):
            remaining = None
        if remaining is not None and math.isfinite(remaining):
            self.leg_baseline = max(self.leg_baseline, remaining, 0.05)
            candidate = 1.0 - remaining / self.leg_baseline
            self.leg_progress = max(self.leg_progress, min(0.99, max(0.0, candidate)))
        self.feedback = {
            "distance_remaining": round(remaining, 3) if remaining is not None else None,
            "estimated_time_remaining_sec": self._duration_seconds(
                getattr(feedback, "estimated_time_remaining", None)
            ),
            "navigation_time_sec": self._duration_seconds(
                getattr(feedback, "navigation_time", None)
            ),
            "number_of_recoveries": int(getattr(feedback, "number_of_recoveries", 0)),
        }
        self._write()

    def _wait_future(self, future: Any, deadline: float | None = None) -> bool:
        assert self.node is not None
        while rclpy.ok() and not future.done():
            if deadline is not None and time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self._write()
        return future.done()

    def _cancel_active_goal(self, reason: str) -> None:
        if not self.goal_handle:
            return
        self._write(force=True, status="cancelling", message=reason)
        try:
            future = self.goal_handle.cancel_goal_async()
            self._wait_future(future, time.monotonic() + 5.0)
        except Exception as exc:  # ROS transport may already be gone.
            print(f"cancel_goal failed: {exc}", flush=True)

    def _initial_distance(self, waypoint: dict[str, Any]) -> float:
        if self.node is None or self.tf_buffer is None:
            return 0.0
        try:
            current = lookup_pose(self.node, self.tf_buffer, 0.8, self.cancel_event)
            dx = float(waypoint["position"]["x"]) - float(current["position"]["x"])
            dy = float(waypoint["position"]["y"]) - float(current["position"]["y"])
            return math.hypot(dx, dy)
        except (TimeoutError, InterruptedError, KeyError, TypeError, ValueError):
            return 0.0

    def _navigate_one(self, waypoint: dict[str, Any], timeout_seconds: float) -> tuple[bool, str]:
        assert self.node is not None and self.client is not None
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.node.get_clock().now().to_msg()
        position = waypoint["position"]
        orientation = normalize_quaternion(waypoint["orientation"])
        pose.pose.position.x = float(position["x"])
        pose.pose.position.y = float(position["y"])
        pose.pose.position.z = float(position.get("z", 0.0))
        pose.pose.orientation.x = orientation["x"]
        pose.pose.orientation.y = orientation["y"]
        pose.pose.orientation.z = orientation["z"]
        pose.pose.orientation.w = orientation["w"]

        self.leg_progress = 0.0
        self.leg_baseline = max(0.05, self._initial_distance(waypoint))
        self.feedback = {
            "distance_remaining": round(self.leg_baseline, 3),
            "estimated_time_remaining_sec": None,
            "navigation_time_sec": 0.0,
            "number_of_recoveries": 0,
        }
        self._write(
            force=True,
            status="running",
            message=f"正在前往 {waypoint.get('name', '目标点')}",
            error=None,
        )

        goal = NavigateToPose.Goal()
        goal.pose = pose
        send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_callback)
        if not self._wait_future(send_future, time.monotonic() + 10.0):
            return False, "Nav2 接收目标超时"
        self.goal_handle = send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.goal_handle = None
            return False, "Nav2 拒绝了目标点"

        result_future = self.goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and not result_future.done():
            if self.cancel_event.is_set():
                self._cancel_active_goal("正在取消当前导航目标…")
                self._wait_future(result_future, time.monotonic() + 5.0)
                self.goal_handle = None
                return False, "任务已取消"
            if time.monotonic() >= deadline:
                self._cancel_active_goal(f"目标点超时（{timeout_seconds:.0f} 秒），正在停止")
                self._wait_future(result_future, time.monotonic() + 5.0)
                self.goal_handle = None
                return False, f"目标点导航超过 {timeout_seconds:.0f} 秒"
            rclpy.spin_once(self.node, timeout_sec=0.1)
            self._write()

        response = result_future.result() if result_future.done() else None
        self.goal_handle = None
        if response and response.status == GoalStatus.STATUS_SUCCEEDED:
            self.leg_progress = 1.0
            self.feedback["distance_remaining"] = 0.0
            self._write(force=True)
            return True, "已到达"
        if response and response.status == GoalStatus.STATUS_CANCELED:
            return False, "任务已取消"
        status = getattr(response, "status", GoalStatus.STATUS_UNKNOWN)
        return False, f"Nav2 导航失败（状态码 {status}）"

    def run(self) -> int:
        if not self.waypoints:
            return self._finish("failed", "任务没有目标点", "任务没有目标点")
        rclpy.init(args=None)
        self.node = Node(f"map_manager_waypoint_{os.getpid()}")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        try:
            if not self._wait_server(float(self.mission.get("server_timeout_sec", 20.0))):
                if self.cancel_event.is_set():
                    return self._finish("cancelled", "导航任务已取消")
                return self._finish(
                    "failed",
                    "无法连接 Nav2 导航服务",
                    "请确认 Web 中的导航流程已经成功启动",
                )

            waypoint_timeout = float(self.mission.get("waypoint_timeout_sec", 300.0))
            pause_seconds = float(self.mission.get("pause_between_sec", 0.0))
            stop_on_failure = bool(self.mission.get("stop_on_failure", True))
            for index, waypoint in enumerate(self.waypoints):
                if self.cancel_event.is_set():
                    return self._finish("cancelled", "导航任务已取消")
                self.current_index = index
                self.current_waypoint = waypoint
                succeeded, message = self._navigate_one(waypoint, waypoint_timeout)
                if self.cancel_event.is_set() or message == "任务已取消":
                    return self._finish("cancelled", "导航任务已取消")
                self.processed += 1
                if succeeded:
                    self.completed += 1
                else:
                    self.failed_count += 1
                self.results.append({
                    "waypoint_id": waypoint.get("id"),
                    "waypoint_name": waypoint.get("name"),
                    "index": index,
                    "succeeded": succeeded,
                    "message": message,
                    "finished_at": now_iso(),
                })
                self.leg_progress = 0.0
                self.last_progress = self.processed * 100.0 / len(self.waypoints)
                if not succeeded:
                    if stop_on_failure:
                        return self._finish("failed", message, message)
                self._write(force=True, message=message)

                if index + 1 < len(self.waypoints) and pause_seconds > 0:
                    pause_deadline = time.monotonic() + pause_seconds
                    self._write(
                        force=True,
                        status="running",
                        message=f"已处理 {index + 1}/{len(self.waypoints)}，短暂停留…",
                    )
                    while time.monotonic() < pause_deadline:
                        if self.cancel_event.wait(timeout=min(0.1, pause_deadline - time.monotonic())):
                            return self._finish("cancelled", "导航任务已取消")
                        self._write()

            if self.failed_count:
                return self._finish(
                    "partial",
                    f"任务完成：成功 {self.completed} 个，失败 {self.failed_count} 个",
                )
            return self._finish("succeeded", f"已完成全部 {self.completed} 个目标点")
        except Exception as exc:
            return self._finish("failed", "导航任务异常终止", str(exc))
        finally:
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def capture_pose(output: Path, timeout_seconds: float) -> int:
    rclpy.init(args=None)
    node = Node(f"map_manager_pose_capture_{os.getpid()}")
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    try:
        pose = lookup_pose(node, tf_buffer, timeout_seconds)
        write_json_atomic(output, pose)
        return 0
    except Exception as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        del listener
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Map manager waypoint ROS worker")
    subparsers = parser.add_subparsers(dest="command", required=True)
    pose_parser = subparsers.add_parser("pose", help="Capture map -> base_link TF")
    pose_parser.add_argument("--output", type=Path, required=True)
    pose_parser.add_argument("--timeout", type=float, default=5.0)
    navigate_parser = subparsers.add_parser("navigate", help="Run a multi-goal mission")
    navigate_parser.add_argument("--mission", type=Path, required=True)
    navigate_parser.add_argument("--status", type=Path, required=True)
    arguments = parser.parse_args()

    if arguments.command == "pose":
        return capture_pose(arguments.output, max(0.5, min(arguments.timeout, 30.0)))

    runner = MissionRunner(arguments.mission, arguments.status)
    signal.signal(signal.SIGINT, runner.request_cancel)
    signal.signal(signal.SIGTERM, runner.request_cancel)
    return runner.run()


if __name__ == "__main__":
    raise SystemExit(main())

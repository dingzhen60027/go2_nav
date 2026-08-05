from __future__ import annotations

import os
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


# Processes outside the workspace install tree must be listed explicitly.  Keep
# this narrow: the emergency cleanup endpoint must never become a generic
# "pkill ros2" that can terminate the Web server or unrelated user programs.
PROJECT_EXECUTABLES = {
    "bt_navigator": "Nav2 BT navigator",
    "component_container_isolated": "Nav2 component container",
    "controller_server": "Nav2 controller",
    "behavior_server": "Nav2 behavior server",
    "ekf_node": "localization EKF",
    "fast_icp_loc_node": "pure ICP localization",
    "fused_icp_matcher": "fused ICP matcher",
    "go2_bridge": "Go2 command bridge",
    "go2_cmd_adapter": "Go2 velocity adapter",
    "icp_fusion_bridge": "ICP fusion bridge",
    "lifecycle_manager": "Nav2 lifecycle manager",
    "lio_node": "FAST-LIO2 mapping",
    "livox_ros_driver2_node": "Livox driver",
    "map_capture_node": "FAST-LIO2 map capture",
    "map_server": "Nav2 map server",
    "map_saver_cli": "Nav2 map saver",
    "neutral_joint_state_publisher": "Go2 neutral joint publisher",
    "pcd2pgm_node": "PCD map converter",
    "planner_server": "Nav2 planner",
    "pointcloud_cluster_tool": "point cloud cluster tool",
    "run_mapping_online": "FASTer-LIO mapping",
    "smoother_server": "Nav2 path smoother",
    "sport_state_adapter": "Go2 state adapter",
    "static_transform_publisher": "legacy static TF",
    "velocity_smoother": "Nav2 velocity smoother",
    "waypoint_follower": "Nav2 waypoint follower",
}

PROJECT_LAUNCH_PACKAGES = {
    "fast_icp_loc",
    "faster_lio",
    "go2_description",
    "go2_localization",
    "go2_mapping",
    "livox_ros_driver2",
    "nav2_bringup",
}

PROJECT_START_SCRIPTS = {
    "start_fused_localization.sh",
    "start_fused_navigation.sh",
    "start_localization.sh",
    "start_mapping.sh",
    "start_mapping_fastlio2.sh",
    "start_navigation.sh",
}


@dataclass(frozen=True)
class ProcessTarget:
    pid: int
    start_time: str
    label: str
    command: tuple[str, ...]

    @property
    def identity(self) -> tuple[int, str]:
        return self.pid, self.start_time


def _basename(value: str) -> str:
    return Path(value).name


def _ros2_launch_package(command: tuple[str, ...]) -> str | None:
    for index, token in enumerate(command[:-2]):
        if _basename(token) == "ros2" and command[index + 1] == "launch":
            return command[index + 2]
    return None


def classify_project_process(
    command: Iterable[str],
    executable: str,
    repo_root: Path,
) -> str | None:
    """Return a human-readable label only for this workspace's ROS stack."""

    argv = tuple(command)
    if not argv:
        return None
    repo = str(repo_root.resolve())
    install_prefix = f"{repo}{os.sep}install{os.sep}"
    executable_path = executable or argv[0]
    name = _basename(argv[0])
    executable_name = _basename(executable_path)

    # Anything executed from this workspace's install tree is a project ROS
    # binary. Python console scripts have the installed path in argv[1], while
    # compiled ROS binaries normally expose it in argv[0]. Never classify the
    # generic interpreter (python/node/npm) itself as a project process.
    installed_paths = [
        token
        for token in (executable_path, argv[0])
        if token.startswith(install_prefix)
    ]
    if name in {"python", "python3", "python3.10"} and len(argv) > 1:
        if argv[1].startswith(install_prefix):
            installed_paths.append(argv[1])
    if installed_paths:
        installed_name = _basename(installed_paths[-1])
        generic_names = {
            "bash", "node", "npm", "python", "python3", "python3.10", "sh"
        }
        if installed_name not in generic_names:
            return PROJECT_EXECUTABLES.get(installed_name, installed_name)

    launch_package = _ros2_launch_package(argv)
    if launch_package in PROJECT_LAUNCH_PACKAGES:
        return f"{launch_package} launch"

    for token in argv:
        candidate = Path(token)
        if candidate.name in PROJECT_START_SCRIPTS and str(candidate).startswith(repo):
            return candidate.name
        if candidate.name == "waypoint_executor.py" and str(candidate).startswith(repo):
            return "Web waypoint executor"

    # These exact executable names are the ROS/Nav2 processes used by the
    # workspace. Extra guards keep generic system nodes from being selected.
    selected_name = name if name in PROJECT_EXECUTABLES else executable_name
    if selected_name not in PROJECT_EXECUTABLES:
        if selected_name == "rviz2":
            joined = " ".join(argv)
            if repo in joined or any(
                marker in joined
                for marker in ("go2_description_rviz", "fused_localization_rviz")
            ):
                return "project RViz"
        if selected_name == "robot_state_publisher":
            joined = " ".join(argv)
            if "go2_robot_state_publisher" in joined:
                return "Go2 robot state publisher"
        return None

    joined = " ".join(argv)
    if selected_name == "component_container_isolated" and "nav2_container" not in joined:
        return None
    if selected_name == "ekf_node" and not any(
        marker in joined for marker in ("ekf_local", "ekf_global")
    ):
        return None
    return PROJECT_EXECUTABLES[selected_name]


def _read_process(pid: int, repo_root: Path) -> ProcessTarget | None:
    proc = Path("/proc") / str(pid)
    try:
        if proc.stat().st_uid != os.getuid():
            return None
        raw_command = (proc / "cmdline").read_bytes()
        command = tuple(
            item.decode(errors="replace")
            for item in raw_command.split(b"\0")
            if item
        )
        executable = os.readlink(proc / "exe")
        # Field 22 is process start time. The parenthesized comm field may
        # contain spaces, so split only after the final closing parenthesis.
        stat_tail = (proc / "stat").read_text(encoding="ascii").rsplit(")", 1)[1]
        start_time = stat_tail.split()[19]
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError, IndexError):
        return None
    label = classify_project_process(command, executable, repo_root)
    if label is None:
        return None
    return ProcessTarget(pid=pid, start_time=start_time, label=label, command=command)


def discover_project_processes(
    repo_root: Path,
    *,
    excluded_pids: Iterable[int] = (),
) -> list[ProcessTarget]:
    excluded = {int(pid) for pid in excluded_pids}
    targets: list[ProcessTarget] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in excluded:
            continue
        target = _read_process(pid, repo_root)
        if target is not None:
            targets.append(target)
    return sorted(targets, key=lambda target: target.pid)


def _ancestor_pids(pid: int) -> set[int]:
    ancestors = {pid}
    current = pid
    while current > 1:
        try:
            status = (Path("/proc") / str(current) / "status").read_text(
                encoding="utf-8"
            )
            parent_line = next(
                line for line in status.splitlines() if line.startswith("PPid:")
            )
            parent = int(parent_line.split()[1])
        except (FileNotFoundError, PermissionError, StopIteration, ValueError, OSError):
            break
        if parent <= 0 or parent in ancestors:
            break
        ancestors.add(parent)
        current = parent
    return ancestors


def cleanup_project_processes(repo_root: Path) -> dict[str, object]:
    """Stop project ROS processes while preserving this Web server."""

    excluded = _ancestor_pids(os.getpid())
    seen: dict[tuple[int, str], ProcessTarget] = {}
    signals_used: list[str] = []

    for stop_signal, wait_seconds in (
        (signal.SIGINT, 2.0),
        (signal.SIGTERM, 1.5),
        (signal.SIGKILL, 0.5),
    ):
        targets = discover_project_processes(repo_root, excluded_pids=excluded)
        if not targets:
            break
        signals_used.append(signal.Signals(stop_signal).name)
        for target in targets:
            seen[target.identity] = target
            # Verify PID identity immediately before signalling so PID reuse
            # cannot redirect the cleanup to a new, unrelated process.
            current = _read_process(target.pid, repo_root)
            if current is None or current.identity != target.identity:
                continue
            try:
                os.kill(target.pid, stop_signal)
            except (ProcessLookupError, PermissionError):
                continue

        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline:
            remaining_ids = {
                target.identity
                for target in discover_project_processes(
                    repo_root, excluded_pids=excluded
                )
            }
            if not remaining_ids:
                break
            time.sleep(0.05)

    remaining = discover_project_processes(repo_root, excluded_pids=excluded)
    remaining_ids = {target.identity for target in remaining}
    stopped = [target for identity, target in seen.items() if identity not in remaining_ids]
    return {
        "stopped_count": len(stopped),
        "stopped": [
            {"pid": target.pid, "label": target.label}
            for target in sorted(stopped, key=lambda item: item.pid)
        ],
        "remaining_count": len(remaining),
        "remaining": [
            {"pid": target.pid, "label": target.label}
            for target in remaining
        ],
        "signals_used": signals_used,
    }

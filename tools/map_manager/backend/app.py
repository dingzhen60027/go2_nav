from __future__ import annotations

import csv
import hashlib
import io
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

import yaml
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from PIL import Image, ImageDraw
from pydantic import BaseModel, Field

from .process_cleanup import cleanup_project_processes
from .runtime import RuntimeManager
from .waypoints import ACTIVE_MISSION_STATES, WaypointMissionManager, WaypointStore


REPO_ROOT = Path(os.environ.get("GO2_NAV_ROOT", Path(__file__).resolve().parents[3])).resolve()
MAPS_ROOT = Path(os.environ.get("GO2_MAPS_ROOT", REPO_ROOT / "maps")).resolve()
WORKSPACE_ROOT = MAPS_ROOT / "workspace"
SESSIONS_ROOT = WORKSPACE_ROOT / "sessions"
VERSIONS_ROOT = WORKSPACE_ROOT / "versions"
TRASH_ROOT = WORKSPACE_ROOT / "trash"
STATE_PATH = WORKSPACE_ROOT / "state.json"
RUNTIME_STATE_PATH = WORKSPACE_ROOT / "runtime.json"
BUILD_PROFILES_PATH = WORKSPACE_ROOT / "build_profiles.yaml"
WAYPOINTS_PATH = WORKSPACE_ROOT / "waypoints.yaml"
WAYPOINT_NAV_ROOT = WORKSPACE_ROOT / "waypoint_navigation"
ACTIVE_LINK = MAPS_ROOT / "active"
CURRENT_RAW_PCD = REPO_ROOT / "src" / "faster-lio" / "PCD" / "scans.pcd"
LEGACY_ICP_PCD = MAPS_ROOT / "clean" / "pcd_icp_latest.pcd"
ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{2,80}$")

for directory in (SESSIONS_ROOT, VERSIONS_ROOT, TRASH_ROOT):
    directory.mkdir(parents=True, exist_ok=True)

class ArchiveRequest(BaseModel):
    name: str = Field(min_length=1, max_length=80)
    site: str = Field(default="default", min_length=1, max_length=80)
    note: str = Field(default="", max_length=500)


class BuildParameters(BaseModel):
    statistical_mean_k: int = Field(default=20, ge=2, le=500)
    statistical_std_dev_mul: float = Field(default=0.5, ge=0.05, le=10.0)
    radius: float = Field(default=0.3, ge=0.01, le=5.0)
    radius_min_points: int = Field(default=4, ge=1, le=500)
    voxel_leaf: float = Field(default=0.15, ge=0.05, le=1.0)
    z_min: float = Field(default=0.4, ge=-10.0, le=10.0)
    z_max: float = Field(default=1.5, ge=-10.0, le=10.0)
    resolution: float = Field(default=0.05, ge=0.01, le=0.5)


class BuildRequest(BuildParameters):
    session_id: str
    name: str = Field(min_length=1, max_length=80)
    note: str = Field(default="", max_length=500)


class BuildProfileRequest(BuildParameters):
    name: str = Field(min_length=1, max_length=80)


class AdoptRequest(BaseModel):
    name: str = Field(default="现有导航地图", min_length=1, max_length=80)
    note: str = Field(default="从 legacy latest 文件纳管，需实机验证", max_length=500)


class ActivateRequest(BaseModel):
    version_id: str


class RuntimeRequest(BaseModel):
    mode: Literal["mapping", "localization", "navigation"]
    algorithm: Literal[
        "faster_lio",
        "fastlio2",
        "pure_icp",
        "fused_ekf",
    ] | None = None


class TrashRequest(BaseModel):
    kind: Literal["version", "session"]
    item_id: str


class RenameRequest(BaseModel):
    name: str = Field(min_length=1, max_length=80)


class MapEditPoint(BaseModel):
    x: float = Field(ge=0.0, le=100000.0)
    y: float = Field(ge=0.0, le=100000.0)


class MapEditOperation(BaseModel):
    mode: Literal["occupied", "free", "unknown"]
    shape: Literal["brush", "line"]
    size: int = Field(ge=1, le=100)
    points: list[MapEditPoint] = Field(min_length=1, max_length=10000)


class EditMapRequest(BaseModel):
    name: str = Field(min_length=1, max_length=80)
    note: str = Field(default="", max_length=500)
    operations: list[MapEditOperation] = Field(min_length=1, max_length=500)


class PurgeTrashRequest(BaseModel):
    trash_ids: list[str] = Field(min_length=1, max_length=500)


class ClusterRequest(BaseModel):
    voxel_leaf: float = Field(default=0.15, ge=0.05, le=0.50)
    tolerance: float = Field(default=0.30, ge=0.05, le=2.0)
    min_voxels: int = Field(default=5, ge=2, le=10000)
    z_min: float = Field(default=0.20, ge=-10.0, le=10.0)
    z_max: float = Field(default=2.20, ge=-10.0, le=10.0)


class ClusterApplyRequest(BaseModel):
    cluster_run_id: str
    cluster_ids: list[int] = Field(min_length=1, max_length=500)
    name: str = Field(min_length=1, max_length=80)
    note: str = Field(default="", max_length=500)


class CaptureWaypointRequest(BaseModel):
    name: str | None = Field(default=None, max_length=80)


class RenameWaypointRequest(BaseModel):
    name: str = Field(min_length=1, max_length=80)


class ReorderWaypointsRequest(BaseModel):
    waypoint_ids: list[str] = Field(min_length=1, max_length=100)


class WaypointMissionRequest(BaseModel):
    stop_on_failure: bool = True
    waypoint_timeout_sec: float = Field(default=300.0, ge=30.0, le=3600.0)
    pause_between_sec: float = Field(default=0.0, ge=0.0, le=60.0)


BUILD_PARAMETER_FIELDS = (
    "statistical_mean_k",
    "statistical_std_dev_mul",
    "radius",
    "radius_min_points",
    "voxel_leaf",
    "z_min",
    "z_max",
    "resolution",
)


job_lock = threading.Lock()
process_lock = threading.Lock()
session_preview_lock = threading.Lock()
cluster_lock = threading.Lock()
process_cleanup_lock = threading.Lock()
managed_processes: set[subprocess.Popen[Any]] = set()
job_cancel_event = threading.Event()
job_thread: threading.Thread | None = None
job_state: dict[str, Any] = {
    "running": False,
    "id": None,
    "kind": None,
    "progress": 0,
    "stage": "idle",
    "logs": [],
    "error": None,
    "result": None,
}
runtime_manager = RuntimeManager(REPO_ROOT, RUNTIME_STATE_PATH)
waypoint_store = WaypointStore(WAYPOINTS_PATH)
waypoint_mission_manager = WaypointMissionManager(REPO_ROOT, WAYPOINT_NAV_ROOT)


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def make_id(prefix: str) -> str:
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    suffix = hashlib.sha1(os.urandom(12)).hexdigest()[:6]
    return f"{prefix}-{stamp}-{suffix}"


def require_id(value: str) -> str:
    if not ID_RE.fullmatch(value):
        raise HTTPException(status_code=400, detail="Invalid identifier")
    return value


def human_size(size: int) -> str:
    value = float(size)
    for unit in ("B", "KB", "MB", "GB"):
        if value < 1024 or unit == "GB":
            return f"{value:.1f} {unit}" if unit != "B" else f"{int(value)} B"
        value /= 1024
    return f"{value:.1f} GB"


def read_state() -> dict[str, Any]:
    default = {"active_id": None, "previous_active_id": None, "archived": []}
    if not STATE_PATH.exists():
        return default
    try:
        return {**default, **json.loads(STATE_PATH.read_text(encoding="utf-8"))}
    except (OSError, json.JSONDecodeError):
        return default


def write_json_atomic(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(temp, path)


def write_state(data: dict[str, Any]) -> None:
    write_json_atomic(STATE_PATH, data)


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream) or {}
    return value if isinstance(value, dict) else {}


def save_yaml(path: Path, data: dict[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def save_yaml_atomic(path: Path, data: dict[str, Any]) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    os.replace(temp, path)


def build_parameter_values(parameters: BuildParameters) -> dict[str, Any]:
    return {
        field: getattr(parameters, field)
        for field in BUILD_PARAMETER_FIELDS
    }


def list_build_profiles() -> list[dict[str, Any]]:
    if not BUILD_PROFILES_PATH.is_file():
        return []
    try:
        stored = load_yaml(BUILD_PROFILES_PATH).get("profiles", [])
    except (OSError, yaml.YAMLError):
        return []
    profiles = []
    for value in stored if isinstance(stored, list) else []:
        if not isinstance(value, dict):
            continue
        profile_id = str(value.get("id", ""))
        name = str(value.get("name", "")).strip()
        if not ID_RE.fullmatch(profile_id) or not name:
            continue
        try:
            parameters = BuildParameters(**(value.get("parameters") or {}))
        except ValueError:
            continue
        profiles.append({
            "id": profile_id,
            "name": name,
            "created_at": value.get("created_at"),
            "updated_at": value.get("updated_at"),
            "parameters": build_parameter_values(parameters),
        })
    return profiles


def save_build_profile(request: BuildProfileRequest) -> dict[str, Any]:
    name = request.name.strip()
    if not name:
        raise HTTPException(status_code=400, detail="方案名称不能为空")
    if request.z_min >= request.z_max:
        raise HTTPException(status_code=400, detail="高度切片下限必须小于上限")

    profiles = list_build_profiles()
    profile = next(
        (item for item in profiles if item["name"].casefold() == name.casefold()),
        None,
    )
    created = profile is None
    if profile is None:
        profile = {
            "id": make_id("profile"),
            "created_at": now_iso(),
        }
        profiles.append(profile)
    profile.update({
        "name": name,
        "updated_at": now_iso(),
        "parameters": build_parameter_values(request),
    })
    save_yaml_atomic(BUILD_PROFILES_PATH, {"schema": 1, "profiles": profiles})
    return {"profile": profile, "created": created}


def rename_metadata_item(
    item_id: str,
    request: RenameRequest,
    *,
    kind: Literal["version", "session"],
) -> dict[str, Any]:
    item_id = require_id(item_id)
    if kind == "version":
        item_path = version_dir(item_id)
        metadata_path = item_path / "manifest.yaml"
    else:
        item_path = session_dir(item_id)
        metadata_path = item_path / "session.yaml"
    if not item_path.is_dir() or not metadata_path.is_file():
        raise HTTPException(status_code=404, detail="要重命名的项目不存在")

    name = request.name.strip()
    if not name:
        raise HTTPException(status_code=400, detail="名称不能为空")
    try:
        metadata = load_yaml(metadata_path)
        metadata["name"] = name
        save_yaml_atomic(metadata_path, metadata)
    except (OSError, yaml.YAMLError) as exc:
        raise HTTPException(status_code=500, detail="项目名称保存失败") from exc
    return {"kind": kind, "item_id": item_id, "name": name}


def file_info(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    stat = path.stat()
    return {
        "path": str(path),
        "name": path.name,
        "size": stat.st_size,
        "size_human": human_size(stat.st_size),
        "modified_at": datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(timespec="seconds"),
    }


def capture_identity(path: Path) -> dict[str, int] | None:
    try:
        stat = path.stat()
    except (FileNotFoundError, OSError):
        return None
    return {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}


def runtime_output_root() -> Path:
    return WORKSPACE_ROOT / "runtime"


def is_safe_capture_path(path: Path) -> bool:
    absolute = path.absolute()
    if absolute == CURRENT_RAW_PCD.absolute():
        return True
    root = runtime_output_root().resolve()
    try:
        absolute.resolve(strict=False).relative_to(root)
        return True
    except (OSError, ValueError):
        return False


def session_dir(session_id: str) -> Path:
    return SESSIONS_ROOT / require_id(session_id)


def version_dir(version_id: str) -> Path:
    return VERSIONS_ROOT / require_id(version_id)


def trash_dir(trash_id: str) -> Path:
    return TRASH_ROOT / require_id(trash_id)


def validate_version(path: Path) -> list[str]:
    required = {
        "manifest": path / "manifest.yaml",
        "2D YAML": path / "map.yaml",
        "2D PGM": path / "map.pgm",
        "ICP PCD": path / "localization.pcd",
    }
    issues = [f"缺少 {label}" for label, file_path in required.items() if not file_path.exists()]
    yaml_path = path / "map.yaml"
    if yaml_path.exists():
        try:
            image = load_yaml(yaml_path).get("image")
            if not image:
                issues.append("map.yaml 未声明 image")
            elif Path(str(image)).name != "map.pgm":
                issues.append("map.yaml 未引用同版本 map.pgm")
        except OSError:
            issues.append("map.yaml 无法读取")
    pgm_path = path / "map.pgm"
    if pgm_path.exists():
        try:
            with Image.open(pgm_path) as occupancy_image:
                grayscale = occupancy_image.convert("L")
                width, height = grayscale.size
                extrema = grayscale.getextrema()
            if width < 2 or height < 2:
                issues.append(f"map.pgm 尺寸异常（{width}x{height}）")
            elif extrema is None or extrema[0] == extrema[1]:
                issues.append("map.pgm 没有有效的占据/自由空间差异")
        except (OSError, ValueError):
            issues.append("map.pgm 无法读取")
    return issues


def list_sessions() -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for path in sorted(SESSIONS_ROOT.iterdir(), reverse=True):
        if not path.is_dir() or not ID_RE.fullmatch(path.name):
            continue
        metadata_path = path / "session.yaml"
        metadata = load_yaml(metadata_path) if metadata_path.exists() else {}
        raw = path / "raw.pcd"
        items.append({
            "id": path.name,
            "name": metadata.get("name", path.name),
            "site": metadata.get("site", "default"),
            "note": metadata.get("note", ""),
            "algorithm": metadata.get("algorithm", "unknown"),
            "origin": metadata.get("origin", "mapping"),
            "source_session": metadata.get("source_session"),
            "removed_clusters": metadata.get("removed_clusters", []),
            "created_at": metadata.get("created_at"),
            "raw": file_info(raw),
            "complete": raw.exists(),
            "cloud_preview_url": f"/api/sessions/{path.name}/cloud.ply" if raw.exists() else None,
        })
    return items


def list_versions() -> list[dict[str, Any]]:
    state = read_state()
    archived = set(state.get("archived", []))
    items: list[dict[str, Any]] = []
    for path in sorted(VERSIONS_ROOT.iterdir(), reverse=True):
        if not path.is_dir() or path.name.startswith(".") or not ID_RE.fullmatch(path.name):
            continue
        manifest_path = path / "manifest.yaml"
        manifest = load_yaml(manifest_path) if manifest_path.exists() else {}
        issues = validate_version(path)
        if path.name == state.get("active_id"):
            status = "active"
        elif path.name in archived:
            status = "archived"
        else:
            status = "candidate"
        total_size = sum(f.stat().st_size for f in path.rglob("*") if f.is_file())
        items.append({
            "id": path.name,
            "name": manifest.get("name", path.name),
            "site": manifest.get("site", "default"),
            "note": manifest.get("note", ""),
            "created_at": manifest.get("created_at"),
            "source_session": manifest.get("source_session"),
            "parent_version": manifest.get("parent_version"),
            "mapping_algorithm": manifest.get("mapping_algorithm", "unknown"),
            "parameters": manifest.get("parameters", {}),
            "edit_summary": manifest.get("edit_summary"),
            "origin": manifest.get("origin", "managed"),
            "status": status,
            "complete": not issues,
            "issues": issues,
            "size": total_size,
            "size_human": human_size(total_size),
            "paths": {
                "root": str(path),
                "map_yaml": str(path / "map.yaml"),
                "map_pgm": str(path / "map.pgm"),
                "localization_pcd": str(path / "localization.pcd"),
            },
            "map_preview_url": f"/api/versions/{path.name}/map.png",
            "cloud_preview_url": (
                f"/api/versions/{path.name}/cloud.ply" if (path / "preview.ply").exists() else None
            ),
        })
    return items


def list_legacy_maps() -> list[dict[str, Any]]:
    items = []
    for yaml_path in sorted(MAPS_ROOT.glob("map*.yaml"), reverse=True):
        if yaml_path.is_symlink():
            continue
        try:
            config = load_yaml(yaml_path)
        except OSError:
            config = {}
        image_value = config.get("image", f"{yaml_path.stem}.pgm")
        pgm_path = (yaml_path.parent / str(image_value)).resolve()
        items.append({
            "id": yaml_path.stem,
            "name": yaml_path.stem,
            "yaml": file_info(yaml_path),
            "pgm": file_info(pgm_path),
            "complete_2d": pgm_path.exists(),
            "map_preview_url": f"/api/legacy/{yaml_path.stem}/map.png" if pgm_path.exists() else None,
        })
    return items


def list_trash() -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for path in sorted(TRASH_ROOT.iterdir(), reverse=True):
        if not path.is_dir() or not ID_RE.fullmatch(path.name):
            continue
        metadata_path = path / ".trash.json"
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        kind = metadata.get("kind")
        source_meta_path = path / ("manifest.yaml" if kind == "version" else "session.yaml")
        source_meta = load_yaml(source_meta_path) if source_meta_path.exists() else {}
        total_size = sum(file.stat().st_size for file in path.rglob("*") if file.is_file())
        items.append({
            "id": path.name,
            "kind": kind,
            "original_id": metadata.get("original_id"),
            "name": source_meta.get("name", metadata.get("original_id", path.name)),
            "deleted_at": metadata.get("deleted_at"),
            "size": total_size,
            "size_human": human_size(total_size),
        })
    return items


def current_capture() -> dict[str, Any]:
    runtime = runtime_manager.snapshot()
    configured_path = runtime.get("capture_path") if runtime.get("mode") == "mapping" else None
    source_path = Path(configured_path) if configured_path else CURRENT_RAW_PCD
    if not is_safe_capture_path(source_path):
        source_path = CURRENT_RAW_PCD
    info = file_info(source_path)
    identity = capture_identity(source_path) if info else None
    resolved_name = source_path.resolve().name if info else ""
    algorithm = runtime.get("algorithm") if runtime.get("mode") == "mapping" else None
    if not algorithm:
        algorithm = "fastlio2" if resolved_name.startswith("fastlio2_scans_") else "faster_lio"
    archived_session_id = runtime.get("capture_session_id") if runtime.get("capture_status") == "archived" else None
    if info:
        for session_path in SESSIONS_ROOT.iterdir():
            raw = session_path / "raw.pcd"
            if not raw.exists():
                continue
            metadata_path = session_path / "session.yaml"
            metadata = load_yaml(metadata_path) if metadata_path.exists() else {}
            same_run = not runtime.get("run_id") or metadata.get("run_id") == runtime.get("run_id")
            if metadata.get("source_identity") == identity and same_run:
                archived_session_id = session_path.name
                break
            raw_identity = capture_identity(raw)
            if not runtime.get("run_id") and not metadata.get("source_identity") and raw_identity == identity:
                archived_session_id = session_path.name
                break
    new_for_run = bool(
        info
        and runtime.get("mode") == "mapping"
        and runtime.get("run_id")
        and runtime.get("capture_status") == "pending"
        and identity != runtime.get("capture_baseline")
    )
    return {
        "available": info is not None,
        "file": info,
        "source_path": str(source_path),
        "algorithm": algorithm,
        "archived": archived_session_id is not None,
        "archived_session_id": archived_session_id,
        "run_id": runtime.get("run_id"),
        "new_for_run": new_for_run,
        "source_identity": identity,
    }


def remove_capture_output(capture: dict[str, Any]) -> None:
    runtime = runtime_manager.snapshot()
    capture_dir_value = runtime.get("capture_dir")
    if capture_dir_value:
        capture_dir = Path(capture_dir_value)
        root = runtime_output_root().resolve()
        try:
            capture_dir.resolve(strict=False).relative_to(root)
        except (OSError, ValueError) as exc:
            raise RuntimeError("拒绝清理工作区之外的建图目录") from exc
        shutil.rmtree(capture_dir, ignore_errors=False)
        return

    source = Path(capture["source_path"])
    if source.absolute() != CURRENT_RAW_PCD.absolute():
        raise RuntimeError("拒绝清理未纳管的建图文件")
    resolved = source.resolve(strict=False)
    if source.is_symlink() or source.exists():
        source.unlink()
    if resolved != source.absolute() and resolved.parent == CURRENT_RAW_PCD.parent.resolve() and resolved.exists():
        resolved.unlink()


def archive_capture_output(capture: dict[str, Any], request: ArchiveRequest) -> dict[str, Any]:
    """Copy one completed runtime capture into the persistent session store."""

    if not capture["available"]:
        raise HTTPException(status_code=404, detail="本次建图没有生成可归档的 PCD")
    if capture["archived"]:
        raise HTTPException(status_code=409, detail="当前建图结果已经归档")
    if not capture["new_for_run"]:
        raise HTTPException(status_code=409, detail="检测到的是旧 PCD，不属于本次建图")
    source_path = Path(capture["source_path"])
    if not is_safe_capture_path(source_path):
        raise HTTPException(status_code=409, detail="建图结果不在受管目录中")

    session_id = make_id("session")
    target = session_dir(session_id)
    temp = SESSIONS_ROOT / f".{session_id}.copying"
    temp.mkdir(parents=True, exist_ok=False)
    try:
        shutil.copy2(source_path, temp / "raw.pcd")
        save_yaml(temp / "session.yaml", {
            "schema": 1,
            "id": session_id,
            "name": request.name,
            "site": request.site,
            "note": request.note,
            "created_at": now_iso(),
            "source_path": str(source_path),
            "source_identity": capture["source_identity"],
            "run_id": capture["run_id"],
            "algorithm": capture["algorithm"],
        })
        os.replace(temp, target)
    except Exception:
        shutil.rmtree(temp, ignore_errors=True)
        raise

    runtime_manager.mark_capture("archived", session_id)
    cleanup_warning = None
    if runtime_manager.snapshot().get("capture_dir"):
        try:
            remove_capture_output(capture)
        except (OSError, RuntimeError) as exc:
            cleanup_warning = str(exc)
    return {"session_id": session_id, "cleanup_warning": cleanup_warning}


def auto_archive_pending_capture(reason: str) -> dict[str, Any] | None:
    """Preserve a completed PCD before another workflow replaces runtime state."""

    capture = current_capture()
    if not capture["new_for_run"] or capture["archived"]:
        return None
    runtime = runtime_manager.snapshot()
    algorithm_name = "FAST-LIO2" if capture["algorithm"] == "fastlio2" else "FASTer-LIO"
    started_at = str(runtime.get("started_at") or now_iso()).replace("T", " ")[:19]
    return archive_capture_output(
        capture,
        ArchiveRequest(
            name=f"{algorithm_name} 自动保存 {started_at}",
            site="default",
            note=f"{reason}；系统自动归档已落盘的原始 PCD",
        ),
    )


def job_snapshot() -> dict[str, Any]:
    with job_lock:
        return {**job_state, "logs": list(job_state["logs"][-120:])}


def set_job(**changes: Any) -> None:
    with job_lock:
        job_state.update(changes)


def log_job(message: str, *, progress: int | None = None, stage: str | None = None) -> None:
    stamp = datetime.now().strftime("%H:%M:%S")
    with job_lock:
        job_state["logs"].append(f"[{stamp}] {message}")
        if progress is not None:
            job_state["progress"] = progress
        if stage is not None:
            job_state["stage"] = stage


def start_managed_process(command: list[str], **kwargs: Any) -> subprocess.Popen[Any]:
    runner = Path(__file__).with_name("process_runner.py")
    process = subprocess.Popen(
        [sys.executable, str(runner), *command],
        cwd=REPO_ROOT,
        start_new_session=True,
        **kwargs,
    )
    with process_lock:
        managed_processes.add(process)
    return process


def terminate_managed_process(process: subprocess.Popen[Any], interrupt_timeout: float = 5.0) -> None:
    if process.poll() is not None:
        return
    for stop_signal, wait_seconds in (
        (signal.SIGINT, interrupt_timeout),
        (signal.SIGTERM, 3.0),
        (signal.SIGKILL, 1.0),
    ):
        try:
            os.killpg(process.pid, stop_signal)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=wait_seconds)
            break
        except subprocess.TimeoutExpired:
            continue


def unregister_process(process: subprocess.Popen[Any]) -> None:
    with process_lock:
        managed_processes.discard(process)


def terminate_all_managed_processes() -> None:
    """Stop converters/editors still owned by the Web backend."""

    with process_lock:
        processes = list(managed_processes)
    for process in processes:
        terminate_managed_process(process, interrupt_timeout=1.0)
        unregister_process(process)


def cancel_map_job() -> dict[str, Any]:
    if not job_snapshot()["running"]:
        return job_snapshot()
    job_cancel_event.set()
    log_job("正在取消并清理地图任务", stage="cancelling")
    with process_lock:
        processes = list(managed_processes)
    for process in processes:
        terminate_managed_process(process, interrupt_timeout=2.0)
    thread = job_thread
    if thread and thread is not threading.current_thread():
        thread.join(timeout=8.0)
    return job_snapshot()


def run_command(command: list[str], label: str, timeout: int = 180, track_job: bool = True) -> None:
    if track_job and job_cancel_event.is_set():
        raise RuntimeError("地图任务已取消")
    if track_job:
        log_job(label)
    process = start_managed_process(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    try:
        try:
            output, _ = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            terminate_managed_process(process)
            raise RuntimeError(f"{label}超时") from exc
    finally:
        unregister_process(process)
    if track_job:
        for line in output.splitlines()[-20:]:
            log_job(line)
    if process.returncode != 0:
        if track_job and job_cancel_event.is_set():
            raise RuntimeError("地图任务已取消")
        raise RuntimeError(f"{label}失败，退出码 {process.returncode}")


def generate_cloud_preview(
    source: Path,
    output: Path,
    leaf: float = 0.15,
    track_job: bool = True,
) -> None:
    temp_pcd = output.with_suffix(".preview.pcd")
    try:
        run_command(
            ["pcl_voxel_grid", str(source), str(temp_pcd), "-leaf", f"{leaf},{leaf},{leaf}"],
            "生成 3D 预览降采样",
            timeout=120,
            track_job=track_job,
        )
        run_command(
            ["pcl_pcd2ply", "-format", "1", "-use_camera", "0", str(temp_pcd), str(output)],
            "生成浏览器点云预览",
            timeout=120,
            track_job=track_job,
        )
    finally:
        temp_pcd.unlink(missing_ok=True)


def cluster_tool_path() -> Path:
    return REPO_ROOT / "install" / "go2_mapping" / "lib" / "go2_mapping" / "pointcloud_cluster_tool"


def cluster_run_dir(session_id: str, cluster_run_id: str) -> Path:
    return session_dir(session_id) / "clusters" / require_id(cluster_run_id)


def load_cluster_result(session_id: str, cluster_run_id: str) -> dict[str, Any]:
    path = cluster_run_dir(session_id, cluster_run_id)
    metadata_path = path / "metadata.json"
    if not metadata_path.is_file():
        raise HTTPException(status_code=404, detail="聚类结果不存在")
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise HTTPException(status_code=500, detail="聚类结果元数据损坏") from exc
    return {
        **metadata,
        "preview_url": f"/api/sessions/{session_id}/clusters/{cluster_run_id}/preview.ply",
    }


def parse_cluster_table(path: Path) -> list[dict[str, Any]]:
    clusters: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream, delimiter="\t"):
            cluster_id = int(row["id"])
            color = [int(row["r"]), int(row["g"]), int(row["b"])]
            clusters.append({
                "id": cluster_id,
                "voxel_count": int(row["voxel_count"]),
                "point_count": int(row["point_count"]),
                "centroid": [float(row["cx"]), float(row["cy"]), float(row["cz"])],
                "bounds": {
                    "min": [float(row["min_x"]), float(row["min_y"]), float(row["min_z"])],
                    "max": [float(row["max_x"]), float(row["max_y"]), float(row["max_z"])],
                },
                "color": color,
                "color_hex": "#" + "".join(f"{channel:02x}" for channel in color),
            })
    return clusters


def create_cluster_result(session_id: str, request: ClusterRequest) -> dict[str, Any]:
    source = session_dir(session_id)
    raw = source / "raw.pcd"
    if not raw.is_file():
        raise HTTPException(status_code=404, detail="原始 PCD 不存在")
    if request.z_min >= request.z_max:
        raise HTTPException(status_code=400, detail="聚类高度下限必须小于上限")
    if request.tolerance < request.voxel_leaf:
        raise HTTPException(status_code=400, detail="聚类距离不能小于体素尺寸")
    tool = cluster_tool_path()
    if not tool.is_file():
        raise HTTPException(status_code=500, detail="点云聚类工具尚未编译")

    cluster_run_id = make_id("cluster")
    root = source / "clusters"
    root.mkdir(exist_ok=True)
    temporary = root / f".{cluster_run_id}.building"
    final = root / cluster_run_id
    temporary.mkdir()
    table = temporary / "clusters.tsv"
    try:
        run_command(
            [
                str(tool), "cluster",
                "--input", str(raw),
                "--preview", str(temporary / "preview.ply"),
                "--voxels", str(temporary / "voxels.bin"),
                "--clusters", str(table),
                "--leaf", str(request.voxel_leaf),
                "--tolerance", str(request.tolerance),
                "--min-voxels", str(request.min_voxels),
                "--z-min", str(request.z_min),
                "--z-max", str(request.z_max),
            ],
            "原始 PCD 欧式聚类",
            timeout=180,
            track_job=False,
        )
        clusters = parse_cluster_table(table)
        metadata = {
            "schema": 1,
            "id": cluster_run_id,
            "session_id": session_id,
            "created_at": now_iso(),
            "source_identity": capture_identity(raw),
            "parameters": request.model_dump(),
            "clusters": clusters,
            "cluster_count": len(clusters),
        }
        write_json_atomic(temporary / "metadata.json", metadata)
        table.unlink(missing_ok=True)
        os.replace(temporary, final)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return load_cluster_result(session_id, cluster_run_id)


def apply_cluster_removal(
    session_id: str, request: ClusterApplyRequest
) -> dict[str, Any]:
    source = session_dir(session_id)
    raw = source / "raw.pcd"
    result = load_cluster_result(session_id, require_id(request.cluster_run_id))
    source_identity = capture_identity(raw)
    if source_identity != result.get("source_identity"):
        raise HTTPException(status_code=409, detail="原始 PCD 已发生变化，请重新聚类")
    available_ids = {int(cluster["id"]) for cluster in result["clusters"]}
    selected_ids = sorted(set(request.cluster_ids))
    if any(cluster_id not in available_ids for cluster_id in selected_ids):
        raise HTTPException(status_code=400, detail="选择中包含不存在的聚类")

    source_meta_path = source / "session.yaml"
    source_meta = load_yaml(source_meta_path) if source_meta_path.exists() else {}
    derived_id = make_id("session")
    temporary = SESSIONS_ROOT / f".{derived_id}.copying"
    final = session_dir(derived_id)
    temporary.mkdir()
    tool = cluster_tool_path()
    try:
        run_command(
            [
                str(tool), "filter",
                "--input", str(raw),
                "--output", str(temporary / "raw.pcd"),
                "--voxels", str(cluster_run_dir(session_id, request.cluster_run_id) / "voxels.bin"),
                "--remove", ",".join(str(value) for value in selected_ids),
            ],
            "生成聚类清理 PCD 副本",
            timeout=180,
            track_job=False,
        )
        generate_cloud_preview(
            temporary / "raw.pcd", temporary / "preview.ply", leaf=0.20, track_job=False
        )
        selected_clusters = [
            cluster for cluster in result["clusters"] if int(cluster["id"]) in selected_ids
        ]
        save_yaml(temporary / "session.yaml", {
            "schema": 1,
            "id": derived_id,
            "name": request.name,
            "site": source_meta.get("site", "default"),
            "note": request.note,
            "created_at": now_iso(),
            "algorithm": source_meta.get("algorithm", "unknown"),
            "origin": "cluster-cleaned",
            "source_session": session_id,
            "source_identity": result.get("source_identity"),
            "cluster_run_id": request.cluster_run_id,
            "cluster_parameters": result.get("parameters", {}),
            "removed_clusters": selected_ids,
            "removed_point_count": sum(cluster["point_count"] for cluster in selected_clusters),
        })
        if capture_identity(raw) != source_identity:
            raise RuntimeError("生成副本期间原始 PCD 发生变化，已取消输出")
        os.replace(temporary, final)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return {
        "session_id": derived_id,
        "source_session": session_id,
        "original_preserved": raw.is_file(),
        "removed_clusters": selected_ids,
        "session": next(item for item in list_sessions() if item["id"] == derived_id),
    }


def rewrite_map_yaml(source: Path, destination: Path) -> None:
    config = load_yaml(source)
    config["image"] = "map.pgm"
    save_yaml(destination, config)


def _draw_map_operation(
    draw: ImageDraw.ImageDraw,
    operation: MapEditOperation,
    width: int,
    height: int,
) -> None:
    if operation.shape == "line" and len(operation.points) != 2:
        raise ValueError("直线工具必须包含起点和终点")
    coordinates: list[tuple[int, int]] = []
    for point in operation.points:
        x = int(round(point.x))
        y = int(round(point.y))
        if x < 0 or x >= width or y < 0 or y >= height:
            raise ValueError("编辑笔画超出地图边界")
        coordinates.append((x, y))
    fill = {"occupied": 0, "free": 254, "unknown": 205}[operation.mode]
    line_width = operation.size
    if len(coordinates) > 1:
        draw.line(coordinates, fill=fill, width=line_width, joint="curve")
    endpoints = coordinates if len(coordinates) == 1 else (coordinates[0], coordinates[-1])
    if line_width == 1:
        for endpoint in endpoints:
            draw.point(endpoint, fill=fill)
        return
    radius = line_width / 2.0
    for x, y in endpoints:
        draw.ellipse(
            (x - radius, y - radius, x + radius, y + radius),
            fill=fill,
        )


def edit_2d_map_version(source_version_id: str, request: EditMapRequest) -> dict[str, Any]:
    source_version_id = require_id(source_version_id)
    source = version_dir(source_version_id)
    source_issues = validate_version(source) if source.is_dir() else ["源地图不存在"]
    if source_issues:
        raise HTTPException(status_code=409, detail="源地图不可编辑：" + "；".join(source_issues))
    total_points = sum(len(operation.points) for operation in request.operations)
    if total_points > 100000:
        raise HTTPException(status_code=400, detail="编辑笔画点数过多，请分批保存")

    try:
        with Image.open(source / "map.pgm") as source_file:
            source_image = source_file.convert("L")
            source_image.load()
    except OSError as exc:
        raise HTTPException(status_code=409, detail="源 PGM 无法读取") from exc

    edited_image = source_image.copy()
    draw = ImageDraw.Draw(edited_image)
    try:
        for operation in request.operations:
            _draw_map_operation(draw, operation, *edited_image.size)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    source_pixels = source_image.tobytes()
    edited_pixels = edited_image.tobytes()
    changed_pixels = sum(before != after for before, after in zip(source_pixels, edited_pixels))
    if changed_pixels == 0:
        raise HTTPException(status_code=400, detail="编辑没有改变任何地图像素")
    added_occupied = sum(
        before != 0 and after == 0 for before, after in zip(source_pixels, edited_pixels)
    )
    removed_occupied = sum(
        before == 0 and after != 0 for before, after in zip(source_pixels, edited_pixels)
    )

    source_manifest = load_yaml(source / "manifest.yaml")
    derived_id = make_id("map")
    temporary = VERSIONS_ROOT / f".{derived_id}.copying"
    target = version_dir(derived_id)
    temporary.mkdir(parents=True, exist_ok=False)
    try:
        shutil.copy2(source / "localization.pcd", temporary / "localization.pcd")
        if (source / "preview.ply").is_file():
            shutil.copy2(source / "preview.ply", temporary / "preview.ply")
        rewrite_map_yaml(source / "map.yaml", temporary / "map.yaml")
        edited_image.save(temporary / "map.pgm", format="PPM")
        edit_summary = {
            "operation_count": len(request.operations),
            "changed_pixels": changed_pixels,
            "added_occupied": added_occupied,
            "removed_occupied": removed_occupied,
        }
        save_yaml(temporary / "manifest.yaml", {
            "schema": 1,
            "id": derived_id,
            "name": request.name.strip(),
            "site": source_manifest.get("site", "default"),
            "note": request.note,
            "created_at": now_iso(),
            "source_session": source_manifest.get("source_session"),
            "parent_version": source_version_id,
            "origin": "2d-edited",
            "mapping_algorithm": source_manifest.get("mapping_algorithm", "unknown"),
            "parameters": source_manifest.get("parameters", {}),
            "edit_summary": edit_summary,
        })
        issues = validate_version(temporary)
        if issues:
            raise RuntimeError("；".join(issues))
        os.replace(temporary, target)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise

    version = next(item for item in list_versions() if item["id"] == derived_id)
    return {"version_id": derived_id, "version": version, "edit_summary": edit_summary}


def export_occupancy_map(sliced_pcd: Path, output_dir: Path, request: BuildRequest) -> None:
    node_log = output_dir / "pcd2pgm.log"
    output_prefix = output_dir / "map"
    command = [
        "ros2", "run", "pcd2pgm", "pcd2pgm_node", "--ros-args",
        "-p", f"pcd_file:={sliced_pcd}",
        "-p", f"map_resolution:={request.resolution}",
        "-p", "thre_z_min:=-5.0",
        "-p", "thre_z_max:=5.0",
        "-p", "thre_radius:=0.01",
        "-p", "thres_point_count:=1",
        "-p", "flag_pass_through:=false",
        "-p", "map_topic_name:=map_manager_internal",
        "-p", f"map_output_prefix:={output_prefix}",
    ]
    log_job("直接生成 Nav2 PGM/YAML")
    failure: Exception | None = None
    with node_log.open("w", encoding="utf-8") as log_stream:
        process = start_managed_process(
            command,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                if (output_prefix.with_suffix(".pgm").is_file() and
                        output_prefix.with_suffix(".yaml").is_file()):
                    break
                if process.poll() is not None:
                    raise RuntimeError(f"pcd2pgm 提前退出，退出码 {process.returncode}")
                if job_cancel_event.wait(0.1):
                    raise RuntimeError("任务已取消")
            else:
                raise RuntimeError("pcd2pgm 直接保存 PGM/YAML 超时")
        except Exception as exc:
            failure = exc
        finally:
            terminate_managed_process(process)
            unregister_process(process)
    try:
        node_output = node_log.read_text(encoding="utf-8", errors="replace")
    except OSError:
        node_output = ""
    for line in node_output.splitlines()[-20:]:
        log_job(line)
    if failure is not None:
        raise failure
    if not (output_dir / "map.pgm").exists() or not (output_dir / "map.yaml").exists():
        raise RuntimeError("Nav2 地图输出不完整")
    rewrite_map_yaml(output_dir / "map.yaml", output_dir / "map.yaml")


def build_version_worker(request: BuildRequest, version_id: str) -> None:
    build_dir = VERSIONS_ROOT / f".{version_id}.building"
    final_dir = VERSIONS_ROOT / version_id
    session = session_dir(request.session_id)
    raw = session / "raw.pcd"
    temp_stat = build_dir / "statistical.pcd"
    temp_slice = build_dir / "sliced.pcd"
    try:
        build_dir.mkdir(parents=True, exist_ok=False)
        log_job("检查建图会话", progress=5, stage="validate")
        if not raw.exists():
            raise RuntimeError("建图会话缺少 raw.pcd")

        run_command(
            [
                "pcl_outlier_removal", str(raw), str(temp_stat),
                "-method", "statistical",
                "-mean_k", str(request.statistical_mean_k),
                "-std_dev_mul", str(request.statistical_std_dev_mul),
            ],
            "统计离群点过滤",
            timeout=300,
        )
        log_job("统计过滤完成", progress=25, stage="filter")

        localization = build_dir / "localization.pcd"
        run_command(
            [
                "pcl_outlier_removal", str(temp_stat), str(localization),
                "-method", "radius",
                "-radius", str(request.radius),
                "-min_pts", str(request.radius_min_points),
            ],
            "半径离群点过滤",
            timeout=300,
        )
        temp_stat.unlink(missing_ok=True)
        log_job("ICP 定位地图完成", progress=45, stage="localization")

        run_command(
            [
                "pcl_passthrough_filter", str(localization), str(temp_slice),
                "-field", "z", "-min", str(request.z_min), "-max", str(request.z_max),
            ],
            "导航高度切片",
            timeout=180,
        )
        log_job("生成 2D 导航地图", progress=58, stage="occupancy")
        export_occupancy_map(temp_slice, build_dir, request)
        temp_slice.unlink(missing_ok=True)

        log_job("生成可视化预览", progress=78, stage="preview")
        generate_cloud_preview(localization, build_dir / "preview.ply", request.voxel_leaf)

        session_meta = load_yaml(session / "session.yaml")
        manifest = {
            "schema": 1,
            "id": version_id,
            "name": request.name,
            "site": session_meta.get("site", "default"),
            "note": request.note,
            "created_at": now_iso(),
            "source_session": request.session_id,
            "origin": "managed",
            "mapping_algorithm": session_meta.get("algorithm", "unknown"),
            "parameters": {
                "statistical_mean_k": request.statistical_mean_k,
                "statistical_std_dev_mul": request.statistical_std_dev_mul,
                "radius": request.radius,
                "radius_min_points": request.radius_min_points,
                "z_min": request.z_min,
                "z_max": request.z_max,
                "resolution": request.resolution,
                "preview_voxel_leaf": request.voxel_leaf,
            },
        }
        save_yaml(build_dir / "manifest.yaml", manifest)
        (build_dir / "pcd2pgm.log").unlink(missing_ok=True)
        issues = validate_version(build_dir)
        if issues:
            raise RuntimeError("；".join(issues))
        os.replace(build_dir, final_dir)
        log_job("地图版本已发布，未改变当前导航地图", progress=100, stage="complete")
        set_job(running=False, result={"version_id": version_id})
    except Exception as exc:
        cancelled = job_cancel_event.is_set()
        log_job(str(exc), stage="cancelled" if cancelled else "failed")
        shutil.rmtree(build_dir, ignore_errors=True)
        set_job(running=False, error=None if cancelled else str(exc))


def start_build_job(request: BuildRequest) -> str:
    global job_thread
    with job_lock:
        if job_state["running"]:
            raise HTTPException(status_code=409, detail="已有地图任务正在运行")
        version_id = make_id("map")
        job_state.update({
            "running": True,
            "id": version_id,
            "kind": "build",
            "progress": 0,
            "stage": "queued",
            "logs": [],
            "error": None,
            "result": None,
        })
        job_cancel_event.clear()
    job_thread = threading.Thread(target=build_version_worker, args=(request, version_id), daemon=True)
    job_thread.start()
    return version_id


def active_summary(versions: list[dict[str, Any]]) -> dict[str, Any] | None:
    active_id = read_state().get("active_id")
    return next((item for item in versions if item["id"] == active_id), None)


def require_active_map_id() -> str:
    active_id = read_state().get("active_id")
    if not active_id or validate_version(version_dir(active_id)):
        raise HTTPException(status_code=409, detail="请先激活一个完整地图版本")
    return active_id


def require_waypoint_mission_idle() -> None:
    if waypoint_mission_manager.snapshot().get("status") in ACTIVE_MISSION_STATES:
        raise HTTPException(status_code=409, detail="多目标导航任务运行中，不能修改目标点")


def cleanup_temporary_outputs() -> None:
    temporary_paths = (
        *SESSIONS_ROOT.glob(".*.copying"),
        *VERSIONS_ROOT.glob(".*.building"),
        *VERSIONS_ROOT.glob(".*.copying"),
    )
    for temporary in temporary_paths:
        if temporary.is_dir():
            shutil.rmtree(temporary, ignore_errors=True)
    for session_path in SESSIONS_ROOT.iterdir():
        clusters = session_path / "clusters"
        if clusters.is_dir():
            for temporary in clusters.glob(".*.building"):
                shutil.rmtree(temporary, ignore_errors=True)
    runtime = runtime_manager.snapshot()
    retained = (
        Path(runtime["capture_dir"]).resolve(strict=False)
        if runtime.get("capture_dir") and runtime.get("capture_status") == "pending"
        else None
    )
    output_root = runtime_output_root()
    if output_root.exists():
        for path in output_root.iterdir():
            if path.is_dir() and path.resolve(strict=False) != retained:
                shutil.rmtree(path, ignore_errors=True)


@asynccontextmanager
async def lifespan(_: FastAPI):
    runtime_manager.recover_stale_process()
    waypoint_mission_manager.recover_stale_process()
    cleanup_temporary_outputs()
    yield
    cancel_map_job()
    waypoint_mission_manager.close()
    runtime_manager.close()


app = FastAPI(title="Go2 Map Workspace", version="2.0.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://127.0.0.1:5173", "http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
def health() -> dict[str, Any]:
    return {"ok": True, "repo_root": str(REPO_ROOT), "maps_root": str(MAPS_ROOT)}


@app.get("/api/overview")
def overview() -> dict[str, Any]:
    versions = list_versions()
    state = read_state()
    active_id = state.get("active_id")
    waypoints = waypoint_store.list(active_id) if active_id else []
    return {
        "workspace": {
            "repo_root": str(REPO_ROOT),
            "maps_root": str(MAPS_ROOT),
            "active_link": str(ACTIVE_LINK),
        },
        "active": active_summary(versions),
        "previous_active_id": state.get("previous_active_id"),
        "versions": versions,
        "sessions": list_sessions(),
        "trash": list_trash(),
        "current_capture": current_capture(),
        "legacy_maps": list_legacy_maps(),
        "legacy_icp": file_info(LEGACY_ICP_PCD),
        "build_profiles": list_build_profiles(),
        "job": job_snapshot(),
        "runtime": runtime_manager.snapshot(),
        "waypoints": {
            "map_id": active_id,
            "count": len(waypoints),
            "items": waypoints,
        },
        "waypoint_mission": waypoint_mission_manager.snapshot(),
        "runtime_modules": {
            "localization": [
                {
                    "id": "fused_ekf",
                    "name": "融合定位",
                    "description": "运动预测 + ICP 地图修正",
                },
                {
                    "id": "pure_icp",
                    "name": "纯 ICP",
                    "description": "原有点云直接匹配定位",
                },
            ],
        },
    }


@app.post("/api/runtime/start", status_code=202)
def start_runtime(request: RuntimeRequest) -> dict[str, Any]:
    if job_snapshot()["running"]:
        raise HTTPException(status_code=409, detail="地图构建任务进行中，不能启动 ROS 流程")
    runtime = runtime_manager.snapshot()
    if runtime.get("status") in {"running", "stopping"}:
        raise HTTPException(status_code=409, detail="已有流程正在运行，请先停止")
    if request.mode in {"localization", "navigation"}:
        active = read_state().get("active_id")
        if not active or validate_version(version_dir(active)):
            raise HTTPException(status_code=409, detail="请先激活一个完整地图版本")
    try:
        auto_archived = auto_archive_pending_capture("启动新流程前")
    except HTTPException:
        raise
    except (OSError, RuntimeError, yaml.YAMLError) as exc:
        raise HTTPException(status_code=500, detail=f"上一份 PCD 自动归档失败: {exc}") from exc

    if request.mode == "mapping":
        algorithm = request.algorithm or "faster_lio"
    elif request.mode in {"localization", "navigation"}:
        algorithm = request.algorithm or "pure_icp"
    run_id = make_id("run") if request.mode == "mapping" else None
    capture_path: Path | None = None
    capture_dir: Path | None = None
    capture_baseline = None
    environment = None
    if request.mode == "mapping":
        capture_dir = runtime_output_root() / run_id
        capture_dir.mkdir(parents=True, exist_ok=False)
        capture_path = capture_dir / "scans.pcd"
        environment = {"GO2_MAPPING_OUTPUT_DIR": str(capture_dir)}
    try:
        result = runtime_manager.start(
            request.mode,
            algorithm,
            environment=environment,
            run_id=run_id,
            capture_path=capture_path,
            capture_dir=capture_dir,
            capture_baseline=capture_baseline,
        )
        return {
            **result,
            "auto_archived_session_id": auto_archived["session_id"] if auto_archived else None,
        }
    except (RuntimeError, ValueError) as exc:
        if capture_dir:
            shutil.rmtree(capture_dir, ignore_errors=True)
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except OSError as exc:
        if capture_dir:
            shutil.rmtree(capture_dir, ignore_errors=True)
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.post("/api/runtime/stop")
def stop_runtime() -> dict[str, Any]:
    runtime = runtime_manager.snapshot()
    if runtime.get("mode") == "navigation":
        # Let the action client cancel the active Nav2 goal before the Nav2
        # process itself is stopped.
        waypoint_mission_manager.close()
    # FASTer-LIO writes the accumulated cloud during graceful shutdown. Large
    # captures can legitimately need much longer than the generic stop timeout.
    timeout = 75.0 if runtime.get("mode") == "mapping" else 12.0
    result = runtime_manager.stop(timeout=timeout)
    capture = current_capture()
    save_error = None
    if (
        runtime.get("status") in {"running", "stopping"}
        and runtime.get("mode") == "mapping"
        and runtime.get("capture_status") == "pending"
        and not capture.get("available")
    ):
        algorithm_name = "FAST-LIO2" if runtime.get("algorithm") == "fastlio2" else "建图"
        save_error = (
            f"{algorithm_name} 已停止，但没有生成 PCD。"
            "请保留当前页面并查看运行日志中的 ERROR；未保存成功前不要归档。"
        )
    return {**result, "current_capture": capture, "save_error": save_error}


@app.post("/api/runtime/cleanup")
def cleanup_runtime_processes() -> dict[str, Any]:
    """Emergency cleanup for project ROS processes, while keeping Web alive."""

    with process_cleanup_lock:
        errors: list[str] = []
        try:
            waypoint_mission_manager.close()
        except (OSError, RuntimeError) as exc:
            errors.append(f"导航任务取消失败: {exc}")
        try:
            cancel_map_job()
            terminate_all_managed_processes()
        except (OSError, RuntimeError) as exc:
            errors.append(f"地图任务停止失败: {exc}")
        try:
            # This is the emergency path: allow a short graceful stop before
            # the process scanner escalates any survivors.
            runtime_manager.stop(timeout=3.0)
        except (OSError, RuntimeError) as exc:
            errors.append(f"运行流程停止失败: {exc}")

        cleanup = cleanup_project_processes(REPO_ROOT)
        auto_archived = None
        try:
            auto_archived = auto_archive_pending_capture("使用应急进程清理后")
        except (HTTPException, OSError, RuntimeError, yaml.YAMLError) as exc:
            detail = exc.detail if isinstance(exc, HTTPException) else str(exc)
            errors.append(f"建图 PCD 自动归档失败: {detail}")
        return {
            **cleanup,
            "errors": errors,
            "auto_archived_session_id": auto_archived["session_id"] if auto_archived else None,
            "runtime": runtime_manager.snapshot(),
            "job": job_snapshot(),
            "waypoint_mission": waypoint_mission_manager.snapshot(),
        }


@app.post("/api/runtime/discard-mapping")
def discard_mapping() -> dict[str, Any]:
    runtime_manager.stop(timeout=75.0)
    capture = current_capture()
    if not capture["new_for_run"]:
        raise HTTPException(status_code=409, detail="本次建图没有生成可丢弃的新 PCD")
    try:
        remove_capture_output(capture)
        runtime_manager.mark_capture("discarded")
    except (OSError, RuntimeError) as exc:
        raise HTTPException(status_code=500, detail=f"建图结果清理失败: {exc}") from exc
    return {"discarded": True, "run_id": capture["run_id"], "current_capture": current_capture()}


@app.post("/api/waypoints/capture")
def capture_waypoint(request: CaptureWaypointRequest) -> dict[str, Any]:
    require_waypoint_mission_idle()
    active_id = require_active_map_id()
    runtime = runtime_manager.snapshot()
    if runtime.get("status") != "running" or runtime.get("mode") != "navigation":
        raise HTTPException(status_code=409, detail="请先在 Web 中启动导航，再记录当前位置")
    try:
        pose = waypoint_mission_manager.capture_pose(timeout_seconds=5.0)
        waypoint = waypoint_store.add(active_id, pose, request.name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        raise HTTPException(
            status_code=409,
            detail=f"当前位置记录失败：{exc}。请确认定位已收敛且 map → base_link TF 正常。",
        ) from exc
    return {"map_id": active_id, "waypoint": waypoint}


@app.patch("/api/waypoints/{waypoint_id}")
def rename_waypoint(waypoint_id: str, request: RenameWaypointRequest) -> dict[str, Any]:
    require_waypoint_mission_idle()
    active_id = require_active_map_id()
    try:
        waypoint = waypoint_store.rename(active_id, waypoint_id, request.name)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="目标点不存在") from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"map_id": active_id, "waypoint": waypoint}


@app.delete("/api/waypoints/{waypoint_id}")
def delete_waypoint(waypoint_id: str) -> dict[str, Any]:
    require_waypoint_mission_idle()
    active_id = require_active_map_id()
    try:
        waypoint = waypoint_store.delete(active_id, waypoint_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="目标点不存在") from exc
    return {"map_id": active_id, "deleted": waypoint}


@app.put("/api/waypoints/reorder")
def reorder_waypoints(request: ReorderWaypointsRequest) -> dict[str, Any]:
    require_waypoint_mission_idle()
    active_id = require_active_map_id()
    try:
        items = waypoint_store.reorder(active_id, request.waypoint_ids)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    return {"map_id": active_id, "items": items}


@app.post("/api/waypoint-mission/start", status_code=202)
def start_waypoint_mission(request: WaypointMissionRequest) -> dict[str, Any]:
    active_id = require_active_map_id()
    runtime = runtime_manager.snapshot()
    if runtime.get("status") != "running" or runtime.get("mode") != "navigation":
        raise HTTPException(status_code=409, detail="请先在 Web 中成功启动导航流程")
    if job_snapshot()["running"]:
        raise HTTPException(status_code=409, detail="地图任务运行中，不能开始导航")
    waypoints = waypoint_store.list(active_id)
    if not waypoints:
        raise HTTPException(status_code=409, detail="请至少记录一个目标点")
    try:
        return waypoint_mission_manager.start(
            make_id("mission"),
            active_id,
            waypoints,
            stop_on_failure=request.stop_on_failure,
            waypoint_timeout_sec=request.waypoint_timeout_sec,
            pause_between_sec=request.pause_between_sec,
        )
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"导航任务启动失败：{exc}") from exc


@app.get("/api/waypoint-mission")
def waypoint_mission_status() -> dict[str, Any]:
    return waypoint_mission_manager.snapshot()


@app.post("/api/waypoint-mission/cancel", status_code=202)
def cancel_waypoint_mission() -> dict[str, Any]:
    return waypoint_mission_manager.cancel()


@app.post("/api/jobs/cancel")
def cancel_job() -> dict[str, Any]:
    return cancel_map_job()


@app.post("/api/sessions/archive")
def archive_session(request: ArchiveRequest) -> dict[str, Any]:
    runtime = runtime_manager.snapshot()
    if runtime.get("mode") == "mapping" and runtime.get("status") in {"running", "stopping"}:
        raise HTTPException(status_code=409, detail="请先结束建图，再归档输出")
    return archive_capture_output(current_capture(), request)


@app.patch("/api/sessions/{session_id}")
def rename_session(session_id: str, request: RenameRequest) -> dict[str, Any]:
    return rename_metadata_item(session_id, request, kind="session")


@app.post("/api/sessions/{session_id}/clusters")
def cluster_session(session_id: str, request: ClusterRequest) -> dict[str, Any]:
    session_id = require_id(session_id)
    if job_snapshot()["running"]:
        raise HTTPException(status_code=409, detail="地图任务运行中，不能开始聚类")
    if runtime_manager.snapshot()["status"] in {"running", "stopping"}:
        raise HTTPException(status_code=409, detail="请先停止 ROS 流程再进行点云聚类")
    if not cluster_lock.acquire(blocking=False):
        raise HTTPException(status_code=409, detail="已有点云聚类或清理任务正在运行")
    try:
        return create_cluster_result(session_id, request)
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"点云聚类失败：{exc}") from exc
    finally:
        cluster_lock.release()


@app.get("/api/sessions/{session_id}/clusters/{cluster_run_id}")
def cluster_result(session_id: str, cluster_run_id: str) -> dict[str, Any]:
    return load_cluster_result(require_id(session_id), require_id(cluster_run_id))


@app.get("/api/sessions/{session_id}/clusters/{cluster_run_id}/preview.ply")
def cluster_preview(session_id: str, cluster_run_id: str) -> FileResponse:
    path = cluster_run_dir(require_id(session_id), require_id(cluster_run_id)) / "preview.ply"
    if not path.is_file():
        raise HTTPException(status_code=404, detail="聚类预览不存在")
    return FileResponse(
        path,
        media_type="application/octet-stream",
        filename=f"{session_id}-{cluster_run_id}.ply",
    )


@app.post("/api/sessions/{session_id}/clusters/apply")
def apply_session_clusters(session_id: str, request: ClusterApplyRequest) -> dict[str, Any]:
    session_id = require_id(session_id)
    if job_snapshot()["running"]:
        raise HTTPException(status_code=409, detail="地图任务运行中，不能生成清理副本")
    if runtime_manager.snapshot()["status"] in {"running", "stopping"}:
        raise HTTPException(status_code=409, detail="请先停止 ROS 流程再生成清理副本")
    if not cluster_lock.acquire(blocking=False):
        raise HTTPException(status_code=409, detail="已有点云聚类或清理任务正在运行")
    try:
        return apply_cluster_removal(session_id, request)
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"生成清理副本失败：{exc}") from exc
    finally:
        cluster_lock.release()


@app.post("/api/versions/build", status_code=202)
def build_version(request: BuildRequest) -> dict[str, Any]:
    if runtime_manager.snapshot()["status"] in {"running", "stopping"}:
        raise HTTPException(status_code=409, detail="ROS 流程运行中，不能构建地图")
    require_id(request.session_id)
    if request.z_min >= request.z_max:
        raise HTTPException(status_code=400, detail="z_min 必须小于 z_max")
    if not session_dir(request.session_id).exists():
        raise HTTPException(status_code=404, detail="建图会话不存在")
    return {"job_id": start_build_job(request)}


@app.post("/api/build-profiles")
def upsert_build_profile(request: BuildProfileRequest) -> dict[str, Any]:
    return save_build_profile(request)


@app.patch("/api/versions/{version_id}")
def rename_version(version_id: str, request: RenameRequest) -> dict[str, Any]:
    return rename_metadata_item(version_id, request, kind="version")


@app.post("/api/versions/{version_id}/edit-2d", status_code=201)
def edit_version_2d(version_id: str, request: EditMapRequest) -> dict[str, Any]:
    if job_snapshot()["running"]:
        raise HTTPException(status_code=409, detail="地图构建任务进行中，不能保存 2D 编辑")
    if runtime_manager.snapshot()["status"] in {"running", "stopping"}:
        raise HTTPException(status_code=409, detail="请先停止 ROS 流程再保存 2D 编辑")
    try:
        return edit_2d_map_version(version_id, request)
    except HTTPException:
        raise
    except (OSError, RuntimeError, yaml.YAMLError) as exc:
        raise HTTPException(status_code=500, detail=f"2D 地图保存失败：{exc}") from exc


@app.post("/api/versions/adopt-current")
def adopt_current(request: AdoptRequest) -> dict[str, Any]:
    with job_lock:
        if job_state["running"]:
            raise HTTPException(status_code=409, detail="已有地图任务正在运行")
        job_state.update({
            "running": True,
            "id": None,
            "kind": "adopt",
            "progress": 0,
            "stage": "copy",
            "logs": [],
            "error": None,
            "result": None,
        })
    latest_yaml = MAPS_ROOT / "map_latest.yaml"
    if not latest_yaml.exists() or not LEGACY_ICP_PCD.exists():
        set_job(running=False, stage="failed", error="当前 legacy 地图组合不完整")
        raise HTTPException(status_code=404, detail="当前 legacy 地图组合不完整")
    source_yaml = latest_yaml.resolve()
    try:
        config = load_yaml(source_yaml)
    except (OSError, yaml.YAMLError) as exc:
        set_job(running=False, stage="failed", error=str(exc))
        raise HTTPException(status_code=500, detail="map_latest.yaml 无法读取") from exc
    source_pgm = (source_yaml.parent / str(config.get("image", ""))).resolve()
    if not source_pgm.exists():
        set_job(running=False, stage="failed", error="map_latest.yaml 引用的 PGM 不存在")
        raise HTTPException(status_code=404, detail="map_latest.yaml 引用的 PGM 不存在")

    version_id = make_id("legacy")
    temp = VERSIONS_ROOT / f".{version_id}.copying"
    final = version_dir(version_id)
    temp.mkdir(parents=True, exist_ok=False)
    try:
        shutil.copy2(source_pgm, temp / "map.pgm")
        rewrite_map_yaml(source_yaml, temp / "map.yaml")
        shutil.copy2(LEGACY_ICP_PCD, temp / "localization.pcd")
        set_job(id=version_id, progress=70, stage="preview")
        generate_cloud_preview(temp / "localization.pcd", temp / "preview.ply")
        save_yaml(temp / "manifest.yaml", {
            "schema": 1,
            "id": version_id,
            "name": request.name,
            "site": "legacy",
            "note": request.note,
            "created_at": now_iso(),
            "source_session": None,
            "origin": "legacy-adopted",
            "parameters": {},
        })
        os.replace(temp, final)
        set_job(running=False, progress=100, stage="complete", result={"version_id": version_id})
    except Exception as exc:
        shutil.rmtree(temp, ignore_errors=True)
        set_job(running=False, stage="failed", error=str(exc))
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    return {"version_id": version_id}


@app.post("/api/active")
def activate(request: ActivateRequest) -> dict[str, Any]:
    if job_snapshot()["running"]:
        raise HTTPException(status_code=409, detail="地图构建任务进行中，不能切换当前地图")
    if runtime_manager.snapshot()["status"] in {"running", "stopping"}:
        raise HTTPException(status_code=409, detail="请先停止定位、导航或建图，再切换当前地图")
    target = version_dir(request.version_id)
    if not target.exists():
        raise HTTPException(status_code=404, detail="地图版本不存在")
    issues = validate_version(target)
    if issues:
        raise HTTPException(status_code=409, detail="；".join(issues))

    temp_link = MAPS_ROOT / ".active.next"
    temp_link.unlink(missing_ok=True)
    temp_link.symlink_to(target.relative_to(MAPS_ROOT), target_is_directory=True)
    os.replace(temp_link, ACTIVE_LINK)

    state = read_state()
    previous = state.get("active_id")
    state["previous_active_id"] = previous if previous != request.version_id else state.get("previous_active_id")
    state["active_id"] = request.version_id
    write_state(state)
    return {"active_id": request.version_id, "previous_active_id": state.get("previous_active_id")}


@app.post("/api/active/rollback")
def rollback() -> dict[str, Any]:
    state = read_state()
    previous = state.get("previous_active_id")
    if not previous:
        raise HTTPException(status_code=409, detail="没有可回退的地图版本")
    return activate(ActivateRequest(version_id=previous))


@app.post("/api/trash")
def move_to_trash(request: TrashRequest) -> dict[str, Any]:
    item_id = require_id(request.item_id)
    state = read_state()
    if request.kind == "version":
        if item_id == state.get("active_id"):
            raise HTTPException(status_code=409, detail="当前启用地图不能删除，请先切换地图")
        source = version_dir(item_id)
    else:
        source = session_dir(item_id)
    if not source.is_dir():
        raise HTTPException(status_code=404, detail="要删除的项目不存在")

    trash_id = make_id("trash")
    destination = trash_dir(trash_id)
    os.replace(source, destination)
    try:
        write_json_atomic(destination / ".trash.json", {
            "schema": 1,
            "id": trash_id,
            "kind": request.kind,
            "original_id": item_id,
            "deleted_at": now_iso(),
        })
    except Exception:
        os.replace(destination, source)
        raise

    if request.kind == "version":
        if state.get("previous_active_id") == item_id:
            state["previous_active_id"] = None
        state["archived"] = [value for value in state.get("archived", []) if value != item_id]
        write_state(state)
    return {"trash_id": trash_id}


@app.post("/api/trash/{trash_id}/restore")
def restore_trash(trash_id: str) -> dict[str, Any]:
    source = trash_dir(trash_id)
    metadata_path = source / ".trash.json"
    if not metadata_path.exists():
        raise HTTPException(status_code=404, detail="回收站项目不存在")
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        item_id = require_id(str(metadata["original_id"]))
        kind = metadata["kind"]
    except (OSError, KeyError, json.JSONDecodeError, HTTPException) as exc:
        raise HTTPException(status_code=409, detail="回收站元数据损坏") from exc
    destination = version_dir(item_id) if kind == "version" else session_dir(item_id)
    if destination.exists():
        raise HTTPException(status_code=409, detail="原位置已有同名项目，无法恢复")
    metadata_path.unlink()
    try:
        os.replace(source, destination)
    except Exception:
        write_json_atomic(metadata_path, metadata)
        raise
    return {"kind": kind, "item_id": item_id}


def purge_trash_items(trash_ids: list[str]) -> dict[str, Any]:
    unique_ids = list(dict.fromkeys(require_id(value) for value in trash_ids))
    targets = [trash_dir(trash_id) for trash_id in unique_ids]
    missing = [
        trash_id
        for trash_id, target in zip(unique_ids, targets)
        if not target.is_dir() or not (target / ".trash.json").is_file()
    ]
    if missing:
        raise HTTPException(
            status_code=404,
            detail=f"回收站项目不存在: {', '.join(missing)}",
        )
    try:
        for target in targets:
            shutil.rmtree(target)
    except OSError as exc:
        raise HTTPException(status_code=500, detail=f"彻底删除失败: {exc}") from exc
    return {"deleted": unique_ids, "count": len(unique_ids)}


@app.post("/api/trash/purge")
def purge_trash_batch(request: PurgeTrashRequest) -> dict[str, Any]:
    return purge_trash_items(request.trash_ids)


@app.delete("/api/trash/{trash_id}")
def purge_trash(trash_id: str) -> dict[str, Any]:
    result = purge_trash_items([trash_id])
    return {"deleted": result["deleted"][0]}


def pgm_response(path: Path) -> Response:
    if not path.exists():
        raise HTTPException(status_code=404, detail="PGM 不存在")
    with Image.open(path) as image:
        buffer = io.BytesIO()
        image.convert("L").save(buffer, format="PNG", optimize=True)
    return Response(content=buffer.getvalue(), media_type="image/png")


@app.get("/api/versions/{version_id}/map.png")
def version_map_preview(version_id: str) -> Response:
    return pgm_response(version_dir(version_id) / "map.pgm")


@app.get("/api/sessions/{session_id}/cloud.ply")
def session_cloud_preview(session_id: str) -> FileResponse:
    path = session_dir(session_id)
    raw = path / "raw.pcd"
    preview = path / "preview.ply"
    if not raw.exists():
        raise HTTPException(status_code=404, detail="原始 PCD 不存在")

    with session_preview_lock:
        if not preview.exists() or preview.stat().st_mtime < raw.stat().st_mtime:
            preview.unlink(missing_ok=True)
            try:
                generate_cloud_preview(raw, preview, leaf=0.20, track_job=False)
            except Exception as exc:
                preview.unlink(missing_ok=True)
                raise HTTPException(status_code=500, detail=f"原始 PCD 预览生成失败：{exc}") from exc

    return FileResponse(preview, media_type="application/octet-stream", filename=f"{session_id}.ply")


@app.get("/api/versions/{version_id}/cloud.ply")
def version_cloud_preview(version_id: str) -> FileResponse:
    path = version_dir(version_id) / "preview.ply"
    if not path.exists():
        raise HTTPException(status_code=404, detail="3D 预览尚未生成")
    return FileResponse(path, media_type="application/octet-stream", filename=f"{version_id}.ply")


@app.get("/api/legacy/{stem}/map.png")
def legacy_map_preview(stem: str) -> Response:
    if not re.fullmatch(r"map(?:_[0-9_]+)?", stem):
        raise HTTPException(status_code=400, detail="Invalid legacy map name")
    yaml_path = MAPS_ROOT / f"{stem}.yaml"
    if not yaml_path.exists():
        raise HTTPException(status_code=404, detail="Legacy YAML 不存在")
    config = load_yaml(yaml_path)
    pgm_path = (MAPS_ROOT / str(config.get("image", ""))).resolve()
    if MAPS_ROOT not in pgm_path.parents or not pgm_path.exists():
        raise HTTPException(status_code=404, detail="Legacy PGM 不存在")
    return pgm_response(pgm_path)


@app.exception_handler(Exception)
async def unhandled_error(_, exc: Exception) -> JSONResponse:
    return JSONResponse(status_code=500, content={"detail": str(exc)})


DIST_ROOT = Path(__file__).resolve().parents[1] / "frontend" / "dist"
if DIST_ROOT.exists():
    app.mount("/", StaticFiles(directory=DIST_ROOT, html=True), name="frontend")
else:
    @app.get("/")
    def no_frontend() -> dict[str, str]:
        return {"message": "Frontend not built. Run npm install && npm run build in tools/map_manager/frontend."}

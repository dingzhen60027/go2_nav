import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import yaml

from tools.map_manager.backend import app as backend


class ActivationTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.maps_root = Path(self.temp_dir.name) / "maps"
        self.workspace_root = self.maps_root / "workspace"
        self.versions_root = self.workspace_root / "versions"
        self.versions_root.mkdir(parents=True)
        self.patchers = [
            patch.object(backend, "MAPS_ROOT", self.maps_root),
            patch.object(backend, "WORKSPACE_ROOT", self.workspace_root),
            patch.object(backend, "VERSIONS_ROOT", self.versions_root),
            patch.object(backend, "SESSIONS_ROOT", self.workspace_root / "sessions"),
            patch.object(backend, "TRASH_ROOT", self.workspace_root / "trash"),
            patch.object(backend, "STATE_PATH", self.workspace_root / "state.json"),
            patch.object(backend, "BUILD_PROFILES_PATH", self.workspace_root / "build_profiles.yaml"),
            patch.object(backend, "ACTIVE_LINK", self.maps_root / "active"),
            patch.object(backend, "CURRENT_RAW_PCD", self.workspace_root / "capture" / "scans.pcd"),
        ]
        for patcher in self.patchers:
            patcher.start()
        backend.SESSIONS_ROOT.mkdir(parents=True)
        backend.TRASH_ROOT.mkdir(parents=True)

    def tearDown(self):
        for patcher in reversed(self.patchers):
            patcher.stop()
        self.temp_dir.cleanup()

    def create_version(self, version_id):
        path = self.versions_root / version_id
        path.mkdir()
        (path / "manifest.yaml").write_text("schema: 1\n", encoding="utf-8")
        (path / "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")
        (path / "map.pgm").write_bytes(b"P5\n2 2\n255\n\x00\xfe\xfe\x00")
        (path / "localization.pcd").write_text("VERSION 0.7\n", encoding="ascii")
        return path

    def test_validate_version_rejects_empty_occupancy_map(self):
        version = self.create_version("map-20260805-empty-pgm")
        (version / "map.pgm").write_bytes(b"P5\n1 1\n255\n\x00")

        issues = backend.validate_version(version)

        self.assertTrue(any("尺寸异常" in issue for issue in issues))

    def test_2d_editor_creates_derived_version_without_mutating_source(self):
        source = self.create_version("map-20260805-edit-source")
        source_pgm = (source / "map.pgm").read_bytes()
        derived_id = "map-20260805-edit-derived"
        request = backend.EditMapRequest(
            name="补墙修订版",
            note="去除杂点并补墙",
            operations=[backend.MapEditOperation(
                mode="free",
                shape="brush",
                size=1,
                points=[backend.MapEditPoint(x=0, y=0)],
            )],
        )

        with patch.object(backend, "make_id", return_value=derived_id):
            result = backend.edit_2d_map_version(source.name, request)

        derived = self.versions_root / derived_id
        self.assertEqual((source / "map.pgm").read_bytes(), source_pgm)
        self.assertEqual(
            (derived / "localization.pcd").read_bytes(),
            (source / "localization.pcd").read_bytes(),
        )
        self.assertEqual(result["version_id"], derived_id)
        self.assertGreater(result["edit_summary"]["changed_pixels"], 0)
        self.assertGreater(result["edit_summary"]["removed_occupied"], 0)
        manifest = yaml.safe_load((derived / "manifest.yaml").read_text(encoding="utf-8"))
        self.assertEqual(manifest["origin"], "2d-edited")
        self.assertEqual(manifest["parent_version"], source.name)
        self.assertEqual(manifest["name"], "补墙修订版")
        self.assertFalse(backend.validate_version(derived))

    def test_2d_editor_rejects_strokes_outside_map(self):
        source = self.create_version("map-20260805-edit-bounds")
        request = backend.EditMapRequest(
            name="invalid",
            operations=[backend.MapEditOperation(
                mode="occupied",
                shape="line",
                size=2,
                points=[
                    backend.MapEditPoint(x=0, y=0),
                    backend.MapEditPoint(x=5, y=5),
                ],
            )],
        )

        with self.assertRaises(Exception) as context:
            backend.edit_2d_map_version(source.name, request)

        self.assertEqual(context.exception.status_code, 400)

    def create_session(self, session_id):
        path = backend.SESSIONS_ROOT / session_id
        path.mkdir()
        (path / "session.yaml").write_text(
            f"schema: 1\nid: {session_id}\nname: Raw capture\nsite: lab\n",
            encoding="utf-8",
        )
        (path / "raw.pcd").write_text("VERSION 0.7\n", encoding="ascii")
        return path

    def test_activate_and_rollback_are_atomic(self):
        first = self.create_version("map-20260803-first")
        second = self.create_version("map-20260803-second")

        backend.activate(backend.ActivateRequest(version_id=first.name))
        self.assertEqual(backend.ACTIVE_LINK.resolve(), first)

        backend.activate(backend.ActivateRequest(version_id=second.name))
        self.assertEqual(backend.ACTIVE_LINK.resolve(), second)
        self.assertEqual(backend.read_state()["previous_active_id"], first.name)

        backend.rollback()
        self.assertEqual(backend.ACTIVE_LINK.resolve(), first)
        self.assertEqual(backend.read_state()["previous_active_id"], second.name)

    def test_version_delete_moves_to_trash_and_restores(self):
        version = self.create_version("map-20260803-trash")

        result = backend.move_to_trash(backend.TrashRequest(kind="version", item_id=version.name))
        trashed = backend.TRASH_ROOT / result["trash_id"]
        self.assertFalse(version.exists())
        self.assertTrue((trashed / ".trash.json").exists())
        self.assertEqual(backend.list_trash()[0]["original_id"], version.name)

        backend.restore_trash(result["trash_id"])
        self.assertTrue(version.exists())
        self.assertFalse(trashed.exists())

    def test_sessions_and_versions_can_be_renamed_without_changing_ids(self):
        session = self.create_session("session-20260803-rename")
        version = self.create_version("map-20260803-rename")

        session_result = backend.rename_session(
            session.name,
            backend.RenameRequest(name="  仓库原始点云  "),
        )
        version_result = backend.rename_version(
            version.name,
            backend.RenameRequest(name="仓库定位地图"),
        )

        self.assertEqual(session_result["name"], "仓库原始点云")
        self.assertEqual(version_result["name"], "仓库定位地图")
        self.assertEqual(backend.list_sessions()[0]["name"], "仓库原始点云")
        self.assertEqual(backend.list_versions()[0]["name"], "仓库定位地图")
        self.assertTrue((session / "raw.pcd").is_file())
        self.assertTrue((version / "localization.pcd").is_file())
        self.assertEqual(
            yaml.safe_load((session / "session.yaml").read_text(encoding="utf-8"))["id"],
            session.name,
        )

    def test_rename_rejects_whitespace_only_name(self):
        session = self.create_session("session-20260803-empty-name")

        with self.assertRaises(Exception) as context:
            backend.rename_session(session.name, backend.RenameRequest(name="   "))

        self.assertEqual(context.exception.status_code, 400)
        self.assertEqual(backend.list_sessions()[0]["name"], "Raw capture")

    def test_batch_purge_validates_all_items_before_deleting(self):
        session = self.create_session("session-20260803-bulk-trash")
        version = self.create_version("map-20260803-bulk-trash")
        session_trash = backend.move_to_trash(
            backend.TrashRequest(kind="session", item_id=session.name)
        )["trash_id"]
        version_trash = backend.move_to_trash(
            backend.TrashRequest(kind="version", item_id=version.name)
        )["trash_id"]

        with self.assertRaises(Exception) as context:
            backend.purge_trash_batch(backend.PurgeTrashRequest(
                trash_ids=[session_trash, "trash-does-not-exist"],
            ))

        self.assertEqual(context.exception.status_code, 404)
        self.assertTrue((backend.TRASH_ROOT / session_trash).is_dir())
        self.assertTrue((backend.TRASH_ROOT / version_trash).is_dir())

        result = backend.purge_trash_batch(backend.PurgeTrashRequest(
            trash_ids=[session_trash, version_trash, session_trash],
        ))
        self.assertEqual(result["count"], 2)
        self.assertFalse((backend.TRASH_ROOT / session_trash).exists())
        self.assertFalse((backend.TRASH_ROOT / version_trash).exists())

    def test_build_parameter_profile_is_saved_loaded_and_updated_by_name(self):
        profile_id = "profile-20260805-warehouse"
        with patch.object(backend, "make_id", return_value=profile_id):
            created = backend.upsert_build_profile(backend.BuildProfileRequest(
                name="仓库高密度",
                statistical_mean_k=35,
                statistical_std_dev_mul=0.8,
                radius=0.22,
                radius_min_points=7,
                voxel_leaf=0.15,
                z_min=0.25,
                z_max=1.8,
                resolution=0.03,
            ))

        self.assertTrue(created["created"])
        self.assertEqual(created["profile"]["id"], profile_id)
        self.assertTrue(backend.BUILD_PROFILES_PATH.is_file())
        loaded = backend.list_build_profiles()
        self.assertEqual(len(loaded), 1)
        self.assertEqual(loaded[0]["parameters"]["statistical_mean_k"], 35)
        self.assertEqual(loaded[0]["parameters"]["radius"], 0.22)

        updated = backend.upsert_build_profile(backend.BuildProfileRequest(
            name="仓库高密度",
            radius=0.45,
        ))
        self.assertFalse(updated["created"])
        self.assertEqual(updated["profile"]["id"], profile_id)
        self.assertEqual(len(backend.list_build_profiles()), 1)
        self.assertEqual(backend.list_build_profiles()[0]["parameters"]["radius"], 0.45)

    def test_build_parameter_profile_rejects_invalid_height_slice(self):
        with self.assertRaises(Exception) as context:
            backend.upsert_build_profile(backend.BuildProfileRequest(
                name="错误高度",
                z_min=2.0,
                z_max=1.0,
            ))
        self.assertEqual(context.exception.status_code, 400)

    def test_active_version_cannot_be_deleted(self):
        version = self.create_version("map-20260803-active")
        backend.activate(backend.ActivateRequest(version_id=version.name))
        with self.assertRaises(Exception) as context:
            backend.move_to_trash(backend.TrashRequest(kind="version", item_id=version.name))
        self.assertEqual(context.exception.status_code, 409)

    def test_active_version_cannot_change_while_runtime_is_running(self):
        version = self.create_version("map-20260803-runtime")
        with patch.object(backend.runtime_manager, "snapshot", return_value={"status": "running"}):
            with self.assertRaises(Exception) as context:
                backend.activate(backend.ActivateRequest(version_id=version.name))
        self.assertEqual(context.exception.status_code, 409)

    def test_managed_command_is_terminated_as_process_group(self):
        process = backend.start_managed_process(
            ["/bin/bash", "-c", "echo READY; sleep 30"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            self.assertEqual(process.stdout.readline().strip(), "READY")
            backend.terminate_managed_process(process, interrupt_timeout=0.2)
            self.assertIsNotNone(process.returncode)
            self.assertFalse(Path(f"/proc/{process.pid}").exists())
        finally:
            if process.stdout:
                process.stdout.close()
            backend.unregister_process(process)

    def test_session_cloud_preview_is_generated_once_and_cached(self):
        session = self.create_session("session-20260803-preview")

        def generate(_, output, **kwargs):
            self.assertEqual(kwargs, {"leaf": 0.20, "track_job": False})
            output.write_text("ply\n", encoding="ascii")

        with patch.object(backend, "generate_cloud_preview", side_effect=generate) as generator:
            first = backend.session_cloud_preview(session.name)
            second = backend.session_cloud_preview(session.name)

        self.assertEqual(Path(first.path), session / "preview.ply")
        self.assertEqual(Path(second.path), session / "preview.ply")
        self.assertEqual(generator.call_count, 1)
        listed = backend.list_sessions()[0]
        self.assertEqual(listed["cloud_preview_url"], f"/api/sessions/{session.name}/cloud.ply")

    def test_cluster_removal_creates_copy_without_overwriting_original_pcd(self):
        session = self.create_session("session-20260805-source")
        original = b"ORIGINAL PCD CONTENT\nPOINT DATA\n"
        (session / "raw.pcd").write_bytes(original)
        cluster_run_id = "cluster-20260805-test"
        cluster_dir = session / "clusters" / cluster_run_id
        cluster_dir.mkdir(parents=True)
        (cluster_dir / "voxels.bin").write_bytes(b"labels")
        (cluster_dir / "metadata.json").write_text(
            backend.json.dumps({
                "schema": 1,
                "id": cluster_run_id,
                "session_id": session.name,
                "source_identity": backend.capture_identity(session / "raw.pcd"),
                "parameters": {"voxel_leaf": 0.15},
                "clusters": [{"id": 7, "point_count": 12}],
                "cluster_count": 1,
            }),
            encoding="utf-8",
        )

        def run(command, label, **_):
            self.assertEqual(label, "生成聚类清理 PCD 副本")
            output = Path(command[command.index("--output") + 1])
            output.write_bytes(b"CLEANED COPY\n")

        derived_id = "session-20260805-cleaned"
        with (
            patch.object(backend, "make_id", return_value=derived_id),
            patch.object(backend, "run_command", side_effect=run),
            patch.object(
                backend,
                "generate_cloud_preview",
                side_effect=lambda _, output, **__: output.write_bytes(b"ply\n"),
            ),
        ):
            result = backend.apply_cluster_removal(
                session.name,
                backend.ClusterApplyRequest(
                    cluster_run_id=cluster_run_id,
                    cluster_ids=[7],
                    name="Cleaned copy",
                    note="manual removal",
                ),
            )

        derived = backend.SESSIONS_ROOT / derived_id
        self.assertEqual((session / "raw.pcd").read_bytes(), original)
        self.assertEqual((derived / "raw.pcd").read_bytes(), b"CLEANED COPY\n")
        metadata = yaml.safe_load((derived / "session.yaml").read_text(encoding="utf-8"))
        self.assertEqual(metadata["origin"], "cluster-cleaned")
        self.assertEqual(metadata["source_session"], session.name)
        self.assertEqual(metadata["removed_clusters"], [7])
        self.assertTrue(result["original_preserved"])
        self.assertNotEqual(result["session_id"], session.name)

    def test_archive_records_selected_mapping_algorithm(self):
        backend.CURRENT_RAW_PCD.parent.mkdir(parents=True)
        backend.CURRENT_RAW_PCD.write_text("VERSION 0.7\n", encoding="ascii")
        runtime = {
            "status": "idle",
            "mode": "mapping",
            "algorithm": "fastlio2",
            "run_id": "run-20260803-archive",
            "capture_path": str(backend.CURRENT_RAW_PCD),
            "capture_dir": None,
            "capture_baseline": None,
            "capture_status": "pending",
            "capture_session_id": None,
        }

        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(backend.runtime_manager, "mark_capture") as mark_capture,
        ):
            result = backend.archive_session(
                backend.ArchiveRequest(name="FAST-LIO2 lab", site="lab", note="test")
            )

        session_path = backend.SESSIONS_ROOT / result["session_id"]
        metadata = yaml.safe_load((session_path / "session.yaml").read_text(encoding="utf-8"))
        self.assertEqual(metadata["algorithm"], "fastlio2")
        self.assertEqual(metadata["run_id"], runtime["run_id"])
        self.assertEqual(metadata["source_identity"], backend.capture_identity(backend.CURRENT_RAW_PCD))
        mark_capture.assert_called_once_with("archived", result["session_id"])
        self.assertEqual(backend.list_sessions()[0]["algorithm"], "fastlio2")

    def test_archive_rejects_pcd_that_was_not_changed_by_current_run(self):
        backend.CURRENT_RAW_PCD.parent.mkdir(parents=True)
        backend.CURRENT_RAW_PCD.write_text("OLD MAP\n", encoding="ascii")
        runtime = {
            "status": "idle",
            "mode": "mapping",
            "algorithm": "faster_lio",
            "run_id": "run-20260803-stale",
            "capture_path": str(backend.CURRENT_RAW_PCD),
            "capture_baseline": backend.capture_identity(backend.CURRENT_RAW_PCD),
            "capture_status": "pending",
        }
        with patch.object(backend.runtime_manager, "snapshot", return_value=runtime):
            with self.assertRaises(Exception) as context:
                backend.archive_session(backend.ArchiveRequest(name="stale"))
        self.assertEqual(context.exception.status_code, 409)

    def test_discard_removes_only_current_isolated_capture(self):
        capture_dir = backend.runtime_output_root() / "run-20260803-discard"
        capture_dir.mkdir(parents=True)
        capture_path = capture_dir / "scans.pcd"
        capture_path.write_text("NEW MAP\n", encoding="ascii")
        runtime = {
            "status": "idle",
            "mode": "mapping",
            "algorithm": "fastlio2",
            "run_id": capture_dir.name,
            "capture_path": str(capture_path),
            "capture_dir": str(capture_dir),
            "capture_baseline": None,
            "capture_status": "pending",
            "capture_session_id": None,
        }
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(backend.runtime_manager, "stop"),
            patch.object(backend.runtime_manager, "mark_capture") as mark_capture,
        ):
            result = backend.discard_mapping()
        self.assertTrue(result["discarded"])
        self.assertFalse(capture_dir.exists())
        mark_capture.assert_called_once_with("discarded")

    def test_mapping_algorithms_use_isolated_output_directory(self):
        for algorithm in ("faster_lio", "fastlio2"):
            with self.subTest(algorithm=algorithm):
                runtime = {"status": "idle", "mode": None}
                with (
                    patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
                    patch.object(
                        backend.runtime_manager,
                        "start",
                        return_value={"status": "running"},
                    ) as start,
                ):
                    result = backend.start_runtime(
                        backend.RuntimeRequest(mode="mapping", algorithm=algorithm)
                    )
                self.assertEqual(result["status"], "running")
                kwargs = start.call_args.kwargs
                self.assertEqual(kwargs["capture_path"], kwargs["capture_dir"] / "scans.pcd")
                self.assertEqual(
                    kwargs["environment"]["GO2_MAPPING_OUTPUT_DIR"],
                    str(kwargs["capture_dir"]),
                )
                self.assertTrue(kwargs["capture_dir"].is_dir())
                kwargs["capture_dir"].rmdir()

    def test_start_auto_archives_previous_pending_capture(self):
        previous_dir = backend.runtime_output_root() / "run-20260806-pending"
        previous_dir.mkdir(parents=True)
        previous_pcd = previous_dir / "scans.pcd"
        previous_pcd.write_text("VERSION 0.7\nPOINTS 1\n", encoding="ascii")
        runtime = {
            "status": "idle",
            "mode": "mapping",
            "algorithm": "fastlio2",
            "run_id": previous_dir.name,
            "started_at": "2026-08-06T01:04:52+08:00",
            "capture_path": str(previous_pcd),
            "capture_dir": str(previous_dir),
            "capture_baseline": None,
            "capture_status": "pending",
            "capture_session_id": None,
        }
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(
                backend.runtime_manager,
                "start",
                return_value={"status": "running", "mode": "mapping"},
            ) as start,
            patch.object(backend.runtime_manager, "mark_capture") as mark_capture,
        ):
            result = backend.start_runtime(
                backend.RuntimeRequest(mode="mapping", algorithm="fastlio2")
            )

        session_id = result["auto_archived_session_id"]
        self.assertIsNotNone(session_id)
        self.assertEqual(
            (backend.SESSIONS_ROOT / session_id / "raw.pcd").read_text(encoding="ascii"),
            "VERSION 0.7\nPOINTS 1\n",
        )
        metadata = yaml.safe_load(
            (backend.SESSIONS_ROOT / session_id / "session.yaml").read_text(encoding="utf-8")
        )
        self.assertIn("自动保存", metadata["name"])
        self.assertEqual(metadata["run_id"], previous_dir.name)
        self.assertFalse(previous_dir.exists())
        mark_capture.assert_called_once_with("archived", session_id)

        new_capture_dir = start.call_args.kwargs["capture_dir"]
        self.assertTrue(new_capture_dir.is_dir())
        new_capture_dir.rmdir()

    def test_mapping_stop_allows_time_for_pcd_flush(self):
        runtime = {
            "status": "running",
            "mode": "mapping",
            "algorithm": "faster_lio",
            "capture_status": "pending",
        }
        stopped = {"status": "idle", "mode": "mapping"}
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(backend.runtime_manager, "stop", return_value=stopped) as stop,
            patch.object(backend, "current_capture", return_value={"available": True}),
        ):
            result = backend.stop_runtime()
        stop.assert_called_once_with(timeout=75.0)
        self.assertTrue(result["current_capture"]["available"])
        self.assertIsNone(result["save_error"])

    def test_mapping_stop_reports_missing_fastlio2_pcd(self):
        runtime = {
            "status": "running",
            "mode": "mapping",
            "algorithm": "fastlio2",
            "capture_status": "pending",
        }
        stopped = {"status": "idle", "mode": "mapping", "algorithm": "fastlio2"}
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(backend.runtime_manager, "stop", return_value=stopped),
            patch.object(backend, "current_capture", return_value={"available": False}),
        ):
            result = backend.stop_runtime()

        self.assertIn("没有生成 PCD", result["save_error"])
        self.assertFalse(result["current_capture"]["available"])

    def test_build_worker_forwards_configurable_filter_parameters(self):
        session = self.create_session("session-20260805-filter-params")
        version_id = "map-20260805-filter-params"
        commands = {}

        def run(command, label, **_):
            commands[label] = command
            Path(command[2]).write_text("VERSION 0.7\n", encoding="ascii")

        occupancy_source = None

        def export(source, output_dir, __):
            nonlocal occupancy_source
            occupancy_source = source
            (output_dir / "map.pgm").write_bytes(b"P5\n2 2\n255\n\x00\xfe\xfe\x00")
            (output_dir / "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")

        request = backend.BuildRequest(
            session_id=session.name,
            name="可调滤波地图",
            statistical_mean_k=42,
            statistical_std_dev_mul=0.75,
            radius=0.18,
            radius_min_points=9,
            voxel_leaf=0.2,
            z_min=0.3,
            z_max=1.7,
            resolution=0.04,
        )
        with (
            patch.object(backend, "run_command", side_effect=run),
            patch.object(backend, "export_occupancy_map", side_effect=export),
            patch.object(
                backend,
                "generate_cloud_preview",
                side_effect=lambda _, output, __: output.write_text("ply\n", encoding="ascii"),
            ),
        ):
            backend.build_version_worker(request, version_id)

        statistical = commands["统计离群点过滤"]
        radius = commands["半径离群点过滤"]
        self.assertEqual(occupancy_source.name, "sliced.pcd")
        self.assertNotIn("2D 投影体素降采样", commands)
        self.assertEqual(statistical[statistical.index("-mean_k") + 1], "42")
        self.assertEqual(statistical[statistical.index("-std_dev_mul") + 1], "0.75")
        self.assertEqual(radius[radius.index("-radius") + 1], "0.18")
        self.assertEqual(radius[radius.index("-min_pts") + 1], "9")
        manifest = yaml.safe_load(
            (backend.VERSIONS_ROOT / version_id / "manifest.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(manifest["parameters"]["statistical_mean_k"], 42)
        self.assertEqual(manifest["parameters"]["radius_min_points"], 9)

    def test_occupancy_export_writes_files_without_map_saver_dds_roundtrip(self):
        output_dir = self.versions_root / ".map-test.building"
        output_dir.mkdir()
        sliced = output_dir / "sliced.pcd"
        sliced.write_text("VERSION 0.7\n", encoding="ascii")
        process = MagicMock()
        process.poll.return_value = None

        def start(command, **kwargs):
            self.assertIn(f"map_output_prefix:={output_dir / 'map'}", command)
            self.assertNotIn("map_saver_cli", command)
            (output_dir / "map.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
            (output_dir / "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")
            return process

        with (
            patch.object(backend, "start_managed_process", side_effect=start),
            patch.object(backend, "terminate_managed_process") as terminate,
            patch.object(backend, "unregister_process") as unregister,
        ):
            backend.export_occupancy_map(sliced, output_dir, backend.BuildRequest(
                session_id="session-test", name="test"
            ))

        terminate.assert_called_once_with(process)
        unregister.assert_called_once_with(process)
        self.assertTrue((output_dir / "map.pgm").is_file())
        self.assertEqual(yaml.safe_load((output_dir / "map.yaml").read_text())["image"], "map.pgm")

    def test_occupancy_export_surfaces_converter_log_on_failure(self):
        output_dir = self.versions_root / ".map-failed.building"
        output_dir.mkdir()
        sliced = output_dir / "sliced.pcd"
        sliced.write_text("VERSION 0.7\n", encoding="ascii")
        process = MagicMock(returncode=7)
        process.poll.return_value = 7

        def start(command, **kwargs):
            kwargs["stdout"].write("converter detail\n")
            kwargs["stdout"].flush()
            return process

        with (
            patch.object(backend, "start_managed_process", side_effect=start),
            patch.object(backend, "terminate_managed_process"),
            patch.object(backend, "unregister_process"),
            patch.object(backend, "log_job") as log,
        ):
            with self.assertRaisesRegex(RuntimeError, "退出码 7"):
                backend.export_occupancy_map(sliced, output_dir, backend.BuildRequest(
                    session_id="session-test", name="test"
                ))

        self.assertIn("converter detail", [call.args[0] for call in log.call_args_list])

    def test_fused_localization_module_is_forwarded_to_runtime(self):
        version = self.create_version("map-20260804-localization")
        backend.activate(backend.ActivateRequest(version_id=version.name))
        runtime = {"status": "idle", "mode": None}
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(
                backend.runtime_manager,
                "start",
                return_value={"status": "running"},
            ) as start,
        ):
            result = backend.start_runtime(
                backend.RuntimeRequest(
                    mode="localization",
                    algorithm="fused_ekf",
                )
            )

        self.assertEqual(result["status"], "running")
        self.assertEqual(start.call_args.args[:2], ("localization", "fused_ekf"))

    def test_fused_navigation_module_is_forwarded_to_runtime(self):
        version = self.create_version("map-20260804-fused-navigation")
        backend.activate(backend.ActivateRequest(version_id=version.name))
        runtime = {"status": "idle", "mode": None}
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(
                backend.runtime_manager,
                "start",
                return_value={"status": "running"},
            ) as start,
        ):
            result = backend.start_runtime(
                backend.RuntimeRequest(
                    mode="navigation",
                    algorithm="fused_ekf",
                )
            )

        self.assertEqual(result["status"], "running")
        self.assertEqual(start.call_args.args[:2], ("navigation", "fused_ekf"))

    def test_emergency_cleanup_stops_managed_and_orphan_processes(self):
        cleanup_result = {
            "stopped_count": 3,
            "stopped": [{"pid": 42, "label": "legacy static TF"}],
            "remaining_count": 0,
            "remaining": [],
            "signals_used": ["SIGINT"],
        }
        with (
            patch.object(backend.waypoint_mission_manager, "close") as close_mission,
            patch.object(backend, "cancel_map_job") as cancel_job,
            patch.object(backend, "terminate_all_managed_processes") as stop_managed,
            patch.object(backend.runtime_manager, "stop") as stop_runtime,
            patch.object(
                backend, "cleanup_project_processes", return_value=cleanup_result
            ) as cleanup,
            patch.object(
                backend.runtime_manager,
                "snapshot",
                return_value={"status": "idle"},
            ),
            patch.object(
                backend.waypoint_mission_manager,
                "snapshot",
                return_value={"status": "idle"},
            ),
            patch.object(backend, "job_snapshot", return_value={"running": False}),
        ):
            result = backend.cleanup_runtime_processes()

        close_mission.assert_called_once_with()
        cancel_job.assert_called_once_with()
        stop_managed.assert_called_once_with()
        stop_runtime.assert_called_once_with(timeout=3.0)
        cleanup.assert_called_once_with(backend.REPO_ROOT)
        self.assertEqual(result["stopped_count"], 3)
        self.assertEqual(result["remaining_count"], 0)
        self.assertEqual(result["errors"], [])


if __name__ == "__main__":
    unittest.main()

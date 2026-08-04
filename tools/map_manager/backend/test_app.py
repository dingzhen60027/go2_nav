import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

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
        (path / "map.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
        (path / "localization.pcd").write_text("VERSION 0.7\n", encoding="ascii")
        return path

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

    def test_fastlio2_start_uses_isolated_output_directory(self):
        runtime = {"status": "idle", "mode": None}
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value=runtime),
            patch.object(backend.runtime_manager, "start", return_value={"status": "running"}) as start,
        ):
            result = backend.start_runtime(
                backend.RuntimeRequest(mode="mapping", algorithm="fastlio2")
            )
        self.assertEqual(result["status"], "running")
        kwargs = start.call_args.kwargs
        self.assertEqual(kwargs["capture_path"], kwargs["capture_dir"] / "scans.pcd")
        self.assertEqual(kwargs["environment"]["GO2_MAPPING_OUTPUT_DIR"], str(kwargs["capture_dir"]))
        self.assertTrue(kwargs["capture_dir"].is_dir())


if __name__ == "__main__":
    unittest.main()

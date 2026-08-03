import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

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


if __name__ == "__main__":
    unittest.main()

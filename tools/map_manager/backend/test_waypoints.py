import json
import tempfile
import textwrap
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.map_manager.backend import app as backend
from tools.map_manager.backend.waypoints import WaypointMissionManager, WaypointStore


POSE = {
    "position": {"x": 1.25, "y": -2.5, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 2.0},
}


class WaypointStoreTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "waypoints.yaml"
        self.store = WaypointStore(self.path)

    def tearDown(self):
        self.temporary.cleanup()

    def test_crud_reorder_and_map_isolation(self):
        first = self.store.add("map-one", POSE)
        second = self.store.add("map-one", {
            **POSE,
            "position": {"x": 4.0, "y": 5.0, "z": 0.0},
        })
        other = self.store.add("map-two", POSE, "另一张地图")

        self.assertEqual(first["name"], "目标点 1")
        self.assertEqual(second["name"], "目标点 2")
        self.assertAlmostEqual(first["orientation"]["w"], 1.0)
        self.assertEqual([item["id"] for item in self.store.list("map-one")], [first["id"], second["id"]])
        self.assertEqual([item["id"] for item in self.store.list("map-two")], [other["id"]])

        self.store.rename("map-one", first["id"], "装卸区")
        reordered = self.store.reorder("map-one", [second["id"], first["id"]])
        self.assertEqual([item["name"] for item in reordered], ["目标点 2", "装卸区"])
        self.store.delete("map-one", second["id"])
        self.assertEqual([item["name"] for item in self.store.list("map-one")], ["装卸区"])

    def test_invalid_pose_and_incomplete_reorder_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "四元数"):
            self.store.add("map-one", {
                "position": {"x": 0, "y": 0, "z": 0},
                "orientation": {"x": 0, "y": 0, "z": 0, "w": 0},
            })
        waypoint = self.store.add("map-one", POSE)
        with self.assertRaisesRegex(ValueError, "全部目标点"):
            self.store.reorder("map-one", [waypoint["id"], "wp-20260805-invalid"])


class WaypointMissionManagerTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.fake_executor = self.root / "fake_executor.py"
        self.fake_executor.write_text(textwrap.dedent("""
            import argparse
            import json
            import os
            import signal
            import time
            from pathlib import Path

            parser = argparse.ArgumentParser()
            parser.add_argument("command")
            parser.add_argument("--mission")
            parser.add_argument("--status")
            parser.add_argument("--output")
            parser.add_argument("--timeout")
            args = parser.parse_args()

            if args.command == "pose":
                Path(args.output).write_text(json.dumps({
                    "position": {"x": 1, "y": 2, "z": 0},
                    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                }))
                raise SystemExit(0)

            mission = json.loads(Path(args.mission).read_text())
            status_path = Path(args.status)
            cancelled = False
            def cancel(*_):
                global cancelled
                cancelled = True
            signal.signal(signal.SIGINT, cancel)
            status_path.write_text(json.dumps({
                "status": "running", "pid": os.getpid(),
                "mission_id": mission["mission_id"], "map_id": mission["map_id"],
                "total": len(mission["waypoints"]), "processed": 0,
                "progress_percent": 12.5, "message": "fake running",
            }))
            while not cancelled:
                time.sleep(0.02)
            status_path.write_text(json.dumps({
                "status": "cancelled", "pid": None,
                "mission_id": mission["mission_id"], "map_id": mission["map_id"],
                "total": len(mission["waypoints"]), "processed": 0,
                "progress_percent": 12.5, "message": "fake cancelled",
            }))
        """), encoding="utf-8")
        self.manager = WaypointMissionManager(
            self.root,
            self.root / "state",
            executor_path=self.fake_executor,
        )

    def tearDown(self):
        self.manager.close()
        self.temporary.cleanup()

    def wait_for(self, status, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.manager.snapshot()
            if state["status"] == status:
                return state
            time.sleep(0.02)
        self.fail(f"mission did not reach {status}: {self.manager.snapshot()}")

    def test_pose_capture_and_realtime_cancel(self):
        pose = self.manager.capture_pose(timeout_seconds=1.0)
        self.assertEqual(pose["position"]["x"], 1.0)

        state = self.manager.start(
            "mission-20260805-test",
            "map-20260805-test",
            [{"id": "wp-20260805-test", "name": "A", **POSE}],
            stop_on_failure=True,
            waypoint_timeout_sec=300,
            pause_between_sec=0,
        )
        self.assertTrue(state["active"])
        self.assertEqual(self.wait_for("running")["progress_percent"], 12.5)
        cancelling = self.manager.cancel()
        self.assertEqual(cancelling["status"], "cancelling")
        terminal = self.wait_for("cancelled")
        self.assertFalse(terminal["active"])
        self.assertEqual(terminal["message"], "fake cancelled")


class WaypointApiTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.versions = root / "versions"
        self.versions.mkdir()
        self.state_path = root / "state.json"
        self.store = WaypointStore(root / "waypoints.yaml")
        version = self.versions / "map-20260805-active"
        version.mkdir()
        (version / "manifest.yaml").write_text("schema: 1\n")
        (version / "map.yaml").write_text("image: map.pgm\n")
        (version / "map.pgm").write_bytes(b"P5\n2 2\n255\n\x00\xfe\xfe\x00")
        (version / "localization.pcd").write_text("VERSION 0.7\n")
        self.state_path.write_text(json.dumps({"active_id": version.name}))
        self.patchers = [
            patch.object(backend, "VERSIONS_ROOT", self.versions),
            patch.object(backend, "STATE_PATH", self.state_path),
            patch.object(backend, "waypoint_store", self.store),
        ]
        for patcher in self.patchers:
            patcher.start()

    def tearDown(self):
        for patcher in reversed(self.patchers):
            patcher.stop()
        self.temporary.cleanup()

    def test_capture_requires_navigation_and_start_snapshots_waypoints(self):
        with patch.object(backend.runtime_manager, "snapshot", return_value={"status": "idle"}):
            with self.assertRaises(Exception) as context:
                backend.capture_waypoint(backend.CaptureWaypointRequest())
        self.assertEqual(context.exception.status_code, 409)

        mission = {"status": "idle"}
        with (
            patch.object(backend.runtime_manager, "snapshot", return_value={"status": "running", "mode": "navigation"}),
            patch.object(backend.waypoint_mission_manager, "snapshot", return_value=mission),
            patch.object(backend.waypoint_mission_manager, "capture_pose", return_value=POSE),
        ):
            captured = backend.capture_waypoint(backend.CaptureWaypointRequest(name="巡检点"))
        self.assertEqual(captured["waypoint"]["name"], "巡检点")

        with (
            patch.object(backend.runtime_manager, "snapshot", return_value={"status": "running", "mode": "navigation"}),
            patch.object(backend, "job_snapshot", return_value={"running": False}),
            patch.object(backend.waypoint_mission_manager, "start", return_value={"status": "queued"}) as start,
            patch.object(backend, "make_id", return_value="mission-20260805-api"),
        ):
            result = backend.start_waypoint_mission(backend.WaypointMissionRequest())
        self.assertEqual(result["status"], "queued")
        self.assertEqual(start.call_args.args[1], "map-20260805-active")
        self.assertEqual(start.call_args.args[2][0]["name"], "巡检点")


if __name__ == "__main__":
    unittest.main()

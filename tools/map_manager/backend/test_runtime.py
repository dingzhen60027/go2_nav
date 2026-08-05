import tempfile
import time
import unittest
from pathlib import Path

from tools.map_manager.backend.runtime import RuntimeManager


class RuntimeManagerTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        script = self.root / "start_mapping.sh"
        script.write_text(
            "#!/bin/bash\n"
            "trap 'exit 0' INT TERM\n"
            "echo READY\n"
            "while true; do sleep 1; done\n",
            encoding="ascii",
        )
        fastlio2_script = self.root / "start_mapping_fastlio2.sh"
        fastlio2_script.write_text(
            "#!/bin/bash\n"
            "trap 'exit 0' INT TERM\n"
            "echo FASTLIO2_READY\n"
            "echo OUTPUT:$GO2_MAPPING_OUTPUT_DIR\n"
            "while true; do sleep 1; done\n",
            encoding="ascii",
        )
        for name, marker in (
            ("start_localization.sh", "PURE_ICP_READY"),
            ("start_fused_localization.sh", "FUSED_EKF_READY"),
            ("start_navigation.sh", "PURE_ICP_NAV_READY"),
            ("start_fused_navigation.sh", "FUSED_EKF_NAV_READY"),
        ):
            (self.root / name).write_text(
                "#!/bin/bash\n"
                "trap 'exit 0' INT TERM\n"
                f"echo {marker}\n"
                "while true; do sleep 1; done\n",
                encoding="ascii",
            )
        self.manager = RuntimeManager(self.root, self.root / "runtime.json")

    def tearDown(self):
        self.manager.close()
        self.temp_dir.cleanup()

    def test_start_streams_logs_and_stop_closes_process_group(self):
        started = self.manager.start("mapping")
        self.assertEqual(started["status"], "running")
        pid = started["pid"]
        for _ in range(30):
            if any("READY" in line for line in self.manager.snapshot()["logs"]):
                break
            time.sleep(0.05)
        self.assertTrue(any("READY" in line for line in self.manager.snapshot()["logs"]))

        stopped = self.manager.stop(timeout=1.0)
        self.assertEqual(stopped["status"], "idle")
        self.assertIsNone(stopped["pid"])
        self.assertFalse(Path(f"/proc/{pid}").exists())

    def test_selects_fastlio2_without_changing_runtime_mode(self):
        capture_dir = self.root / "capture" / "run-1"
        capture_path = capture_dir / "scans.pcd"
        started = self.manager.start(
            "mapping",
            "fastlio2",
            environment={"GO2_MAPPING_OUTPUT_DIR": str(capture_dir)},
            run_id="run-1",
            capture_path=capture_path,
            capture_dir=capture_dir,
        )
        self.assertEqual(started["mode"], "mapping")
        self.assertEqual(started["algorithm"], "fastlio2")
        self.assertEqual(started["run_id"], "run-1")
        self.assertEqual(started["capture_path"], str(capture_path))
        for _ in range(30):
            if any("FASTLIO2_READY" in line for line in self.manager.snapshot()["logs"]):
                break
            time.sleep(0.05)
        self.assertTrue(any("FASTLIO2_READY" in line for line in self.manager.snapshot()["logs"]))
        self.assertTrue(any(f"OUTPUT:{capture_dir}" in line for line in self.manager.snapshot()["logs"]))

        self.manager.stop(timeout=1.0)
        self.manager.mark_capture("discarded")
        recovered = RuntimeManager(self.root, self.root / "runtime.json")
        recovered.recover_stale_process()
        self.assertEqual(recovered.snapshot()["run_id"], "run-1")
        self.assertEqual(recovered.snapshot()["capture_status"], "discarded")

    def test_stop_waits_for_delayed_mapping_capture(self):
        capture_dir = self.root / "capture" / "run-delayed"
        capture_dir.mkdir(parents=True)
        capture_path = capture_dir / "scans.pcd"
        (self.root / "start_mapping.sh").write_text(
            "#!/bin/bash\n"
            "finish() {\n"
            "  sleep 0.3\n"
            "  printf 'VERSION 0.7\\n' > \"$GO2_MAPPING_OUTPUT_DIR/scans.pcd\"\n"
            "  exit 0\n"
            "}\n"
            "trap finish INT TERM\n"
            "echo READY\n"
            "while true; do sleep 1; done\n",
            encoding="ascii",
        )
        self.manager.start(
            "mapping",
            "faster_lio",
            environment={"GO2_MAPPING_OUTPUT_DIR": str(capture_dir)},
            run_id="run-delayed",
            capture_path=capture_path,
            capture_dir=capture_dir,
        )
        for _ in range(30):
            if any("READY" in line for line in self.manager.snapshot()["logs"]):
                break
            time.sleep(0.05)
        self.assertTrue(any("READY" in line for line in self.manager.snapshot()["logs"]))

        stopped = self.manager.stop(timeout=2.0)

        self.assertEqual(stopped["status"], "idle")
        self.assertTrue(capture_path.is_file())
        self.assertEqual(capture_path.read_text(encoding="ascii"), "VERSION 0.7\n")

    def test_project_fastlio2_script_saves_before_stopping_launch_processes(self):
        project_root = Path(__file__).resolve().parents[3]
        script_text = (project_root / "start_mapping_fastlio2.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn('SAVE_SERVICE="/fastlio2/save"', script_text)
        cleanup_body = script_text.split("cleanup() {", 1)[1].split("shutdown() {", 1)[0]
        self.assertLess(
            cleanup_body.index("save_fastlio2_capture"),
            cleanup_body.index("jobs -pr | xargs -r kill -INT"),
        )

    def test_fused_navigation_binds_go2_dds_and_owns_child_process_groups(self):
        project_root = Path(__file__).resolve().parents[3]
        script_text = (project_root / "start_fused_navigation.sh").read_text(
            encoding="utf-8"
        )

        adapter_launch = script_text.index("setsid ros2 run go2_bridge go2_cmd_adapter")
        self.assertLess(script_text.index("export CYCLONEDDS_URI"), adapter_launch)
        self.assertNotIn('if [ -z "${CYCLONEDDS_URI:-}" ]', script_text)
        self.assertIn('/sys/class/net/${GO2_DDS_IFACE}/carrier', script_text)
        self.assertLess(script_text.index("wait_for_go2_state"), adapter_launch)
        self.assertIn(
            "/sportmodestate unitree_go/msg/SportModeState", script_text
        )
        self.assertNotIn("wait_for_sport_api_subscriber", script_text)
        self.assertLess(script_text.index("wait_for_sport_api_publisher"), script_text.index(
            "setsid ros2 launch nav2_bringup"
        ))
        self.assertIn("Publisher count: [1-9][0-9]*", script_text)
        self.assertIn('kill -TERM -- "-$pid"', script_text)

    def test_rejects_unknown_mapping_algorithm(self):
        with self.assertRaises(ValueError):
            self.manager.start("mapping", "unknown")

    def test_localization_modules_use_independent_scripts(self):
        for module, marker in (
            ("pure_icp", "PURE_ICP_READY"),
            ("fused_ekf", "FUSED_EKF_READY"),
        ):
            started = self.manager.start("localization", module)
            self.assertEqual(started["mode"], "localization")
            self.assertEqual(started["algorithm"], module)
            for _ in range(30):
                if any(marker in line for line in self.manager.snapshot()["logs"]):
                    break
                time.sleep(0.05)
            self.assertTrue(any(marker in line for line in self.manager.snapshot()["logs"]))
            self.manager.stop(timeout=1.0)

    def test_localization_defaults_to_legacy_pure_icp(self):
        started = self.manager.start("localization")
        self.assertEqual(started["algorithm"], "pure_icp")
        self.manager.stop(timeout=1.0)

    def test_rejects_unknown_localization_module(self):
        with self.assertRaises(ValueError):
            self.manager.start("localization", "unknown")

    def test_navigation_uses_selected_localization_algorithm(self):
        for module, marker in (
            ("pure_icp", "PURE_ICP_NAV_READY"),
            ("fused_ekf", "FUSED_EKF_NAV_READY"),
        ):
            started = self.manager.start("navigation", module)
            self.assertEqual(started["mode"], "navigation")
            self.assertEqual(started["algorithm"], module)
            for _ in range(30):
                if any(marker in line for line in self.manager.snapshot()["logs"]):
                    break
                time.sleep(0.05)
            self.assertTrue(any(marker in line for line in self.manager.snapshot()["logs"]))
            self.manager.stop(timeout=1.0)

    def test_navigation_defaults_to_legacy_pure_icp(self):
        started = self.manager.start("navigation")
        self.assertEqual(started["algorithm"], "pure_icp")
        for _ in range(30):
            if any("PURE_ICP_NAV_READY" in line for line in self.manager.snapshot()["logs"]):
                break
            time.sleep(0.05)
        self.assertTrue(any("PURE_ICP_NAV_READY" in line for line in self.manager.snapshot()["logs"]))
        self.manager.stop(timeout=1.0)

    def test_rejects_unknown_navigation_module(self):
        with self.assertRaises(ValueError):
            self.manager.start("navigation", "unknown")


if __name__ == "__main__":
    unittest.main()

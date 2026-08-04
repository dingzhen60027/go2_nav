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

    def test_rejects_unknown_mapping_algorithm(self):
        with self.assertRaises(ValueError):
            self.manager.start("mapping", "unknown")


if __name__ == "__main__":
    unittest.main()

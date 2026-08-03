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


if __name__ == "__main__":
    unittest.main()

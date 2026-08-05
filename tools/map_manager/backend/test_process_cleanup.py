import os
import unittest
from pathlib import Path

from tools.map_manager.backend.process_cleanup import classify_project_process


class ProcessClassificationTest(unittest.TestCase):
    def setUp(self):
        self.repo = Path("/home/operator/go2_nav")

    def test_matches_legacy_static_tf(self):
        label = classify_project_process(
            (
                "/opt/ros/humble/lib/tf2_ros/static_transform_publisher",
                "0",
                "0",
                "0.3",
                "0",
                "0",
                "0",
                "base_link",
                "livox_frame",
            ),
            "/opt/ros/humble/lib/tf2_ros/static_transform_publisher",
            self.repo,
        )
        self.assertEqual(label, "legacy static TF")

    def test_matches_project_install_binary(self):
        executable = self.repo / "install/go2_localization/lib/go2_localization/fused_icp_matcher"
        label = classify_project_process((str(executable),), str(executable), self.repo)
        self.assertEqual(label, "fused ICP matcher")

    def test_matches_installed_python_console_script(self):
        script = self.repo / "install/go2_description/lib/go2_description/neutral_joint_state_publisher"
        label = classify_project_process(
            ("python3", str(script), "--ros-args"),
            "/usr/bin/python3.10",
            self.repo,
        )
        self.assertEqual(label, "Go2 neutral joint publisher")

    def test_matches_only_project_nav2_container(self):
        executable = "/opt/ros/humble/lib/rclcpp_components/component_container_isolated"
        self.assertEqual(
            classify_project_process(
                (executable, "--ros-args", "-r", "__node:=nav2_container"),
                executable,
                self.repo,
            ),
            "Nav2 component container",
        )
        self.assertIsNone(
            classify_project_process(
                (executable, "--ros-args", "-r", "__node:=camera_container"),
                executable,
                self.repo,
            )
        )

    def test_matches_known_ros2_launch(self):
        label = classify_project_process(
            ("/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch", "nav2_bringup", "bringup_launch.py"),
            "/usr/bin/python3.10",
            self.repo,
        )
        self.assertEqual(label, "nav2_bringup launch")

    def test_preserves_web_backend_and_unrelated_programs(self):
        web_command = (
            "/usr/bin/python3",
            "-m",
            "uvicorn",
            "tools.map_manager.backend.app:app",
        )
        self.assertIsNone(
            classify_project_process(web_command, "/usr/bin/python3.10", self.repo)
        )
        self.assertIsNone(
            classify_project_process(
                ("/usr/bin/rviz2", "-d", "/tmp/another_robot.rviz"),
                "/usr/bin/rviz2",
                self.repo,
            )
        )
        self.assertIsNone(
            classify_project_process(
                ("/usr/bin/sleep", "60"), "/usr/bin/sleep", self.repo
            )
        )
        self.assertIsNone(
            classify_project_process(
                ("npm", "run", "build"),
                str(self.repo / "install/node/bin/node"),
                self.repo,
            )
        )

    def test_matches_workspace_start_script_but_not_map_manager(self):
        navigation = str(self.repo / "start_navigation.sh")
        self.assertEqual(
            classify_project_process(("/bin/bash", navigation), "/bin/bash", self.repo),
            "start_navigation.sh",
        )
        manager = str(self.repo / "start_map_manager.sh")
        self.assertIsNone(
            classify_project_process(("/bin/bash", manager), "/bin/bash", self.repo)
        )


if __name__ == "__main__":
    unittest.main()

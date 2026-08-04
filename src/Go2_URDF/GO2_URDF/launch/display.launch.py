from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("go2_description"))
    robot_description = (package_share / "urdf" / "go2_description.urdf").read_text()
    rviz_config = str(package_share / "rviz" / "go2_description.rviz")

    use_rviz = LaunchConfiguration("use_rviz")
    publish_neutral_joints = LaunchConfiguration("publish_neutral_joints")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the Go2 display configuration.",
            ),
            DeclareLaunchArgument(
                "publish_neutral_joints",
                default_value="true",
                description="Publish a fixed neutral pose for URDF display only.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="go2_robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="go2_description",
                executable="neutral_joint_state_publisher",
                name="go2_neutral_joint_state_publisher",
                condition=IfCondition(publish_neutral_joints),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="go2_description_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )

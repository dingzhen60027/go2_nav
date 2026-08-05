from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("go2_localization"))
    description_share = Path(get_package_share_directory("go2_description"))
    livox_share = Path(get_package_share_directory("livox_ros_driver2"))

    localization_config = str(package_share / "config" / "localization.yaml")
    rviz_config = str(package_share / "rviz" / "fused_localization.rviz")

    map_pcd = LaunchConfiguration("map_pcd")
    sport_state_topic = LaunchConfiguration("sport_state_topic")
    start_description = LaunchConfiguration("start_description")
    start_livox = LaunchConfiguration("start_livox")
    start_icp = LaunchConfiguration("start_icp")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_pcd",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable(
                            "GO2_NAV_ROOT",
                            default_value=str(Path.home() / "go2_nav"),
                        ),
                        "maps",
                        "active",
                        "localization.pcd",
                    ]
                ),
                description="PCD map used by the independent fused ICP matcher.",
            ),
            DeclareLaunchArgument(
                "sport_state_topic",
                default_value="/sportmodestate",
                description="Go2 SportModeState topic.",
            ),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_icp", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            str(description_share / "launch" / "display.launch.py")
                        ),
                        launch_arguments={
                            "use_rviz": "false",
                            # The Go2 ROS2 stack does not provide joint_states
                            # on its own. Publish a neutral pose so the leg
                            # revolute links receive TF and RobotModel renders
                            # the complete body before any joint driver exists.
                            "publish_neutral_joints": "true",
                        }.items(),
                    ),
                ],
                scoped=True,
                condition=IfCondition(start_description),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(livox_share / "launch_ROS2" / "msg_MID360_launch.py")
                ),
                condition=IfCondition(start_livox),
            ),
            Node(
                package="go2_localization",
                executable="sport_state_adapter",
                name="sport_state_adapter",
                output="screen",
                parameters=[
                    localization_config,
                    {"input_topic": sport_state_topic},
                ],
            ),
            Node(
                package="go2_localization",
                executable="icp_fusion_bridge",
                name="icp_fusion_bridge",
                output="screen",
                parameters=[localization_config],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_local",
                output="screen",
                parameters=[localization_config],
                remappings=[
                    ("odometry/filtered", "/localization/odometry/local"),
                    ("set_pose", "/localization/local/set_pose"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global",
                output="screen",
                parameters=[localization_config],
                remappings=[
                    ("odometry/filtered", "/localization/odometry/global"),
                    ("set_pose", "/localization/global/set_pose"),
                ],
            ),
            Node(
                package="go2_localization",
                executable="fused_icp_matcher",
                name="fused_icp_matcher",
                output="screen",
                parameters=[
                    localization_config,
                    {
                        "map_pcd": map_pcd,
                    },
                ],
                condition=IfCondition(start_icp),
            ),
            Node(
                package="go2_localization",
                executable="obstacle_cloud_filter",
                name="obstacle_cloud_filter",
                output="screen",
                parameters=[localization_config],
                condition=IfCondition(start_icp),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="fused_localization_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )

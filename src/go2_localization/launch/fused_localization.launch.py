from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("go2_localization"))
    icp_share = Path(get_package_share_directory("fast_icp_loc"))
    description_share = Path(get_package_share_directory("go2_description"))
    livox_share = Path(get_package_share_directory("livox_ros_driver2"))

    localization_config = str(package_share / "config" / "localization.yaml")
    icp_config = str(icp_share / "config" / "fast_icp_loc.yaml")
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
                default_value="/home/wjg/go2_nav/maps/active/localization.pcd",
                description="PCD map used by Fast ICP.",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "display.launch.py")
                ),
                launch_arguments={
                    "use_rviz": "false",
                    "publish_neutral_joints": "false",
                }.items(),
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
                package="fast_icp_loc",
                executable="fast_icp_loc_node",
                name="fast_icp_fused",
                output="screen",
                parameters=[
                    icp_config,
                    {
                        "map_pcd": map_pcd,
                        "body_frame": "icp_tracking_frame",
                        "lidar_frame": "livox_frame",
                        "prediction_topic": "/localization/icp_prediction",
                        "prediction_timeout_sec": 0.25,
                        "publish_only_accepted_pose": True,
                    },
                ],
                remappings=[
                    ("/tf", "/localization/icp_internal_tf"),
                    ("/tf_static", "/localization/icp_internal_tf_static"),
                    ("/icp_pose", "/localization/icp_pose_raw"),
                    ("/initialpose", "/localization/icp_initialpose"),
                ],
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

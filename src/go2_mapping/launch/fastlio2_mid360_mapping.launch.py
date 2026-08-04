from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare("go2_mapping"), "config", "fastlio2_mid360.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"
    ])

    output_dir = LaunchConfiguration("output_dir")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(
            package="fastlio2",
            namespace="fastlio2",
            executable="lio_node",
            name="lio_node",
            output="screen",
            parameters=[{"config_path": LaunchConfiguration("config")}],
        ),
        Node(
            package="go2_mapping",
            namespace="fastlio2",
            executable="map_capture_node",
            name="map_capture",
            output="screen",
            parameters=[{
                "input_topic": "/fastlio2/world_cloud",
                "output_directory": output_dir,
                "file_prefix": "fastlio2_scans",
                "latest_name": "scans.pcd",
                "voxel_leaf": 0.08,
                "compact_every_n_scans": 20,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="fastlio2_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])

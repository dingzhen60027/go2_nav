import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('fast_icp_loc'),
                          'config', 'fast_icp_loc.yaml')
    default_map = '/home/wjg/go2_nav/maps/clean/pcd_icp_latest.pcd'

    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd',
        default_value=default_map,
        description='PCD map used for ICP localization',
    )

    node = Node(
        package='fast_icp_loc',
        executable='fast_icp_loc_node',
        name='fast_icp_loc',
        output='screen',
        parameters=[config, {'map_pcd': LaunchConfiguration('map_pcd')}],
    )

    return LaunchDescription([map_pcd_arg, node])

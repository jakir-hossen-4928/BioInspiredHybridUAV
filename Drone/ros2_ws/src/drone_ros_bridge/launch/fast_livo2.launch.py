"""Webots ROS2 TCP receiver plus optional upstream FAST-LIVO2 ROS2 estimator.

Requires external GPL-licensed FAST-LIVO2 from the U-AMC ROS2 fork and
its dependencies to have been built in the sourced workspace.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    upstream = get_package_share_directory('fast_livo')
    launch_path = os.path.join(upstream, 'launch', 'mapping_mid360_lvi.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='15882'),
        DeclareLaunchArgument('mid360_params_file',
                              default_value=os.path.join(upstream, 'config', 'mid360_lvi.yaml')),
        DeclareLaunchArgument('camera_params_file',
                              default_value=os.path.join(upstream, 'config', 'camera_pinhole.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(package='drone_ros_bridge', executable='webots_receiver',
             output='screen', arguments=['--add-sim-gravity', '--host', LaunchConfiguration('host'),
                                         '--port', LaunchConfiguration('port')]),
        Node(package='drone_ros_bridge', executable='sensor_health', output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path),
                                 launch_arguments={
                                     'mid360_params_file': LaunchConfiguration('mid360_params_file'),
                                     'camera_params_file': LaunchConfiguration('camera_params_file'),
                                     'use_rviz': LaunchConfiguration('use_rviz'),
                                     'use_respawn': 'false',
                                 }.items()),
    ])

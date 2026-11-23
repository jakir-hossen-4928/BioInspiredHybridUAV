"""ROS 2 TCP bridge + sensor health for Windows Webots/WSL Ubuntu."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='15882'),
        Node(package='drone_ros_bridge', executable='webots_receiver',
             name='webots_fast_livo_bridge', output='screen',
             arguments=['--host', LaunchConfiguration('host'), '--port', LaunchConfiguration('port')]),
        Node(package='drone_ros_bridge', executable='sensor_health',
             name='webots_sensor_health', output='screen'),
    ])

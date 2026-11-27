"""Run upstream ROS 2 LIO-SAM with the simulated Webots LiDAR and IMU."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = {
        'use_sim_time': True,
        'pointCloudTopic': '/webots/lidar/points',
        'imuTopic': '/livox/imu',
        'gpsTopic': '/lio_sam/unused_gps',
        'lidarFrame': 'livox_frame',
        'baselinkFrame': 'base_link',
        'sensor': 'velodyne',  # PointCloud2 ring/time schema, fixed instantaneous scan.
        'N_SCAN': 24,
        'Horizon_SCAN': 360,
        'lidarMinRange': 1.,
        'lidarMaxRange': 45.,
        'imuGravity': 9.81,
        'extrinsicTrans': [0., 0., -.555],  # IMU origin relative to LiDAR.
        'numberOfCores': 2,
    }
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('gravity_mode', default_value='zero-world'),
        Node(package='drone_ros_bridge', executable='webots_receiver', output='screen',
             arguments=['--gravity-mode', LaunchConfiguration('gravity_mode'), '--host', LaunchConfiguration('host')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--z', '0.555', '--frame-id', 'base_link', '--child-frame-id', 'livox_frame']),
        *(Node(package='lio_sam', executable='lio_sam_' + stage,
               parameters=[parameters], output='screen')
          for stage in ('imuPreintegration', 'imageProjection', 'featureExtraction', 'mapOptimization')),
    ])

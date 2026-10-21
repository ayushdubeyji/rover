#!/usr/bin/env python3
"""
Rover ROS 2 Bringup Launch.
Brings up:
  - Robot State Publisher (URDF with exact 11cm camera & 10cm +/-33 deg ToF links)
  - Static TF Broadcasters
  - ToF LaserScan & Range Publisher Node
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_rover')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'rover.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # Static TF broadcasters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'static_tf.launch.py')
            )
        ),

        # ToF LaserScan and Range Publisher
        Node(
            package='ros2_rover',
            executable='tof_laserscan_publisher',
            name='tof_laserscan_publisher',
            output='screen',
            parameters=[{
                'udp_port': 8888,
                'publish_rate_hz': 20.0,
                'tof_angle_deg': 33.0,
                'tof_dist_m': 0.100,
                'cam_dist_m': 0.110,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])

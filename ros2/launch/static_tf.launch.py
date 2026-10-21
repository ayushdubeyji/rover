#!/usr/bin/env python3
"""
Static Transform Broadcasters for Mecanum Rover Sensors.
Exact geometry:
  - Camera: 11 cm from centre at front (0 deg)
  - ToF Left: 10 cm from centre, rotated +33 deg (+0.57596 rad)
  - ToF Right: 10 cm from centre, rotated -33 deg (-0.57596 rad)
  - Front Ultrasonic: 11 cm from centre at front (0 deg)
  - IMU: Center of chassis
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # base_footprint -> base_link (ground projection to chassis origin at wheel radius 40mm)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_footprint_to_base_link',
            arguments=['--x', '0', '--y', '0', '--z', '0.040',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
        ),

        # base_link -> camera_link (11 cm from centre, 0 deg)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_camera',
            arguments=['--x', '0.110', '--y', '0.0', '--z', '0.080',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
        ),

        # camera_link -> camera_optical_frame (REP-103 optical convention)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_camera_to_optical',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                       '--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame']
        ),

        # base_link -> tof_left_link (10 cm from centre, +33 deg = 0.575959 rad)
        # x = 0.10 * cos(33°) = 0.083867, y = 0.10 * sin(33°) = 0.054464
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_tof_left',
            arguments=['--x', '0.083867', '--y', '0.054464', '--z', '0.022',
                       '--roll', '0', '--pitch', '0', '--yaw', '0.57595865',
                       '--frame-id', 'base_link', '--child-frame-id', 'tof_left_link']
        ),

        # base_link -> tof_right_link (10 cm from centre, -33 deg = -0.575959 rad)
        # x = 0.10 * cos(-33°) = 0.083867, y = -0.10 * sin(33°) = -0.054464
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_tof_right',
            arguments=['--x', '0.083867', '--y', '-0.054464', '--z', '0.022',
                       '--roll', '0', '--pitch', '0', '--yaw', '-0.57595865',
                       '--frame-id', 'base_link', '--child-frame-id', 'tof_right_link']
        ),

        # base_link -> ultrasonic_link (11 cm from centre, 0 deg)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_ultrasonic',
            arguments=['--x', '0.110', '--y', '0.0', '--z', '0.022',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'ultrasonic_link']
        ),

        # base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.063',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
        ),

        # base_link -> scan (Virtual 2D LaserScan projection frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_scan',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.022',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'scan']
        ),

        # base_link -> mic_array_link (Center mast top: x=0.0m, y=0.0m, z=0.16m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_mic_array',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.160',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'mic_array_link']
        ),

        # base_link -> ld2402_radar_link (Front center: x=+0.11m, y=0.0m, z=0.06m)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_ld2402_radar',
            arguments=['--x', '0.110', '--y', '0.0', '--z', '0.060',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'ld2402_radar_link']
        ),
    ])


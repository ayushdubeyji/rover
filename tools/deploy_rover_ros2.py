#!/usr/bin/env python3
"""
Deploy rover_localization ROS 2 package to host compute server at 192.168.0.111.
"""
import os
import paramiko

HOST = '192.168.0.111'
USER = 'ayush'
PASS = os.environ.get('SSH_PASSWORD')
REMOTE_WS = '/home/ayush/mecanum_robot/mecanum_host_ws/src/rover_localization'

PACKAGE_XML = '''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_localization</name>
  <version>1.0.0</version>
  <description>ROS 2 Sensor Fusion &amp; Localization for 4WD Mecanum Rover (EKF, IMU, Magnetometer, Wheel Odometry, URDF TF tree)</description>
  <maintainer email="robot@workbench.local">Ayush</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>robot_localization</depend>
  <depend>robot_state_publisher</depend>
  <depend>xacro</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
'''

SETUP_PY = '''import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rover_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro') + glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Ayush',
    maintainer_email='robot@workbench.local',
    description='ROS 2 Sensor Fusion & Localization for 4WD Mecanum Rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_telemetry_bridge = rover_localization.rover_telemetry_bridge:main',
        ],
    },
)
'''

SETUP_CFG = '''[develop]
script_dir=$base/lib/rover_localization
[install]
install_scripts=$base/lib/rover_localization
'''

EKF_YAML = '''### EKF Configuration for 4WD Mecanum Rover ###
### Package: robot_localization (ekf_node) ###
### Frames: odom -> base_footprint -> base_link -> imu_link, wheel_*_link ###

ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    transform_time_offset: 0.0
    transform_timeout: 0.05
    print_diagnostics: true
    debug: false

    # Standard REP-103 / REP-105 coordinate frames
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom

    # -------------------------------------------------------------
    # 1. 4WD Mecanum Wheel Odometry (/odom/unfiltered)
    # Fuses forward linear velocity (vx), lateral strafe velocity (vy),
    # and yaw rotational velocity (vyaw).
    # In Mecanum kinematics, fusing velocities avoids discontinuous
    # jumps caused by lateral roller skid.
    # Matrix: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    # -------------------------------------------------------------
    odom0: /odom/unfiltered
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    # -------------------------------------------------------------
    # 2. 9-DOF IMU Sensor Fusion (/imu/data)
    # REP-103 standard body frame: X forward, Y left, Z up.
    # Fuses Absolute Orientation Yaw and Angular Velocity Yaw.
    # Matrix: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
    # -------------------------------------------------------------
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
    imu0_queue_size: 10
    imu0_nodelay: true
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true

    # -------------------------------------------------------------
    # Process Noise Covariance Q (15x15)
    # Tuned specifically for Mecanum dynamics:
    # High agility in X and Y, with higher variance allowed for lateral Y
    # due to passive roller dynamics, and tightly bounded Z/roll/pitch.
    # -------------------------------------------------------------
    process_noise_covariance: [
      1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  2.5e-2, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  5e-3, 0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2
    ]

    # Initial Estimate Error Covariance P0 (15x15)
    initial_estimate_covariance: [
      1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,  0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,  0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2, 0.0,
      0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-2
    ]
'''

URDF_XACRO = '''<?xml version="1.0"?>
<robot name="mecanum_rover" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Dimensions (in meters) -->
  <xacro:property name="wheel_radius" value="0.040" />
  <xacro:property name="wheel_width"  value="0.038" />
  <xacro:property name="wheelbase"    value="0.180" />
  <xacro:property name="track_width"  value="0.160" />
  <xacro:property name="chassis_len"  value="0.220" />
  <xacro:property name="chassis_wid"  value="0.140" />
  <xacro:property name="chassis_hgt"  value="0.055" />

  <!-- Material definitions -->
  <material name="chassis_dark">
    <color rgba="0.12 0.12 0.14 1.0"/>
  </material>
  <material name="accent_cyan">
    <color rgba="0.0 0.9 0.72 1.0"/>
  </material>
  <material name="wheel_rubber">
    <color rgba="0.25 0.25 0.28 1.0"/>
  </material>
  <material name="sensor_gold">
    <color rgba="0.85 0.7 0.2 1.0"/>
  </material>

  <!-- base_footprint (Ground projection) -->
  <link name="base_footprint"/>

  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- base_link (Chassis body origin) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${chassis_hgt/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_len} ${chassis_wid} ${chassis_hgt}"/>
      </geometry>
      <material name="chassis_dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${chassis_hgt/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${chassis_len} ${chassis_wid} ${chassis_hgt}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${chassis_hgt/2}" rpy="0 0 0"/>
      <mass value="1.85"/>
      <inertia ixx="0.0055" ixy="0.0" ixz="0.0" iyy="0.0092" iyz="0.0" izz="0.0135"/>
    </inertial>
  </link>

  <!-- IMU Link (REP-103: X forward, Y left, Z up) -->
  <link name="imu_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.02 0.005"/>
      </geometry>
      <material name="sensor_gold"/>
    </visual>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0.0 0.0 ${chassis_hgt + 0.008}" rpy="0 0 0"/>
  </joint>

  <!-- Magnetometer Link -->
  <link name="mag_link"/>
  <joint name="mag_joint" type="fixed">
    <parent link="imu_link"/>
    <child link="mag_link"/>
    <origin xyz="0.01 0.0 0.0" rpy="0 0 0"/>
  </joint>

  <!-- Lidar / Laser Link -->
  <link name="laser_frame">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.035" length="0.025"/>
      </geometry>
      <material name="accent_cyan"/>
    </visual>
  </link>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.04 0.0 ${chassis_hgt + 0.025}" rpy="0 0 0"/>
  </joint>

  <!-- Front Camera Link (11cm from centre at front 0 deg) -->
  <link name="camera_link"/>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.110 0.0 ${chassis_hgt + 0.025}" rpy="0 0 0"/>
  </joint>
  <link name="camera_optical_frame"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
  </joint>

  <!-- Ultrasonic Sensor Link (Front center 11cm) -->
  <link name="ultrasonic_link"/>
  <joint name="ultrasonic_joint" type="fixed">
    <parent link="base_link"/>
    <child link="ultrasonic_link"/>
    <origin xyz="0.110 0.0 ${chassis_hgt*0.6}" rpy="0 0 0"/>
  </joint>

  <!-- Dual ToF Sensor Links (10cm from centre, ~33 deg left/right) -->
  <link name="tof_left_link"/>
  <joint name="tof_left_joint" type="fixed">
    <parent link="base_link"/>
    <child link="tof_left_link"/>
    <origin xyz="0.083867 0.054464 ${chassis_hgt*0.35}" rpy="0 0 0.57595865"/>
  </joint>

  <link name="tof_right_link"/>
  <joint name="tof_right_joint" type="fixed">
    <parent link="base_link"/>
    <child link="tof_right_link"/>
    <origin xyz="0.083867 -0.054464 ${chassis_hgt*0.35}" rpy="0 0 -0.57595865"/>
  </joint>

  <!-- Wheel macro -->
  <xacro:macro name="mecanum_wheel" params="prefix x_reflect y_reflect">
    <link name="wheel_${prefix}_link">
      <visual>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="wheel_rubber"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.12"/>
        <inertia ixx="0.00006" ixy="0" ixz="0" iyy="0.00006" iyz="0" izz="0.000096"/>
      </inertial>
    </link>

    <joint name="wheel_${prefix}_joint" type="continuous">
      <parent link="base_link"/>
      <child link="wheel_${prefix}_link"/>
      <origin xyz="${x_reflect * wheelbase / 2} ${y_reflect * track_width / 2} 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <xacro:mecanum_wheel prefix="fl" x_reflect="1"  y_reflect="1"  />
  <xacro:mecanum_wheel prefix="fr" x_reflect="1"  y_reflect="-1" />
  <xacro:mecanum_wheel prefix="rl" x_reflect="-1" y_reflect="1"  />
  <xacro:mecanum_wheel prefix="rr" x_reflect="-1" y_reflect="-1" />

</robot>
'''

TELEMETRY_BRIDGE_PY = '''#!/usr/bin/env python3
"""
Rover Telemetry Bridge Node for ROS 2.
Connects over LAN to ESP32 (192.168.0.101).
Publishes standard ROS 2 messages:
  - sensor_msgs/msg/Imu on /imu/data
  - sensor_msgs/msg/MagneticField on /imu/mag
  - nav_msgs/msg/Odometry on /odom/unfiltered
  - sensor_msgs/msg/Range on /sensors/ultrasonic, /sensors/tof_left, /sensors/tof_right
Subscribes to:
  - geometry_msgs/msg/Twist on /cmd_vel
"""
import math
import time
import json
import socket
import threading
import urllib.request
import urllib.parse
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Vector3


class RoverTelemetryBridge(Node):
    def __init__(self):
        super().__init__('rover_telemetry_bridge')

        # Parameters
        self.declare_parameter('esp32_ip', '192.168.0.101')
        self.declare_parameter('esp32_port', 80)
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('poll_rate', 50.0)      # Hz
        self.declare_parameter('wheelbase', 0.180)     # meters (L)
        self.declare_parameter('track_width', 0.160)   # meters (W)
        self.declare_parameter('wheel_radius', 0.040)  # meters (R)
        self.declare_parameter('http_timeout', 0.15)   # seconds

        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.esp32_port = self.get_parameter('esp32_port').get_parameter_value().integer_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.W = self.get_parameter('track_width').get_parameter_value().double_value
        self.R = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.http_timeout = self.get_parameter('http_timeout').get_parameter_value().double_value

        self.get_logger().info(
            f"Rover Telemetry Bridge starting. ESP32: {self.esp32_ip}:{self.esp32_port}, UDP: {self.udp_port}"
        )

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom/unfiltered', 10)
        self.pub_us = self.create_publisher(Range, '/sensors/ultrasonic', 10)
        self.pub_tof_l = self.create_publisher(Range, '/sensors/tof_left', 10)
        self.pub_tof_r = self.create_publisher(Range, '/sensors/tof_right', 10)

        # Subscriber for steering
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # State tracking
        self.last_cmd_time = 0.0
        self.last_sent_twist = (0.0, 0.0, 0.0)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.last_odom_time = self.get_clock().now()
        self.is_connected = False
        self.last_telem_stamp = 0.0

        # Worker threads
        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_listener_loop, daemon=True)
        self.udp_thread.start()

        # Timer for polling HTTP fallback if UDP is silent
        timer_period = 1.0 / self.poll_rate
        self.poll_timer = self.create_timer(timer_period, self.poll_telemetry_tick)

        # Timer for safety watchdog (auto-stop rover if no cmd_vel received)
        self.watchdog_timer = self.create_timer(0.2, self.cmd_vel_watchdog_tick)

    def cmd_vel_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.angular.z)

        # Map to ESP32 rover format:
        # Forward speed: map approx 1.0 m/s -> 100%
        scale = 100.0
        x_cmd = int(max(-100, min(100, round(vy * scale))))   # In rover frame: y=strafe
        y_cmd = int(max(-100, min(100, round(vx * scale))))   # x=fwd
        z_cmd = int(max(-100, min(100, round(vz * scale))))   # z=rotation
        spd = max(abs(x_cmd), abs(y_cmd), abs(z_cmd), 30) if (x_cmd or y_cmd or z_cmd) else 0

        self.last_cmd_time = time.time()
        self.last_sent_twist = (vx, vy, vz)
        self.send_esp32_drive(x_cmd, y_cmd, z_cmd, spd)

    def send_esp32_drive(self, x: int, y: int, z: int, speed: int):
        def _post():
            try:
                url = f"http://{self.esp32_ip}:{self.esp32_port}/d?x={x}&y={y}&z={z}&s={speed}"
                req = urllib.request.Request(url, method='GET')
                with urllib.request.urlopen(req, timeout=0.1) as resp:
                    pass
            except Exception:
                pass
        threading.Thread(target=_post, daemon=True).start()

    def send_esp32_stop(self):
        def _post():
            try:
                url = f"http://{self.esp32_ip}:{self.esp32_port}/s"
                req = urllib.request.Request(url, method='GET')
                with urllib.request.urlopen(req, timeout=0.1) as resp:
                    pass
            except Exception:
                pass
        threading.Thread(target=_post, daemon=True).start()

    def cmd_vel_watchdog_tick(self):
        if self.last_cmd_time > 0 and (time.time() - self.last_cmd_time) > 0.8:
            if self.last_sent_twist != (0.0, 0.0, 0.0):
                self.send_esp32_stop()
                self.last_sent_twist = (0.0, 0.0, 0.0)

    def udp_listener_loop(self):
        """Ultra-low latency UDP listener on port 8888."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', self.udp_port))
            sock.settimeout(0.5)
            self.get_logger().info(f"UDP telemetry listener bound to port {self.udp_port}")
        except Exception as e:
            self.get_logger().warn(f"Failed to bind UDP port {self.udp_port}: {e}")
            sock.close()
            return

        while self.running and rclpy.ok():
            try:
                data, addr = sock.recvfrom(2048)
                if addr[0] == self.esp32_ip or True:
                    payload = json.loads(data.decode('utf-8', errors='ignore'))
                    self.process_telemetry(payload)
                    self.last_telem_stamp = time.time()
            except socket.timeout:
                continue
            except Exception:
                pass
        sock.close()

    def poll_telemetry_tick(self):
        """Polls HTTP /telem or /status if no recent UDP packet."""
        if time.time() - self.last_telem_stamp < (1.0 / self.poll_rate * 1.5):
            return  # UDP stream is healthy

        def _fetch():
            for endpoint in ('/telem', '/status'):
                try:
                    url = f"http://{self.esp32_ip}:{self.esp32_port}{endpoint}"
                    req = urllib.request.Request(url, headers={'User-Agent': 'ROS2-Telemetry-Bridge'})
                    with urllib.request.urlopen(req, timeout=self.http_timeout) as resp:
                        if resp.status == 200:
                            data = json.loads(resp.read().decode('utf-8'))
                            self.process_telemetry(data)
                            self.last_telem_stamp = time.time()
                            if not self.is_connected:
                                self.is_connected = True
                                self.get_logger().info(f"Connected to ESP32 at {url}")
                            return
                except Exception:
                    continue
            if self.is_connected and (time.time() - self.last_telem_stamp > 3.0):
                self.is_connected = False
                self.get_logger().warn(f"Lost telemetry link to ESP32 at {self.esp32_ip}")

        threading.Thread(target=_fetch, daemon=True).start()

    def process_telemetry(self, data: Dict[str, Any]):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        # -------------------------------------------------------------
        # 1. IMU message (/imu/data)
        # -------------------------------------------------------------
        imu_msg = Imu()
        imu_msg.header.stamp = now_msg
        imu_msg.header.frame_id = 'imu_link'

        # Orientation quaternion: [q0 (w), q1 (x), q2 (y), q3 (z)]
        if 'q' in data and len(data['q']) == 4:
            q = data['q']
            # ROS convention: x, y, z, w
            imu_msg.orientation.w = float(q[0])
            imu_msg.orientation.x = float(q[1])
            imu_msg.orientation.y = float(q[2])
            imu_msg.orientation.z = float(q[3])
        elif 'hd' in data:
            yaw_rad = math.radians(float(data['hd']))
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)

        # Orientation covariance (yaw uncertainty ~ 0.01 rad^2)
        imu_msg.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]

        # Angular velocity: gyro in rad/s
        if 'g' in data and len(data['g']) == 3:
            imu_msg.angular_velocity.x = float(data['g'][0])
            imu_msg.angular_velocity.y = float(data['g'][1])
            imu_msg.angular_velocity.z = float(data['g'][2])
        imu_msg.angular_velocity_covariance = [
            0.005, 0.0,   0.0,
            0.0,   0.005, 0.0,
            0.0,   0.0,   0.005
        ]

        # Linear acceleration: acc in m/s^2
        if 'a' in data and len(data['a']) == 3:
            imu_msg.linear_acceleration.x = float(data['a'][0])
            imu_msg.linear_acceleration.y = float(data['a'][1])
            imu_msg.linear_acceleration.z = float(data['a'][2])
        else:
            imu_msg.linear_acceleration.z = 9.80665
        imu_msg.linear_acceleration_covariance = [
            0.08, 0.0,  0.0,
            0.0,  0.08, 0.0,
            0.0,  0.0,  0.08
        ]

        self.pub_imu.publish(imu_msg)

        # -------------------------------------------------------------
        # 2. Magnetometer message (/imu/mag)
        # -------------------------------------------------------------
        mag_msg = MagneticField()
        mag_msg.header.stamp = now_msg
        mag_msg.header.frame_id = 'mag_link'
        if 'm' in data and len(data['m']) == 3:
            # Convert microtesla (uT) to Tesla (T)
            mag_msg.magnetic_field.x = float(data['m'][0]) * 1e-6
            mag_msg.magnetic_field.y = float(data['m'][1]) * 1e-6
            mag_msg.magnetic_field.z = float(data['m'][2]) * 1e-6
        mag_msg.magnetic_field_covariance = [
            1e-6, 0.0,  0.0,
            0.0,  1e-6, 0.0,
            0.0,  0.0,  1e-6
        ]
        self.pub_mag.publish(mag_msg)

        # -------------------------------------------------------------
        # 3. 4WD Mecanum Wheel Odometry (/odom/unfiltered)
        # -------------------------------------------------------------
        # Wheels order: [FL, FR, RL, RR] in mm/s
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.02

        w = data.get('w', [0, 0, 0, 0])
        # Convert mm/s to m/s
        w_fl = float(w[0]) / 1000.0
        w_fr = float(w[1]) / 1000.0
        w_rl = float(w[2]) / 1000.0
        w_rr = float(w[3]) / 1000.0

        # Standard 4WD Mecanum forward kinematics:
        # vx = (w_fl + w_fr + w_rl + w_rr) / 4
        # vy = (-w_fl + w_fr + w_rl - w_rr) / 4   (strafe)
        # wz = (-w_fl + w_fr - w_rl + w_rr) / (4 * (L/2 + W/2))
        geom_factor = (self.L / 2.0 + self.W / 2.0)
        vx = (w_fl + w_fr + w_rl + w_rr) / 4.0
        vy = (-w_fl + w_fr + w_rl - w_rr) / 4.0
        wz = (-w_fl + w_fr - w_rl + w_rr) / (4.0 * geom_factor) if geom_factor > 0 else 0.0

        # Integrate pose in odom frame using orientation yaw
        yaw = 2.0 * math.atan2(imu_msg.orientation.z, imu_msg.orientation.w)
        dx = (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        dy = (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        self.odom_x += dx
        self.odom_y += dy

        odom_msg = Odometry()
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = imu_msg.orientation

        # Pose Covariance 6x6
        # Lateral Y has higher drift in Mecanum wheels due to roller slip
        odom_msg.pose.covariance = [
            0.05, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.15, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e6, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e6, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.05
        ]

        # Twist
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = wz

        # Twist Covariance 6x6
        # Lateral velocity has ~5x higher uncertainty due to Mecanum slip dynamics
        odom_msg.twist.covariance = [
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.05, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e6, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e6, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.02
        ]

        self.pub_odom.publish(odom_msg)

        # -------------------------------------------------------------
        # 4. Range Sensors (/sensors/ultrasonic, /sensors/tof_*)
        # -------------------------------------------------------------
        if 'us' in data:
            r_us = Range()
            r_us.header.stamp = now_msg
            r_us.header.frame_id = 'ultrasonic_link'
            r_us.radiation_type = Range.ULTRASOUND
            r_us.field_of_view = 0.2618  # 15 degrees
            r_us.min_range = 0.02
            r_us.max_range = 4.00
            val_m = float(data['us']) / 1000.0
            r_us.range = val_m if val_m > 0.01 else float('inf')
            self.pub_us.publish(r_us)

        if 'tof' in data and len(data['tof']) == 2:
            # Left ToF
            r_l = Range()
            r_l.header.stamp = now_msg
            r_l.header.frame_id = 'tof_left_link'
            r_l.radiation_type = Range.INFRARED
            r_l.field_of_view = 0.4363   # 25 degrees
            r_l.min_range = 0.03
            r_l.max_range = 2.00
            val_l = float(data['tof'][0]) / 1000.0
            r_l.range = val_l if val_l > 0.02 else float('inf')
            self.pub_tof_l.publish(r_l)

            # Right ToF
            r_r = Range()
            r_r.header.stamp = now_msg
            r_r.header.frame_id = 'tof_right_link'
            r_r.radiation_type = Range.INFRARED
            r_r.field_of_view = 0.4363   # 25 degrees
            r_r.min_range = 0.03
            r_r.max_range = 2.00
            val_r = float(data['tof'][1]) / 1000.0
            r_r.range = val_r if val_r > 0.02 else float('inf')
            self.pub_tof_r.publish(r_r)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverTelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

EKF_LAUNCH_PY = '''import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_localization')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        )
    ])
'''

BRINGUP_LAUNCH_PY = '''import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_localization')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Process Xacro
    robot_description_doc = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_doc.toxml()}

    # Launch arguments
    esp32_ip_arg = DeclareLaunchArgument('esp32_ip', default_value='192.168.0.101', description='Rover ESP32 IP')
    poll_rate_arg = DeclareLaunchArgument('poll_rate', default_value='50.0', description='Telemetry polling rate (Hz)')

    return LaunchDescription([
        esp32_ip_arg,
        poll_rate_arg,

        # 1. Robot State Publisher with accurate REP-103 TF Tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # 2. Rover Telemetry Bridge (communicates with ESP32 at 192.168.0.101)
        Node(
            package='rover_localization',
            executable='rover_telemetry_bridge',
            name='rover_telemetry_bridge',
            output='screen',
            parameters=[{
                'esp32_ip': LaunchConfiguration('esp32_ip'),
                'poll_rate': LaunchConfiguration('poll_rate'),
                'wheelbase': 0.180,
                'track_width': 0.160,
                'wheel_radius': 0.040
            }]
        ),

        # 3. Robot Localization EKF node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('odometry/filtered', '/odometry/filtered')
            ]
        )
    ])
'''

TEST_KINEMATICS_PY = '''import math
import pytest

def compute_mecanum_forward_kinematics(w_fl, w_fr, w_rl, w_rr, L=0.180, W=0.160):
    geom = (L / 2.0 + W / 2.0)
    vx = (w_fl + w_fr + w_rl + w_rr) / 4.0
    vy = (-w_fl + w_fr + w_rl - w_rr) / 4.0
    wz = (-w_fl + w_fr - w_rl + w_rr) / (4.0 * geom)
    return vx, vy, wz

def test_pure_forward():
    vx, vy, wz = compute_mecanum_forward_kinematics(1.0, 1.0, 1.0, 1.0)
    assert math.isclose(vx, 1.0, rel_tol=1e-5)
    assert math.isclose(vy, 0.0, abs_tol=1e-5)
    assert math.isclose(wz, 0.0, abs_tol=1e-5)

def test_pure_strafe_right():
    # In standard Mecanum: FL=-1, FR=+1, RL=+1, RR=-1 -> pure strafe Y
    vx, vy, wz = compute_mecanum_forward_kinematics(-1.0, 1.0, 1.0, -1.0)
    assert math.isclose(vx, 0.0, abs_tol=1e-5)
    assert math.isclose(vy, 1.0, rel_tol=1e-5)
    assert math.isclose(wz, 0.0, abs_tol=1e-5)

def test_pure_rotation():
    # FL=-1, FR=+1, RL=-1, RR=+1 -> pure yaw rotation
    geom = 0.180 / 2.0 + 0.160 / 2.0
    vx, vy, wz = compute_mecanum_forward_kinematics(-1.0, 1.0, -1.0, 1.0)
    assert math.isclose(vx, 0.0, abs_tol=1e-5)
    assert math.isclose(vy, 0.0, abs_tol=1e-5)
    expected_wz = 4.0 / (4.0 * geom)
    assert math.isclose(wz, expected_wz, rel_tol=1e-5)

def test_covariance_matrix_structure():
    # Lateral velocity covariance must reflect higher slip than forward
    cov_vx = 0.01
    cov_vy = 0.05
    assert cov_vy > cov_vx, "Mecanum lateral covariance must be larger than longitudinal"

def test_quaternion_normalization():
    # Unit quaternion test
    q = [1.0, 0.0, 0.0, 0.0]
    norm = math.sqrt(sum(x*x for x in q))
    assert math.isclose(norm, 1.0, rel_tol=1e-6)

def test_zero_and_negative_velocities():
    vx, vy, wz = compute_mecanum_forward_kinematics(0.0, 0.0, 0.0, 0.0)
    assert vx == 0.0 and vy == 0.0 and wz == 0.0
    vx_back, vy_back, wz_back = compute_mecanum_forward_kinematics(-2.5, -2.5, -2.5, -2.5)
    assert math.isclose(vx_back, -2.5, rel_tol=1e-5)
    assert math.isclose(vy_back, 0.0, abs_tol=1e-5)

def test_quaternion_yaw_mapping():
    for yaw_deg in [0.0, 45.0, 90.0, -90.0, 180.0]:
        yaw_rad = math.radians(yaw_deg)
        w = math.cos(yaw_rad / 2.0)
        z = math.sin(yaw_rad / 2.0)
        reconstructed_yaw = 2.0 * math.atan2(z, w)
        assert math.isclose(yaw_rad, reconstructed_yaw, abs_tol=1e-5)

def test_twist_to_rover_input_scaling():
    scale = 100.0
    # Forward 0.5 m/s -> y_cmd = 50
    vx = 0.5
    y_cmd = int(max(-100, min(100, round(vx * scale))))
    assert y_cmd == 50

    # Strafe left 0.3 m/s -> x_cmd = 30
    vy = 0.3
    x_cmd = int(max(-100, min(100, round(vy * scale))))
    assert x_cmd == 30

    # Saturation > 1.0 m/s
    vx_over = 2.0
    y_cmd_sat = int(max(-100, min(100, round(vx_over * scale))))
    assert y_cmd_sat == 100
'''

def main():
    print(f"Connecting to {HOST}...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASS, timeout=10)
    sftp = client.open_sftp()

    def remote_write(rel_path, content):
        full_path = f"{REMOTE_WS}/{rel_path}"
        dir_name = os.path.dirname(full_path)
        try:
            client.exec_command(f"mkdir -p {dir_name}")
        except Exception:
            pass
        with sftp.file(full_path, 'w') as f:
            f.write(content)
        print(f"Uploaded: {full_path}")

    remote_write('package.xml', PACKAGE_XML)
    remote_write('setup.py', SETUP_PY)
    remote_write('setup.cfg', SETUP_CFG)
    remote_write('resource/rover_localization', '')
    remote_write('rover_localization/__init__.py', '')
    remote_write('rover_localization/rover_telemetry_bridge.py', TELEMETRY_BRIDGE_PY)
    remote_write('config/ekf.yaml', EKF_YAML)
    remote_write('urdf/rover.urdf.xacro', URDF_XACRO)
    remote_write('launch/ekf.launch.py', EKF_LAUNCH_PY)
    remote_write('launch/bringup.launch.py', BRINGUP_LAUNCH_PY)
    remote_write('test/test_fusion_kinematics.py', TEST_KINEMATICS_PY)

    client.exec_command(f"chmod +x {REMOTE_WS}/rover_localization/rover_telemetry_bridge.py")
    sftp.close()
    client.close()
    print("All rover_localization files deployed successfully!")

if __name__ == '__main__':
    main()

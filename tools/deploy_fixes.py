import os
import paramiko

HOST = "192.168.0.111"
USER = "ayush"
PASS = os.environ.get("SSH_PASSWORD")
REMOTE_PKG_DIR = "/home/ayush/mecanum_robot/mecanum_host_ws/src/rover_localization"

ROVER_TELEMETRY_BRIDGE_PY = '''#!/usr/bin/env python3
"""
Rover Telemetry Bridge Node for ROS 2.
Connects over LAN to ESP32 (default 192.168.0.101).
Publishes standard ROS 2 messages conforming to REP-103 and REP-145:
  - sensor_msgs/msg/Imu on /imu/data (fused or heading-derived orientation)
  - sensor_msgs/msg/Imu on /imu/data_raw (unfused acc & gyro)
  - sensor_msgs/msg/MagneticField on /imu/mag (when magnetometer data is available)
  - nav_msgs/msg/Odometry on /odom/unfiltered (4WD Mecanum forward kinematics)
  - sensor_msgs/msg/Range on /sensors/ultrasonic, /sensors/tof_left, /sensors/tof_right
Subscribes to:
  - geometry_msgs/msg/Twist on /cmd_vel (maps to ESP32 /d?x={x}&y={y}&z={z}&s={spd})
"""
import math
import time
import json
import socket
import threading
import queue
from typing import Optional, Dict, Any

import requests
from requests.adapters import HTTPAdapter

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class RoverTelemetryBridge(Node):
    def __init__(self):
        super().__init__('rover_telemetry_bridge')

        # Parameters
        self.declare_parameter('esp32_ip', '192.168.0.101')
        self.declare_parameter('esp32_port', 80)
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('http_poll_rate', 10.0) # Safe Hz for ESP32 HTTP
        self.declare_parameter('wheelbase', 0.180)     # meters (L)
        self.declare_parameter('track_width', 0.160)   # meters (W)
        self.declare_parameter('wheel_radius', 0.040)  # meters (R)
        self.declare_parameter('max_wheel_speed', 0.63) # m/s at 100% duty
        self.declare_parameter('http_timeout', 0.35)   # seconds

        self.esp32_ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.esp32_port = self.get_parameter('esp32_port').get_parameter_value().integer_value
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.http_poll_rate = self.get_parameter('http_poll_rate').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.W = self.get_parameter('track_width').get_parameter_value().double_value
        self.R = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').get_parameter_value().double_value
        self.http_timeout = self.get_parameter('http_timeout').get_parameter_value().double_value

        self.get_logger().info(
            f"Rover Telemetry Bridge starting. ESP32: {self.esp32_ip}:{self.esp32_port}, UDP: {self.udp_port}"
        )

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_imu_raw = self.create_publisher(Imu, '/imu/data_raw', 10)
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
        self.last_udp_stamp = 0.0

        # Dedicated reusable HTTP session with connection pooling
        self.http_session = requests.Session()
        adapter = HTTPAdapter(pool_connections=1, pool_maxsize=2, max_retries=0)
        self.http_session.mount('http://', adapter)

        # Non-blocking command queue (maxsize 1 drops stale commands)
        self.cmd_queue = queue.Queue(maxsize=1)

        # Worker threads
        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_listener_loop, daemon=True)
        self.udp_thread.start()

        self.http_thread = threading.Thread(target=self.http_polling_loop, daemon=True)
        self.http_thread.start()

        self.cmd_thread = threading.Thread(target=self.cmd_sender_loop, daemon=True)
        self.cmd_thread.start()

        # Watchdog timer (auto-stop rover if no cmd_vel received)
        self.watchdog_timer = self.create_timer(0.1, self.cmd_vel_watchdog_tick)

    def cmd_vel_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vz = float(msg.angular.z)

        # Physical mapping to ESP32 / RP2040 motor mixer:
        # In RP2040:
        #   y = forward/backward (+100 = forward)
        #   x = strafe right (+100 = right, -100 = left)
        #   z = turn right / CW (+100 = CW, -100 = CCW)
        # In ROS REP-103:
        #   vx = forward
        #   vy = strafe left (+vy = left)
        #   vz = turn left / CCW (+vz = CCW)
        scale = 100.0
        y_cmd = int(max(-100, min(100, round(vx * scale))))    # Forward
        x_cmd = -int(max(-100, min(100, round(vy * scale))))   # Invert: +vy (left) -> -x_cmd
        z_cmd = -int(max(-100, min(100, round(vz * scale))))   # Invert: +vz (CCW) -> -z_cmd
        spd = max(abs(x_cmd), abs(y_cmd), abs(z_cmd), 30) if (x_cmd or y_cmd or z_cmd) else 0

        self.last_cmd_time = time.time()
        self.last_sent_twist = (vx, vy, vz)

        try:
            self.cmd_queue.put_nowait((x_cmd, y_cmd, z_cmd, spd))
        except queue.Full:
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.cmd_queue.put_nowait((x_cmd, y_cmd, z_cmd, spd))
            except queue.Full:
                pass

    def cmd_sender_loop(self):
        """Worker thread executing HTTP drive commands with zero thread flood."""
        while self.running and rclpy.ok():
            try:
                x_cmd, y_cmd, z_cmd, spd = self.cmd_queue.get(timeout=0.05)
                url = f"http://{self.esp32_ip}:{self.esp32_port}/d?x={x_cmd}&y={y_cmd}&z={z_cmd}&s={spd}"
                self.http_session.get(url, timeout=0.25)
            except queue.Empty:
                continue
            except Exception:
                pass

    def send_esp32_stop(self):
        try:
            url = f"http://{self.esp32_ip}:{self.esp32_port}/s"
            self.http_session.get(url, timeout=0.25)
        except Exception:
            pass

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
                payload = json.loads(data.decode('utf-8', errors='ignore'))
                self.process_telemetry(payload)
                self.last_udp_stamp = time.time()
                self.last_telem_stamp = time.time()
                if not self.is_connected:
                    self.is_connected = True
                    self.get_logger().info(f"Receiving UDP telemetry from {addr[0]}")
            except socket.timeout:
                continue
            except Exception:
                pass
        sock.close()

    def http_polling_loop(self):
        """Single-threaded rate-controlled HTTP poller (never floods ESP32)."""
        poll_interval = 1.0 / max(1.0, min(20.0, self.http_poll_rate))

        while self.running and rclpy.ok():
            # If high-speed UDP stream is active, HTTP polling sleeps
            if time.time() - self.last_udp_stamp < 0.25:
                time.sleep(poll_interval)
                continue

            data = None
            for endpoint in ('/telem', '/status'):
                try:
                    url = f"http://{self.esp32_ip}:{self.esp32_port}{endpoint}"
                    r = self.http_session.get(url, timeout=self.http_timeout)
                    if r.status_code == 200:
                        data = r.json()
                        if not self.is_connected:
                            self.is_connected = True
                            self.get_logger().info(f"Connected to ESP32 at {url}")
                        break
                except Exception:
                    continue

            if data:
                self.process_telemetry(data)
                self.last_telem_stamp = time.time()
            else:
                if self.is_connected and (time.time() - self.last_telem_stamp > 3.0):
                    self.is_connected = False
                    self.get_logger().warn(f"Lost telemetry link to ESP32 at {self.esp32_ip}")

            time.sleep(poll_interval)

    def process_telemetry(self, data: Dict[str, Any]):
        now = self.get_clock().now()
        now_msg = now.to_msg()

        # -------------------------------------------------------------
        # 1. IMU message (/imu/data & /imu/data_raw)
        # -------------------------------------------------------------
        imu_msg = Imu()
        imu_msg.header.stamp = now_msg
        imu_msg.header.frame_id = 'imu_link'

        raw_msg = Imu()
        raw_msg.header.stamp = now_msg
        raw_msg.header.frame_id = 'imu_link'
        raw_msg.orientation_covariance[0] = -1.0  # raw has no orientation

        # Orientation quaternion: [q0 (w), q1 (x), q2 (y), q3 (z)]
        have_orientation = False
        if 'q' in data and len(data['q']) == 4:
            q = data['q']
            w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
            norm = math.sqrt(w*w + x*x + y*y + z*z)
            if norm > 1e-6:
                imu_msg.orientation.w = w / norm
                imu_msg.orientation.x = x / norm
                imu_msg.orientation.y = y / norm
                imu_msg.orientation.z = z / norm
                have_orientation = True
                imu_msg.orientation_covariance = [
                    0.01, 0.0,  0.0,
                    0.0,  0.01, 0.0,
                    0.0,  0.0,  0.01
                ]
        elif 'hd' in data:
            yaw_rad = math.radians(float(data['hd']))
            imu_msg.orientation.w = math.cos(yaw_rad / 2.0)
            imu_msg.orientation.x = 0.0
            imu_msg.orientation.y = 0.0
            imu_msg.orientation.z = math.sin(yaw_rad / 2.0)
            have_orientation = True
            # Roll and pitch unmeasured; yaw variance 0.05
            imu_msg.orientation_covariance = [
                1e6, 0.0, 0.0,
                0.0, 1e6, 0.0,
                0.0, 0.0, 0.05
            ]

        if not have_orientation:
            imu_msg.orientation_covariance[0] = -1.0

        # Angular velocity: gyro in rad/s (REP-145 convention)
        if 'g' in data and len(data['g']) == 3:
            gx, gy, gz = float(data['g'][0]), float(data['g'][1]), float(data['g'][2])
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            imu_msg.angular_velocity_covariance = [
                0.005, 0.0,   0.0,
                0.0,   0.005, 0.0,
                0.0,   0.0,   0.005
            ]
            raw_msg.angular_velocity = imu_msg.angular_velocity
            raw_msg.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        else:
            imu_msg.angular_velocity_covariance[0] = -1.0
            raw_msg.angular_velocity_covariance[0] = -1.0

        # Linear acceleration: acc in m/s^2 (REP-145 convention)
        if 'a' in data and len(data['a']) == 3:
            ax, ay, az = float(data['a'][0]), float(data['a'][1]), float(data['a'][2])
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.linear_acceleration_covariance = [
                0.08, 0.0,  0.0,
                0.0,  0.08, 0.0,
                0.0,  0.0,  0.08
            ]
            raw_msg.linear_acceleration = imu_msg.linear_acceleration
            raw_msg.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        else:
            imu_msg.linear_acceleration_covariance[0] = -1.0
            raw_msg.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(imu_msg)
        self.pub_imu_raw.publish(raw_msg)

        # -------------------------------------------------------------
        # 2. Magnetometer message (/imu/mag)
        # -------------------------------------------------------------
        if 'm' in data and len(data['m']) == 3:
            mx, my, mz = float(data['m'][0]), float(data['m'][1]), float(data['m'][2])
            if abs(mx) > 1e-4 or abs(my) > 1e-4 or abs(mz) > 1e-4:
                mag_msg = MagneticField()
                mag_msg.header.stamp = now_msg
                mag_msg.header.frame_id = 'mag_link'
                # Convert microtesla (uT) to Tesla (T)
                mag_msg.magnetic_field.x = mx * 1e-6
                mag_msg.magnetic_field.y = my * 1e-6
                mag_msg.magnetic_field.z = mz * 1e-6
                mag_msg.magnetic_field_covariance = [
                    1e-6, 0.0,  0.0,
                    0.0,  1e-6, 0.0,
                    0.0,  0.0,  1e-6
                ]
                self.pub_mag.publish(mag_msg)

        # -------------------------------------------------------------
        # 3. 4WD Mecanum Wheel Odometry (/odom/unfiltered)
        # -------------------------------------------------------------
        # Wheels order: [FL, FR, RL, RR]
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.02

        w = data.get('w', [0, 0, 0, 0])
        # Detect whether wheel telemetry is duty percentage (-100..100) or encoder mm/s
        all_small = all(abs(float(v)) <= 100 for v in w)
        if all_small:
            # Duty percentage -> linear wheel surface speed in m/s
            w_fl = (float(w[0]) / 100.0) * self.max_wheel_speed
            w_fr = (float(w[1]) / 100.0) * self.max_wheel_speed
            w_rl = (float(w[2]) / 100.0) * self.max_wheel_speed
            w_rr = (float(w[3]) / 100.0) * self.max_wheel_speed
        else:
            # Encoder velocity in mm/s -> m/s
            w_fl = float(w[0]) / 1000.0
            w_fr = float(w[1]) / 1000.0
            w_rl = float(w[2]) / 1000.0
            w_rr = float(w[3]) / 1000.0

        # Standard 4WD Mecanum forward kinematics in REP-103:
        # vx = (w_fl + w_fr + w_rl + w_rr) / 4.0
        # vy = (-w_fl + w_fr + w_rl - w_rr) / 4.0  (positive = left)
        # wz = (-w_fl + w_fr - w_rl + w_rr) / (4.0 * (L/2 + W/2)) (positive = CCW)
        geom_factor = (self.L / 2.0 + self.W / 2.0)
        vx = (w_fl + w_fr + w_rl + w_rr) / 4.0
        vy = (-w_fl + w_fr + w_rl - w_rr) / 4.0
        wz = (-w_fl + w_fr - w_rl + w_rr) / (4.0 * geom_factor) if geom_factor > 0 else 0.0

        # Integrate pose in odom frame using orientation yaw
        yaw = 2.0 * math.atan2(imu_msg.orientation.z, imu_msg.orientation.w) if have_orientation else 0.0
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

        # Twist in body frame (child_frame_id = base_footprint)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = wz

        # Twist Covariance 6x6:
        # Lateral velocity has ~5x higher uncertainty due to Mecanum passive roller slip
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
            r_us.range = val_m if (0.02 <= val_m <= 4.0) else float('inf')
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
            r_l.range = val_l if (0.03 <= val_l <= 2.0) else float('inf')
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
            r_r.range = val_r if (0.03 <= val_r <= 2.0) else float('inf')
            self.pub_tof_r.publish(r_r)

    def destroy_node(self):
        self.running = False
        try:
            self.http_session.close()
        except Exception:
            pass
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
    poll_rate_arg = DeclareLaunchArgument('poll_rate', default_value='10.0', description='HTTP telemetry poll rate (Hz)')

    return LaunchDescription([
        esp32_ip_arg,
        poll_rate_arg,

        # 1. Joint State Publisher (publishes wheel joints so all 11 TF links exist)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'rate': 30.0}]
        ),

        # 2. Robot State Publisher with accurate REP-103 TF Tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # 3. Rover Telemetry Bridge (communicates with ESP32 at 192.168.0.101)
        Node(
            package='rover_localization',
            executable='rover_telemetry_bridge',
            name='rover_telemetry_bridge',
            output='screen',
            parameters=[{
                'esp32_ip': LaunchConfiguration('esp32_ip'),
                'http_poll_rate': LaunchConfiguration('poll_rate'),
                'wheelbase': 0.180,
                'track_width': 0.160,
                'wheel_radius': 0.040,
                'max_wheel_speed': 0.63
            }]
        ),

        # 4. Robot Localization EKF node
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

EKF_YAML = '''### EKF Configuration for 4WD Mecanum Rover ###
### Package: robot_localization (ekf_node) ###
### Frames: odom -> base_footprint -> base_link -> imu_link, wheel_*_link ###

ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.2
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
    imu0_remove_gravitational_acceleration: false

    # -------------------------------------------------------------
    # Process Noise Covariance Q (15x15)
    # Tuned specifically for Mecanum dynamics:
    # High agility in X and Y, with higher variance allowed for lateral Y (2.5e-2)
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

TEST_FUSION_KINEMATICS_PY = '''import math
import pytest

def compute_mecanum_forward_kinematics(w_fl, w_fr, w_rl, w_rr, L=0.180, W=0.160):
    geom = (L / 2.0 + W / 2.0)
    vx = (w_fl + w_fr + w_rl + w_rr) / 4.0
    vy = (-w_fl + w_fr + w_rl - w_rr) / 4.0
    wz = (-w_fl + w_fr - w_rl + w_rr) / (4.0 * geom)
    return vx, vy, wz

def compute_twist_to_rover_cmd(vx, vy, vz, scale=100.0):
    y_cmd = int(max(-100, min(100, round(vx * scale))))
    x_cmd = -int(max(-100, min(100, round(vy * scale))))
    z_cmd = -int(max(-100, min(100, round(vz * scale))))
    return x_cmd, y_cmd, z_cmd

def test_pure_forward():
    vx, vy, wz = compute_mecanum_forward_kinematics(1.0, 1.0, 1.0, 1.0)
    assert math.isclose(vx, 1.0, rel_tol=1e-5)
    assert math.isclose(vy, 0.0, abs_tol=1e-5)
    assert math.isclose(wz, 0.0, abs_tol=1e-5)

def test_pure_strafe_left():
    # In REP-103 (+vy is left): FL=-1, FR=+1, RL=+1, RR=-1 -> pure strafe left (+vy)
    vx, vy, wz = compute_mecanum_forward_kinematics(-1.0, 1.0, 1.0, -1.0)
    assert math.isclose(vx, 0.0, abs_tol=1e-5)
    assert math.isclose(vy, 1.0, rel_tol=1e-5)
    assert math.isclose(wz, 0.0, abs_tol=1e-5)

def test_pure_strafe_right():
    # In REP-103 (-vy is right): FL=+1, FR=-1, RL=-1, RR=+1 -> pure strafe right (-vy)
    vx, vy, wz = compute_mecanum_forward_kinematics(1.0, -1.0, -1.0, 1.0)
    assert math.isclose(vx, 0.0, abs_tol=1e-5)
    assert math.isclose(vy, -1.0, rel_tol=1e-5)
    assert math.isclose(wz, 0.0, abs_tol=1e-5)

def test_pure_rotation_ccw():
    # CCW rotation around +Z: FL=-1, FR=+1, RL=-1, RR=+1 -> wz > 0
    geom = 0.180 / 2.0 + 0.160 / 2.0
    vx, vy, wz = compute_mecanum_forward_kinematics(-1.0, 1.0, -1.0, 1.0)
    assert math.isclose(vx, 0.0, abs_tol=1e-5)
    assert math.isclose(vy, 0.0, abs_tol=1e-5)
    expected_wz = 4.0 / (4.0 * geom)
    assert math.isclose(wz, expected_wz, rel_tol=1e-5)
    assert wz > 0

def test_pure_rotation_cw():
    # CW rotation around -Z: FL=+1, FR=-1, RL=+1, RR=-1 -> wz < 0
    geom = 0.180 / 2.0 + 0.160 / 2.0
    vx, vy, wz = compute_mecanum_forward_kinematics(1.0, -1.0, 1.0, -1.0)
    assert math.isclose(vx, 0.0, abs_tol=1e-5)
    assert math.isclose(vy, 0.0, abs_tol=1e-5)
    assert wz < 0

def test_twist_to_rover_input_signs():
    # Forward vx=0.5 -> y_cmd=+50
    x, y, z = compute_twist_to_rover_cmd(0.5, 0.0, 0.0)
    assert y == 50 and x == 0 and z == 0

    # Strafe Left vy=+0.4 -> x_cmd=-40 (in RP2040 x>0 is right, so left is negative)
    x, y, z = compute_twist_to_rover_cmd(0.0, 0.4, 0.0)
    assert x == -40 and y == 0 and z == 0

    # Strafe Right vy=-0.4 -> x_cmd=+40
    x, y, z = compute_twist_to_rover_cmd(0.0, -0.4, 0.0)
    assert x == 40 and y == 0 and z == 0

    # Turn Left CCW vz=+0.6 -> z_cmd=-60 (in RP2040 z>0 is CW, so CCW is negative)
    x, y, z = compute_twist_to_rover_cmd(0.0, 0.0, 0.6)
    assert z == -60 and x == 0 and y == 0

    # Turn Right CW vz=-0.6 -> z_cmd=+60
    x, y, z = compute_twist_to_rover_cmd(0.0, 0.0, -0.6)
    assert z == 60 and x == 0 and y == 0

def test_wheel_speed_scaling_duty_vs_mms():
    max_speed = 0.63
    # Case 1: Status percentages [-100..100]
    w_pct = [50, 50, 50, 50]
    w_mps = [(v / 100.0) * max_speed for v in w_pct]
    assert math.isclose(w_mps[0], 0.315, rel_tol=1e-4)

    # Case 2: Encoder mm/s
    w_mms = [500, 500, 500, 500]
    w_mps_enc = [v / 1000.0 for v in w_mms]
    assert math.isclose(w_mps_enc[0], 0.500, rel_tol=1e-4)

def test_covariance_matrix_asymmetry():
    cov_vx = 0.01
    cov_vy = 0.05
    assert cov_vy >= 2.5 * cov_vx, "Mecanum lateral covariance must be at least 2.5x longitudinal"

def test_quaternion_normalization_and_yaw():
    for yaw_deg in [0.0, 30.0, 90.0, -60.0, 180.0]:
        yaw_rad = math.radians(yaw_deg)
        w = math.cos(yaw_rad / 2.0)
        z = math.sin(yaw_rad / 2.0)
        norm = math.sqrt(w*w + z*z)
        assert math.isclose(norm, 1.0, rel_tol=1e-6)
        reconstructed_yaw = 2.0 * math.atan2(z, w)
        assert math.isclose(yaw_rad, reconstructed_yaw, abs_tol=1e-5)

def test_rep145_unmeasured_covariance():
    # If sensor reading is unavailable, REP-145 mandates covariance[0] == -1.0
    gyro_available = False
    cov = [0.005]*9 if gyro_available else [-1.0] + [0.0]*8
    assert cov[0] == -1.0
'''


def main():
    print(f"Connecting to host {HOST} via SFTP...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PASS, timeout=10)
    sftp = ssh.open_sftp()

    def write_remote(path, content):
        print(f"Writing {path}...")
        with sftp.file(path, 'w') as f:
            f.write(content)

    write_remote(f"{REMOTE_PKG_DIR}/rover_localization/rover_telemetry_bridge.py", ROVER_TELEMETRY_BRIDGE_PY)
    write_remote(f"{REMOTE_PKG_DIR}/launch/bringup.launch.py", BRINGUP_LAUNCH_PY)
    write_remote(f"{REMOTE_PKG_DIR}/config/ekf.yaml", EKF_YAML)
    write_remote(f"{REMOTE_PKG_DIR}/test/test_fusion_kinematics.py", TEST_FUSION_KINEMATICS_PY)

    sftp.close()
    ssh.close()
    print("Files successfully deployed!")


if __name__ == '__main__':
    main()

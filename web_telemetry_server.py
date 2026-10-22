#!/usr/bin/env python3
"""
Mecanum Robot High-Speed Telemetry Hub & ROS 2 Bridge.
Zero-latency UDP packet receiver (<1ms) on port 8888 with ROS 2 sensor fusion & odom publishing.
"""
import os
import json
import math
import time
import socket
import struct
import asyncio
import base64
import threading
import urllib.request
from pathlib import Path
from typing import Set, Dict, Any

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Vector3, Quaternion
    from sensor_msgs.msg import Imu, MagneticField, Range
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Int64, Float32, UInt8, String, Int16MultiArray, Bool
except ImportError:
    Node = object
    rclpy = None

import sys
_current_dir = Path(__file__).resolve().parent
if str(_current_dir.parent) not in sys.path:
    sys.path.insert(0, str(_current_dir.parent))

try:
    from mecanum_robot.protocol import (
        MODE_FLAG_HOLONOMIC, MODE_NAMES
    )
except ImportError:
    MODE_FLAG_HOLONOMIC = 1
    MODE_NAMES = {0: "Normal", 1: "Holonomic", 2: "Auto", 3: "Calibration"}
try:
    import sentry_sdk
    sentry_sdk.init(
        dsn="REMOVED",
        traces_sample_rate=0.1,
        environment="production",
        release="mecanum-rover-ros2@2.1.0"
    )
    print("[SENTRY] Sentry SDK initialized for mecanum-rover-ros2@2.1.0")
except ImportError:
    print("[SENTRY] Notice: sentry-sdk not installed, running without telemetry reporting")
except Exception as e:
    print(f"[SENTRY] Notice: Sentry init exception: {e}")

app = FastAPI(title="Mecanum Robot Telemetry Hub")

AUTO_STATES = {
    0: ("CLEAR CRUISE", "Full corridor clear cruise", "#00ff88"),
    1: ("THINKING", "Scanning polar histogram & spatial memory", "#ffbb00"),
    2: ("PLANNING", "Computing optimal VFH+ open heading", "#00d4ff"),
    3: ("VEER LEFT", "Arcing left around right obstacle", "#38ef7d"),
    4: ("VEER RIGHT", "Arcing right around left obstacle", "#11998e"),
    5: ("CREEPING", "Precision narrow corridor crawl (300-480mm)", "#f39c12"),
    6: ("STUCK ALERT", "Obstacle trap or stall detected", "#e74c3c"),
    7: ("BACKTRACKING", "Reversing along breadcrumb trail", "#e67e22"),
    8: ("S-CURVE ESCAPE", "Reversing and curving out of corner trap", "#9b59b6"),
    9: ("COMPASS PIVOT", "In-place pivot toward open heading", "#3498db"),
    10: ("WALL FOLLOW", "Parallel contour along obstacle wall", "#1abc9c"),
    11: ("IMPACT REFLEX", "Emergency reverse-pivot collision reflex", "#ff0055")
}

# Store connected active WebSockets
active_connections: Set[WebSocket] = set()

# ROS 2 Sensory Fusion & Autonomy Memory
obstacle_memory = []   # list of [ox_m, oy_m, timestamp]
path_trail = []        # list of [rx_m, ry_m]
polar_hist = [0.0] * 12 # 12 sectors, 30 deg each (-180 to +180)
auto_state = "FORWARD"
auto_target_heading = 0.0
auto_escape_dir = 1
auto_state_start_t = 0.0

PI_CAM_IPS = ["192.168.0.103", "192.168.0.108"]
active_pi_cam_ip = "192.168.0.103"

# Latest cached telemetry
latest_telemetry: Dict[str, Any] = {
    "yaw": 0.0,
    "mag_heading": 0.0,
    "yaw_deg": 0.0,
    "pitch_deg": 0.0,
    "roll_deg": 0.0,
    "pos_x": 0.0,        # meters
    "pos_y": 0.0,        # meters
    "pos_x_cm": 0.0,
    "pos_y_cm": 0.0,
    "encoder_ticks": 0,
    "tick_rate": 0.0,
    "pwm_fl": 0,
    "pwm_fr": 0,
    "pwm_rl": 0,
    "pwm_rr": 0,
    "ultra_mm": 0,
    "tof_l_mm": 0,
    "tof_r_mm": 0,
    "virtual_tof_l_mm": 2000,
    "virtual_tof_r_mm": 2000,
    "virtual_front_mm": 2000,
    "doa_deg": 0.0,
    "doa_conf": 0.0,
    "radar_pres": False,
    "radar_mm": 0,
    "voice_state": "IDLE_LISTEN",
    "obstacle_dots": [],
    "path_trail": [],
    "mode": 1,
    "mode_name": "Holonomic",
    "link_type": "Initializing...",
    "quaternion": [1.0, 0.0, 0.0, 0.0],
    "q": [1.0, 0.0, 0.0, 0.0],
    "gyro": [0.0, 0.0, 0.0],
    "g": [0.0, 0.0, 0.0],
    "accel": [0.0, 0.0, 9.80665],
    "a": [0.0, 0.0, 9.80665],
    "mag": [0.0, 0.0, 0.0],
    "m": [0.0, 0.0, 0.0],
    "wheel_speeds": [0, 0, 0, 0],
    "w": [0, 0, 0, 0],
    "speed": 50,
    "pi_cam": {
        "cam_online": False,
        "stream_url": "http://192.168.0.103:8081/stream",
        "vis_f_mm": 2000,
        "vis_l_mm": 2000,
        "vis_r_mm": 2000,
        "clear_heading_deg": 0.0,
        "fps": 0.0,
        "frame_seq": 0,
        "uart_tx": 0,
        "uart_rx": 0,
        "cpu_temp_c": 0.0,
        "mem_avail_mb": 0,
        "cpu_usage": 0.0,
        "last_seen": 0.0
    },
    "auto_state_id": 0,
    "auto_state_name": "CLEAR CRUISE",
    "auto_state_desc": "Full corridor clear cruise",
    "auto_state_color": "#00ff88",
    "auto_target_heading": 0.0
}

last_udp_packet_time = 0.0
last_rover_addr = ("192.168.0.105", 8888)
rover_cmd_seq = 0
udp_cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ros_node_bridge = None

def send_rover_drive_packet(vx: float, vy: float, vz: float, speed: int = 50, mode: int = 0, flags: int = 0):
    global rover_cmd_seq, last_rover_addr
    rover_cmd_seq = (rover_cmd_seq + 1) & 0xFFFF
    # Map Web Dashboard coordinate convention to Rover kinematics:
    # vx (Forward/Backward) -> RoverPkt.y (-100 to 100)
    # vy (Strafe Left/Right) -> RoverPkt.x (-100 to 100)
    # vz (Yaw Rotation) -> RoverPkt.z (-100 to 100)
    x_val = int(max(-100, min(100, round(vy * 100.0))))
    y_val = int(max(-100, min(100, round(vx * 100.0))))
    z_val = int(max(-100, min(100, round(vz * 50.0))))
    is_stop = (abs(x_val) == 0 and abs(y_val) == 0 and abs(z_val) == 0)
    pkt_type = 4 if is_stop else 3  # PKT_STOP = 4, PKT_DRIVE = 3

    pkt = struct.pack('<BBbbbBBhH', 0xA7, pkt_type, x_val, y_val, z_val, int(speed), int(mode), int(flags), 0, rover_cmd_seq)

    targets = set()
    if last_rover_addr:
        targets.add((last_rover_addr[0], 8888))
        targets.add(last_rover_addr)
    targets.add(("192.168.0.105", 8888))
    targets.add(("192.168.0.101", 8888))

    for target in targets:
        try:
            udp_cmd_sock.sendto(pkt, target)
        except Exception:
            pass

class WebRosBridgeNode(Node if rclpy else object):
    def __init__(self):
        if rclpy:
            super().__init__('web_telemetry_server_node')
            # Teleop & control publishers
            self.pub_teleop = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
            self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
            self.pub_pan_tilt = self.create_publisher(Vector3, '/pan_tilt', 10)
            self.pub_mode = self.create_publisher(UInt8, '/robot/set_mode', 10)

            # High-speed telemetry publishers (REP-103)
            self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
            self.pub_odom_raw = self.create_publisher(Odometry, '/odom/unfiltered', 10)
            self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
            self.pub_mag = self.create_publisher(MagneticField, '/imu/mag', 10)
            self.pub_us = self.create_publisher(Range, '/sensors/ultrasonic', 10)
            self.pub_tof_l = self.create_publisher(Range, '/sensors/tof_left', 10)
            self.pub_tof_r = self.create_publisher(Range, '/sensors/tof_right', 10)

            # Wheel ticks & mode
            self.pub_ticks = self.create_publisher(Int64, '/wheel_ticks', 10)
            self.pub_tick_rate = self.create_publisher(Float32, '/tick_rate', 10)
            self.pub_heading = self.create_publisher(Float32, '/sensors/heading', 10)

            # Voice DOA & HLK-LD2402 Radar subscriptions
            try:
                self.sub_doa_az = self.create_subscription(Float32, '/audio/doa_azimuth', self._on_doa_az, 10)
                self.sub_doa_conf = self.create_subscription(Float32, '/audio/doa_confidence', self._on_doa_conf, 10)
                self.sub_radar_pres = self.create_subscription(Bool, '/radar/presence', self._on_radar_pres, 10)
                self.sub_radar_dist = self.create_subscription(Float32, '/radar/distance', self._on_radar_dist, 10)
                self.sub_voice_state = self.create_subscription(String, '/voice_follower/state', self._on_voice_state, 10)
            except Exception:
                pass

    def _on_doa_az(self, msg):
        latest_telemetry['doa_deg'] = round(float(msg.data), 1)

    def _on_doa_conf(self, msg):
        latest_telemetry['doa_conf'] = round(float(msg.data), 2)

    def _on_radar_pres(self, msg):
        latest_telemetry['radar_pres'] = bool(msg.data)

    def _on_radar_dist(self, msg):
        latest_telemetry['radar_mm'] = int(float(msg.data) * 1000.0)

    def _on_voice_state(self, msg):
        latest_telemetry['voice_state'] = str(msg.data)

    def publish_telemetry(self, t: Dict[str, Any]):
        if not rclpy or not self.pub_odom:
            return

        now_msg = self.get_clock().now().to_msg()

        # 1. Orientation & IMU
        q = t.get('q', [1.0, 0.0, 0.0, 0.0])
        imu_msg = Imu()
        imu_msg.header.stamp = now_msg
        imu_msg.header.frame_id = 'imu_link'
        if len(q) == 4:
            imu_msg.orientation.w = float(q[0])
            imu_msg.orientation.x = float(q[1])
            imu_msg.orientation.y = float(q[2])
            imu_msg.orientation.z = float(q[3])
            imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        g = t.get('g', [0.0, 0.0, 0.0])
        if len(g) == 3:
            imu_msg.angular_velocity.x = float(g[0])
            imu_msg.angular_velocity.y = float(g[1])
            imu_msg.angular_velocity.z = float(g[2])
            imu_msg.angular_velocity_covariance = [0.005, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.005]

        a = t.get('a', [0.0, 0.0, 9.8])
        if len(a) == 3:
            imu_msg.linear_acceleration.x = float(a[0])
            imu_msg.linear_acceleration.y = float(a[1])
            imu_msg.linear_acceleration.z = float(a[2])
            imu_msg.linear_acceleration_covariance = [0.08, 0.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.08]

        self.pub_imu.publish(imu_msg)

        # 2. Odometry (Positioning in real time)
        pos = t.get('pos', [0.0, 0.0])
        pos_x_m = float(pos[0]) / 100.0
        pos_y_m = float(pos[1]) / 100.0

        odom_msg = Odometry()
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = pos_x_m
        odom_msg.pose.pose.position.y = pos_y_m
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = imu_msg.orientation
        odom_msg.pose.covariance = [
            0.05, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.15, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e6, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e6, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e6, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.05
        ]

        w = t.get('w', [0, 0, 0, 0])
        if len(w) == 4:
            vx = (float(w[0]) + float(w[1]) + float(w[2]) + float(w[3])) / 4000.0
            vy = (-float(w[0]) + float(w[1]) + float(w[2]) - float(w[3])) / 4000.0
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy

        self.pub_odom.publish(odom_msg)
        self.pub_odom_raw.publish(odom_msg)

        # 3. Magnetometer
        m = t.get('m', [0.0, 0.0, 0.0])
        if len(m) == 3 and (abs(m[0]) > 0.1 or abs(m[1]) > 0.1):
            mag_msg = MagneticField()
            mag_msg.header.stamp = now_msg
            mag_msg.header.frame_id = 'mag_link'
            mag_msg.magnetic_field.x = float(m[0]) * 1e-6
            mag_msg.magnetic_field.y = float(m[1]) * 1e-6
            mag_msg.magnetic_field.z = float(m[2]) * 1e-6
            self.pub_mag.publish(mag_msg)

        # 4. Heading
        if 'hd' in t:
            heading_msg = Float32()
            heading_msg.data = float(t['hd'])
            self.pub_heading.publish(heading_msg)


def update_sensory_memory(rx_m: float, ry_m: float, hd_deg: float, us_mm: int, tof_l_mm: int, tof_r_mm: int):
    global obstacle_memory, path_trail, polar_hist
    now = time.time()

    # 1. Update Path Trail (at least 10cm spacing)
    if not path_trail:
        path_trail.append([round(rx_m, 3), round(ry_m, 3)])
    else:
        last_x, last_y = path_trail[-1]
        if math.hypot(rx_m - last_x, ry_m - last_y) >= 0.10:
            path_trail.append([round(rx_m, 3), round(ry_m, 3)])
            if len(path_trail) > 64:
                path_trail.pop(0)

    # 2. Decay polar histogram
    for i in range(12):
        polar_hist[i] *= 0.96

    # 3. Project physical + visual camera hits into world coordinates
    hd_rad = math.radians(hd_deg)
    # Sensor offsets (sx, sy in meters) and beam angles (rel_ang_deg)
    # Front ToF: 11 cm from centre at front (0 deg)
    # Rear ToF: 10 cm from centre at rear (180 deg)
    readings = [
        (tof_l_mm, 0.0, 0.110, 0.0),            # Front Center ToF (0 deg, 11cm)
        (tof_r_mm, 180.0, -0.100, 0.0)          # Rear Center ToF (180 deg, 10cm)
    ]

    pi_cam = latest_telemetry.get("pi_cam", {})
    if pi_cam.get("cam_online") and (now - pi_cam.get("last_seen", 0) < 2.5):
        c_f = pi_cam.get("vis_f_mm", 2000)
        c_l = pi_cam.get("vis_l_mm", 2000)
        c_r = pi_cam.get("vis_r_mm", 2000)
        # Camera mounted at front 0 deg, 11cm from centre (0° offset)
        if 200 <= c_l <= 1500: readings.append((c_l, 16.0, 0.110, 0.0))   # Left (+16° cam)
        if 200 <= c_f <= 1500: readings.append((c_f, 0.0, 0.110, 0.0))    # Center (0° cam)
        if 200 <= c_r <= 1500: readings.append((c_r, -16.0, 0.110, 0.0))  # Right (-16° cam)

    for d_mm, rel_ang_deg, sx, sy in readings:
        if 25 <= d_mm <= 1500:
            d_m = d_mm / 1000.0
            beam_rad = hd_rad + math.radians(rel_ang_deg)
            # Body-frame sensor offset projected into world frame
            ox = rx_m + (sx * math.cos(hd_rad) - sy * math.sin(hd_rad)) + d_m * math.cos(beam_rad)
            oy = ry_m + (sx * math.sin(hd_rad) + sy * math.cos(hd_rad)) + d_m * math.sin(beam_rad)

            found = False
            for dot in obstacle_memory:
                if math.hypot(dot[0] - ox, dot[1] - oy) < 0.08:
                    dot[0] = ox
                    dot[1] = oy
                    dot[2] = now
                    found = True
                    break
            if not found:
                obstacle_memory.append([round(ox, 3), round(oy, 3), now])

            hit_ang_deg = math.degrees(math.atan2(oy - ry_m, ox - rx_m))
            sector = int(math.floor((hit_ang_deg + 180.0) / 30.0)) % 12
            cost_inc = (1.5 - d_m) / 0.5
            polar_hist[sector] = min(10.0, polar_hist[sector] + cost_inc)

    obstacle_memory = [dot for dot in obstacle_memory if (now - dot[2]) < 1.5][-8:]

    # 4. Raycast Virtual ToF distances from spatial memory
    virt_f_m = 2.0
    virt_l_m = 2.0
    virt_r_m = 2.0

    for ox, oy, _ in obstacle_memory:
        dx = ox - rx_m
        dy = oy - ry_m
        dist = math.hypot(dx, dy)
        if dist < 0.05 or dist > 2.0:
            continue
        bearing = math.atan2(dy, dx)
        rel_rad = (bearing - hd_rad + math.pi) % (2 * math.pi) - math.pi
        rel_deg = math.degrees(rel_rad)

        if abs(rel_deg) <= 16.0 and dist < virt_f_m:
            virt_f_m = dist
        if abs(rel_deg - 32.0) <= 18.0 and dist < virt_l_m:
            virt_l_m = dist
        if abs(rel_deg - (-32.0)) <= 18.0 and dist < virt_r_m:
            virt_r_m = dist

    latest_telemetry['virtual_tof_l_mm'] = int(virt_l_m * 1000)
    latest_telemetry['virtual_tof_r_mm'] = int(virt_r_m * 1000)
    latest_telemetry['virtual_front_mm'] = int(virt_f_m * 1000)
    latest_telemetry['obstacle_dots'] = [[p[0], p[1]] for p in obstacle_memory[-48:]]
    latest_telemetry['path_trail'] = path_trail[-48:]


async def ros2_autonomy_computation_loop():
    """20Hz Autonomy computation loop offloaded from Rover Core."""
    global auto_state, auto_target_heading, auto_escape_dir, auto_state_start_t
    while True:
        try:
            curr_mode = latest_telemetry.get('mode', 1)
            if curr_mode == 9:  # MODE_AUTO
                now = time.time()
                hd_deg = latest_telemetry.get('yaw_deg', 0.0)

                # Effective sensor readings (physical with virtual fallback)
                us_mm = latest_telemetry.get('ultra_mm', 0)
                tof_l_mm = latest_telemetry.get('tof_l_mm', 0)
                tof_r_mm = latest_telemetry.get('tof_r_mm', 0)
                v_f_mm = latest_telemetry.get('virtual_front_mm', 2000)
                v_l_mm = latest_telemetry.get('virtual_tof_l_mm', 2000)
                v_r_mm = latest_telemetry.get('virtual_tof_r_mm', 2000)

                d_f = us_mm if (25 <= us_mm <= 2000) else v_f_mm
                d_l = tof_l_mm if (25 <= tof_l_mm <= 2000) else v_l_mm
                d_r = tof_r_mm if (25 <= tof_r_mm <= 2000) else v_r_mm

                pi_cam = latest_telemetry.get("pi_cam", {})
                if pi_cam.get("cam_online") and (now - pi_cam.get("last_seen", 0) < 2.5):
                    d_f = min(d_f, pi_cam.get("vis_f_mm", 2000))
                    d_l = min(d_l, pi_cam.get("vis_l_mm", 2000))
                    d_r = min(d_r, pi_cam.get("vis_r_mm", 2000))

                tof_corroborates = (d_l < 650 or d_r < 650)
                is_boxed_in = ((d_f < 250 and tof_corroborates) or
                               (d_l < 250 and d_r < 250) or
                               (d_f < 350 and tof_corroborates and (d_l < 300 or d_r < 300)))

                base_spd = int(latest_telemetry.get('speed', 50))
                if base_spd <= 0:
                    base_spd = 50
                base_spd = max(5, min(100, base_spd))
                spd = base_spd

                if auto_state == "FORWARD":
                    if is_boxed_in:
                        best_sector = 0
                        best_cost = 999999.0
                        for s in range(12):
                            s_ang = -180.0 + s * 30.0 + 15.0
                            diff = abs((s_ang - hd_deg + 180.0) % 360.0 - 180.0)
                            cost = polar_hist[s]
                            if diff < 60.0:
                                cost += 15.0
                            if cost < best_cost:
                                best_cost = cost
                                best_sector = s
                        auto_target_heading = -180.0 + best_sector * 30.0 + 15.0
                        diff_target = (auto_target_heading - hd_deg + 180.0) % 360.0 - 180.0
                        auto_escape_dir = 1 if diff_target >= 0 else -1
                        auto_state = "REVERSE_OUT"
                        auto_state_start_t = now
                    else:
                        fwd_norm = max(0.0, min(1.0, (min(d_f, min(d_l, d_r)) - 250.0) / 450.0))
                        vx = 0.80 + 0.20 * fwd_norm
                        turn_bias = (d_l - d_r) / 500.0
                        vz = max(-0.7, min(0.7, turn_bias))
                        spd = int(base_spd * (0.80 + 0.20 * fwd_norm))

                elif auto_state == "REVERSE_OUT":
                    vx = -0.65
                    vz = 0.0
                    spd = max(5, min(100, int(base_spd * 1.05)))
                    if now - auto_state_start_t > 0.75:
                        auto_state = "PIVOT_OUT"
                        auto_state_start_t = now

                elif auto_state == "PIVOT_OUT":
                    diff_h = (auto_target_heading - hd_deg + 180.0) % 360.0 - 180.0
                    if abs(diff_h) < 18.0 or (now - auto_state_start_t > 1.8):
                        auto_state = "FORWARD"
                    else:
                        vz = 0.85 * auto_escape_dir
                        spd = max(5, min(100, base_spd))

                vy = 0.0
                # Rover core handles onboard assisted driving. Do not broadcast background drive packets to physical rover.
                latest_telemetry['auto_state_name'] = auto_state

        except Exception:
            pass

        await asyncio.sleep(0.05)


def process_rover_payload(payload_str: str, link_label: str = "UDP"):
    global last_udp_packet_time
    if payload_str.startswith("R,"):
        parts = payload_str.strip().split(",")
        if len(parts) >= 6:
            try:
                hd_deg = float(parts[1])
                latest_telemetry['yaw_deg'] = round(hd_deg, 1)
                latest_telemetry['yaw'] = math.radians(hd_deg)
                latest_telemetry['mag_heading'] = math.radians(hd_deg)
                latest_telemetry['ultra_mm'] = int(parts[2])
                latest_telemetry['tof_l_mm'] = int(parts[3])
                latest_telemetry['tof_r_mm'] = int(parts[4])
                latest_telemetry['mode'] = int(parts[5])
                latest_telemetry['mode_name'] = MODE_NAMES.get(int(parts[5]), f"Mode {parts[5]}")
                if len(parts) >= 7:
                    latest_telemetry['radar_pres'] = (int(parts[6]) != 0)
                latest_telemetry['link_type'] = "Pi UART Bridge"
            except Exception:
                pass
        return

    try:
        t = json.loads(payload_str)
    except Exception:
        return

    if t.get('type') == 'pi_cam':
        cam_info = latest_telemetry.setdefault("pi_cam", {})
        cam_info.update({
            "cam_online": bool(t.get("cam_online", False)),
            "stream_url": f"http://{active_pi_cam_ip}:8081/stream",
            "vis_f_mm": int(t.get("vis_f_mm", 2000)),
            "vis_l_mm": int(t.get("vis_l_mm", 2000)),
            "vis_r_mm": int(t.get("vis_r_mm", 2000)),
            "clear_heading_deg": float(t.get("clear_heading_deg", 0.0)),
            "fps": float(t.get("fps", 0.0)),
            "frame_seq": int(t.get("frame_seq", 0)),
            "uart_tx": int(t.get("uart_tx", 0)),
            "uart_rx": int(t.get("uart_rx", 0)),
            "cpu_temp_c": float(t.get("cpu_temp_c", 0.0)),
            "mem_avail_mb": int(t.get("mem_avail_mb", 0)),
            "cpu_usage": float(t.get("cpu_usage", 0.0)),
            "last_seen": time.time()
        })
        return

    if link_label == "UDP":
        last_udp_packet_time = time.time()

    # Orientation
    hd_deg = float(t.get('hd', 0.0))
    if hd_deg < 0:
        hd_deg += 360.0
    p_deg = float(t.get('p', 0.0))
    r_deg = float(t.get('r', 0.0))

    latest_telemetry['yaw'] = hd_deg * math.pi / 180.0
    latest_telemetry['mag_heading'] = hd_deg * math.pi / 180.0
    latest_telemetry['yaw_deg'] = round(hd_deg, 1)
    latest_telemetry['pitch_deg'] = round(p_deg, 1)
    latest_telemetry['roll_deg'] = round(r_deg, 1)

    # Real-Time 2D Position & Odometry
    pos = t.get('pos', [0.0, 0.0])
    if len(pos) >= 2:
        pos_x_cm = float(pos[0])
        pos_y_cm = float(pos[1])
        latest_telemetry['pos_x_cm'] = round(pos_x_cm, 1)
        latest_telemetry['pos_y_cm'] = round(pos_y_cm, 1)
        latest_telemetry['pos_x'] = round(pos_x_cm / 100.0, 3)
        latest_telemetry['pos_y'] = round(pos_y_cm / 100.0, 3)

    # Wheel velocities & actual mm/s speeds
    w = t.get('w', [0, 0, 0, 0])
    if len(w) >= 4:
        wheel_ints = [int(w[0]), int(w[1]), int(w[2]), int(w[3])]
        latest_telemetry['w'] = wheel_ints
        latest_telemetry['wheel_speeds'] = wheel_ints
        latest_telemetry['pwm_fl'] = wheel_ints[0]
        latest_telemetry['pwm_fr'] = wheel_ints[1]
        latest_telemetry['pwm_rl'] = wheel_ints[2]
        latest_telemetry['pwm_rr'] = wheel_ints[3]

    # Distance sensors
    latest_telemetry['ultra_mm'] = int(t.get('us', 0))
    tof = t.get('tof', [0, 0])
    if len(tof) >= 2:
        latest_telemetry['tof_l_mm'] = int(tof[0])
        latest_telemetry['tof_r_mm'] = int(tof[1])

    latest_telemetry['link_type'] = f"{link_label} (Zero-Lag)"

    # Mode extraction
    if 'mode' in t:
        latest_telemetry['mode'] = int(t['mode'])
        latest_telemetry['mode_name'] = MODE_NAMES.get(int(t['mode']), f"Mode {t['mode']}")

    # Autonomous Navigation State extraction
    if 'as' in t:
        as_id = int(t['as'])
        as_info = AUTO_STATES.get(as_id, ("CLEAR CRUISE", "Full corridor clear cruise", "#00ff88"))
        latest_telemetry['auto_state_id'] = as_id
        latest_telemetry['auto_state_name'] = as_info[0]
        latest_telemetry['auto_state_desc'] = as_info[1]
        latest_telemetry['auto_state_color'] = as_info[2]
        latest_telemetry['auto_target_heading'] = float(t.get('at', 0.0))

    # Strict Onboard Rover IMU Coordinates: Gyroscope, Accelerometer gravity vector, Magnetometer, Quaternion
    if 'q' in t and len(t['q']) >= 4:
        q_floats = [float(v) for v in t['q'][:4]]
        latest_telemetry['q'] = q_floats
        latest_telemetry['quaternion'] = q_floats

    if 'g' in t and len(t['g']) >= 3:
        g_floats = [round(float(v), 4) for v in t['g'][:3]]
        latest_telemetry['g'] = g_floats
        latest_telemetry['gyro'] = g_floats

    if 'a' in t and len(t['a']) >= 3:
        a_floats = [round(float(v), 4) for v in t['a'][:3]]
        latest_telemetry['a'] = a_floats
        latest_telemetry['accel'] = a_floats

    if 'm' in t and len(t['m']) >= 3:
        m_floats = [round(float(v), 2) for v in t['m'][:3]]
        latest_telemetry['m'] = m_floats
        latest_telemetry['mag'] = m_floats

    # Voice DOA & HLK-LD2402 Radar telemetry
    if 'rp' in t or 'radar_pres' in t:
        latest_telemetry['radar_pres'] = bool(t.get('rp', t.get('radar_pres', False)))
    if 'rm' in t or 'radar_mm' in t:
        latest_telemetry['radar_mm'] = int(t.get('rm', t.get('radar_mm', 0)))
    if 'doa' in t or 'doa_deg' in t:
        latest_telemetry['doa_deg'] = round(float(t.get('doa', t.get('doa_deg', 0.0))), 1)
    if 'dc' in t or 'doa_conf' in t:
        latest_telemetry['doa_conf'] = round(float(t.get('dc', t.get('doa_conf', 0.0))), 2)
    if 'vs' in t or 'voice_state' in t:
        latest_telemetry['voice_state'] = str(t.get('vs', t.get('voice_state', 'IDLE_LISTEN')))

    # Update Sensory Fusion Memory & Virtual ToF raycasting
    update_sensory_memory(
        latest_telemetry['pos_x'],
        latest_telemetry['pos_y'],
        latest_telemetry['yaw_deg'],
        latest_telemetry['ultra_mm'],
        latest_telemetry['tof_l_mm'],
        latest_telemetry['tof_r_mm']
    )

    # Publish to ROS 2 topic bus
    if ros_node_bridge:
        ros_node_bridge.publish_telemetry(t)


def udp_listener_thread():
    """Ultra-low latency non-blocking UDP receiver on port 8888 (<1ms)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(('0.0.0.0', 8888))
        sock.settimeout(0.5)
        print("[HUB] UDP telemetry listener successfully bound to 0.0.0.0:8888")
    except Exception as e:
        print(f"[HUB] Failed to bind UDP 8888: {e}")
        sock.close()
        return

    while True:
        try:
            data, addr = sock.recvfrom(2048)
            global last_rover_addr
            last_rover_addr = addr
            process_rover_payload(data.decode('utf-8', errors='ignore'), link_label="UDP")
        except socket.timeout:
            continue
        except Exception:
            pass


async def fallback_http_polling_loop():
    """Safe fallback HTTP poller (2Hz) only when UDP packets are silent for >1.5s."""
    loop = asyncio.get_event_loop()
    def fetch():
        try:
            req = urllib.request.Request('http://192.168.0.101/telem', headers={'Connection': 'close'})
            with urllib.request.urlopen(req, timeout=0.85) as r:
                return r.read().decode('utf-8')
        except Exception:
            return None

    while True:
        # If UDP is actively streaming, keep HTTP polling asleep (zero load on ESP32)
        if time.time() - last_udp_packet_time > 3.0:
            raw = await loop.run_in_executor(None, fetch)
            if raw:
                process_rover_payload(raw, link_label="HTTP")
        await asyncio.sleep(3.0)


async def poll_pi_cam_loop():
    """Background polling loop for Pi 3B+ camera status every 2.0 seconds."""
    global active_pi_cam_ip
    loop = asyncio.get_event_loop()
    def fetch_status():
        global active_pi_cam_ip
        candidates = [active_pi_cam_ip] + [ip for ip in PI_CAM_IPS if ip != active_pi_cam_ip]
        for ip in candidates:
            try:
                req = urllib.request.Request(f'http://{ip}:8081/status', headers={'Connection': 'close'})
                with urllib.request.urlopen(req, timeout=1.2) as r:
                    active_pi_cam_ip = ip
                    return r.read().decode('utf-8')
            except Exception:
                continue
        return None

    while True:
        try:
            raw = await loop.run_in_executor(None, fetch_status)
            if raw:
                try:
                    data = json.loads(raw)
                    cam_info = latest_telemetry.setdefault("pi_cam", {})
                    cam_info.update({
                        "cam_online": bool(data.get("cam_online", False)),
                        "stream_url": f"http://{active_pi_cam_ip}:8081/stream",
                        "vis_f_mm": int(data.get("vis_f_mm", 2000)),
                        "vis_l_mm": int(data.get("vis_l_mm", 2000)),
                        "vis_r_mm": int(data.get("vis_r_mm", 2000)),
                        "clear_heading_deg": float(data.get("clear_heading_deg", 0.0)),
                        "fps": float(data.get("fps", 0.0)),
                        "frame_seq": int(data.get("frame_seq", 0)),
                        "uart_tx": int(data.get("uart_tx", 0)),
                        "uart_rx": int(data.get("uart_rx", 0)),
                        "cpu_temp_c": float(data.get("cpu_temp_c", 0.0)),
                        "mem_avail_mb": int(data.get("mem_avail_mb", 0)),
                        "cpu_usage": float(data.get("cpu_usage", 0.0)),
                        "last_seen": time.time()
                    })
                except Exception:
                    pass
            else:
                cam_info = latest_telemetry.setdefault("pi_cam", {})
                cam_info["cam_online"] = False
        except Exception:
            cam_info = latest_telemetry.setdefault("pi_cam", {})
            cam_info["cam_online"] = False
        await asyncio.sleep(2.0)


def find_static_dir() -> Path:
    # 1. Prefer live source directory so updates are immediate without rebuilding package
    src_candidates = [
        Path("/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static"),
        Path("/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web/static")
    ]
    for c in src_candidates:
        if (c / "index.html").is_file():
            return c.resolve()

    try:
        from ament_index_python.packages import get_package_share_directory
        share_path = (Path(get_package_share_directory('mecanum_robot')) / 'web' / 'static').resolve()
        if (share_path / 'index.html').is_file():
            return share_path
    except Exception:
        pass

    module_dir = Path(__file__).resolve().parent
    candidates = [
        module_dir / "static",
        module_dir / "web" / "static",
        module_dir.parent / "web" / "static",
        module_dir.parent.parent / "web" / "static"
    ]
    for c in candidates:
        if (c / "index.html").is_file():
            return c.resolve()
    return module_dir.resolve()


STATIC_DIR = find_static_dir().resolve()
if STATIC_DIR.is_dir():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR), follow_symlink=True), name="static")

@app.get("/")
async def get_index():
    index_file = STATIC_DIR / "index.html"
    if index_file.is_file():
        return FileResponse(str(index_file))
    return HTMLResponse("<h3>Mecanum Robot Telemetry Server Online</h3>")

@app.get("/api/telemetry")
async def get_telemetry_snapshot():
    return JSONResponse(latest_telemetry)

@app.get("/boost")
@app.get("/boost/boost")
async def get_boost_dashboard():
    index_file = STATIC_DIR / "index.html"
    if index_file.is_file():
        return FileResponse(str(index_file))
    return HTMLResponse("<h3>Digital Twin Boost View</h3>")

@app.get("/video")
async def get_crawler_video():
    video_file = STATIC_DIR / "rover_crawl.mp4"
    if video_file.is_file():
        return FileResponse(str(video_file), media_type="video/mp4")
    return HTMLResponse("<h3>Crawler Video Not Found</h3>", status_code=404)

@app.get("/api/pi_cam/status")
async def get_pi_cam_status():
    """Returns the latest Raspberry Pi camera node telemetry."""
    return latest_telemetry.get("pi_cam", {})

STANDBY_JPEG = bytes([
    0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x01, 0x00, 0x48,
    0x00, 0x48, 0x00, 0x00, 0xFF, 0xFE, 0x02, 0x00
]) + (b'STANDBY ' * 64) + bytes([
    0xFF, 0xDB, 0x00, 0x43, 0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08,
    0x07, 0x07, 0x07, 0x09, 0x09, 0x08, 0x0A, 0x0C, 0x14, 0x0D, 0x0C, 0x0B, 0x0B, 0x0C, 0x19, 0x12,
    0x13, 0x0F, 0x14, 0x1D, 0x1A, 0x1F, 0x1E, 0x1D, 0x1A, 0x1C, 0x1C, 0x20, 0x24, 0x2E, 0x27, 0x20,
    0x22, 0x2C, 0x23, 0x1C, 0x1C, 0x28, 0x37, 0x29, 0x2C, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1F, 0x27,
    0x39, 0x3D, 0x38, 0x32, 0x3C, 0x2E, 0x33, 0x34, 0x32, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x11, 0x00, 0xFF, 0xC4, 0x00, 0x1F, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01, 0x00, 0x00, 0x3F,
    0x00, 0xBF, 0x80, 0xFF, 0xD9
])

@app.get("/api/pi_cam/snapshot")
async def get_pi_cam_snapshot():
    from fastapi.responses import Response
    def _fetch():
        global active_pi_cam_ip
        candidates = [active_pi_cam_ip] + [ip for ip in PI_CAM_IPS if ip != active_pi_cam_ip]
        for ip in candidates:
            try:
                req = urllib.request.Request(f'http://{ip}:8081/snapshot', headers={'Connection': 'close'})
                with urllib.request.urlopen(req, timeout=1.5) as r:
                    active_pi_cam_ip = ip
                    return r.read()
            except Exception:
                continue
        return None
    try:
        data = await asyncio.get_event_loop().run_in_executor(None, _fetch)
        if not data:
            return Response(content=STANDBY_JPEG, media_type="image/jpeg", headers={"Cache-Control": "no-store", "X-Camera-Status": "standby"}, status_code=200)
    except Exception:
        return Response(content=STANDBY_JPEG, media_type="image/jpeg", headers={"Cache-Control": "no-store", "X-Camera-Status": "standby"}, status_code=200)
    return Response(content=data, media_type="image/jpeg", headers={"Cache-Control": "no-store", "X-Camera-Status": "live"}, status_code=200)



@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await websocket.accept()
    active_connections.add(websocket)
    try:
        while True:
            raw_text = await websocket.receive_text()
            try:
                data = json.loads(raw_text)
                msg_type = data.get("type")

                if msg_type == "control":
                    vx_val = float(data.get("vx", 0.0))
                    vy_val = float(data.get("vy", 0.0))
                    vz_val = float(data.get("vz", 0.0))
                    spd_val = int(data.get("speed", 70))
                    mode_val = int(data.get("mode", latest_telemetry.get("mode", 0)))
                    flags_val = int(data.get("flags", 0))

                    # 1. Forward directly to rover over low-latency UDP (<1ms)
                    send_rover_drive_packet(vx_val, vy_val, vz_val, spd_val, mode_val, flags_val)

                    # 2. Publish to ROS 2 topic bus
                    if ros_node_bridge and rclpy:
                        cmd_msg = Twist()
                        cmd_msg.linear.x = vx_val
                        cmd_msg.linear.y = vy_val
                        cmd_msg.angular.z = vz_val
                        ros_node_bridge.pub_teleop.publish(cmd_msg)
                        if hasattr(ros_node_bridge, 'pub_cmd_vel'):
                            ros_node_bridge.pub_cmd_vel.publish(cmd_msg)

                        pt_msg = Vector3()
                        pt_msg.x = float(data.get("pan", 1500.0))
                        pt_msg.y = float(data.get("tilt", 1500.0))
                        ros_node_bridge.pub_pan_tilt.publish(pt_msg)

                elif msg_type == "set_mode":
                    mode_val = int(data.get("mode", 1))
                    global rover_cmd_seq
                    rover_cmd_seq = (rover_cmd_seq + 1) & 0xFFFF
                    pkt = struct.pack('<BBbbbBBhH', 0xA7, 5, 0, 0, 0, 70, mode_val, 0, 0, rover_cmd_seq)
                    targets = set()
                    if last_rover_addr:
                        targets.add((last_rover_addr[0], 8888))
                    targets.add(("192.168.0.105", 8888))
                    targets.add(("192.168.0.101", 8888))
                    for target in targets:
                        try:
                            udp_cmd_sock.sendto(pkt, target)
                        except Exception:
                            pass



                    if ros_node_bridge and rclpy:
                        m_msg = UInt8()
                        m_msg.data = mode_val
                        ros_node_bridge.pub_mode.publish(m_msg)
                    latest_telemetry["mode"] = mode_val
                    latest_telemetry["mode_name"] = MODE_NAMES.get(mode_val, "Custom")

            except Exception:
                pass

    except WebSocketDisconnect:
        active_connections.discard(websocket)
    except Exception:
        active_connections.discard(websocket)

async def broadcast_telemetry_loop():
    """Broadcasts 20Hz updates to all connected web clients."""
    while True:
        if active_connections:
            payload = json.dumps(latest_telemetry)
            disconnected = []
            for ws in list(active_connections):
                try:
                    await ws.send_text(payload)
                except Exception:
                    disconnected.append(ws)
            for ws in disconnected:
                active_connections.discard(ws)
        await asyncio.sleep(0.05)

@app.on_event("startup")
async def on_startup():
    # Start high-speed UDP background listener
    t = threading.Thread(target=udp_listener_thread, daemon=True)
    t.start()

    # Start broadcast, fallback poller, and ROS 2 autonomy engine
    asyncio.create_task(broadcast_telemetry_loop())
    asyncio.create_task(fallback_http_polling_loop())
    asyncio.create_task(poll_pi_cam_loop())
    asyncio.create_task(ros2_autonomy_computation_loop())

def main(args=None):
    global ros_node_bridge
    if rclpy:
        rclpy.init(args=args)
        ros_node_bridge = WebRosBridgeNode()

        # Run ROS spinning in separate background thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node_bridge,), daemon=True)
        ros_thread.start()

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=int(os.environ.get("PORT", 8090)),
        log_level="warning",
        access_log=False,
        workers=1
    )

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
ROS 2 ToF and Ultrasonic Sensor Fusion Node.
Converts dual VL53L0X Time-of-Flight sensors (~33 deg left/right, 10cm from center)
and demoted/filtered front ultrasonic (11cm from center) into:
  1. sensor_msgs/msg/LaserScan on /scan (projected via exact trigonometry)
  2. sensor_msgs/msg/Range on /sensors/tof_left/range and /sensors/tof_right/range
     (for Nav2 range_sensor_layer)
"""
import math
import socket
import select
from collections import deque
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Header


class TofLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('tof_laserscan_publisher')

        # Declare parameters
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('tof_angle_deg', 33.0)
        self.declare_parameter('tof_dist_m', 0.100)
        self.declare_parameter('cam_dist_m', 0.110)
        self.declare_parameter('min_range_m', 0.03)
        self.declare_parameter('max_range_m', 2.00)
        self.declare_parameter('tof_fov_rad', 0.436)  # ~25 deg for VL53L0X
        self.declare_parameter('scan_beams', 360)     # 1 degree per beam

        self.port = self.get_parameter('udp_port').value
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.tof_angle_deg = self.get_parameter('tof_angle_deg').value
        self.tof_angle_rad = math.radians(self.tof_angle_deg)
        self.tof_dist_m = self.get_parameter('tof_dist_m').value
        self.cam_dist_m = self.get_parameter('cam_dist_m').value
        self.min_range = self.get_parameter('min_range_m').value
        self.max_range = self.get_parameter('max_range_m').value
        self.tof_fov = self.get_parameter('tof_fov_rad').value
        self.num_beams = self.get_parameter('scan_beams').value

        # Sensor positions in body frame (base_link)
        # Fixed Front ToF: 0 deg, +11 cm from center (+0.110m, 0.0m)
        self.tof_front_x = 0.110
        self.tof_front_y = 0.0
        self.tof_front_yaw_rad = 0.0

        # Fixed Rear ToF: 180 deg, -10 cm from center (-0.100m, 0.0m)
        self.tof_rear_x = -0.100
        self.tof_rear_y = 0.0
        self.tof_rear_yaw_rad = math.pi

        # Latest readings (in meters, 0.0 = invalid / out of range)
        self.cur_tof_f_m = 0.0
        self.cur_tof_r_m = 0.0
        self.cur_tof_l_m = 0.0  # legacy alias
        self.cur_us_m = 0.0     # legacy alias

        # ROS 2 Publishers
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_tof_front_range = self.create_publisher(Range, '/sensors/tof_front/range', 10)
        self.pub_tof_rear_range  = self.create_publisher(Range, '/sensors/tof_rear/range', 10)
        self.pub_tof_l_range     = self.create_publisher(Range, '/sensors/tof_left/range', 10)   # backwards compat
        self.pub_tof_r_range     = self.create_publisher(Range, '/sensors/tof_right/range', 10)  # backwards compat
        self.pub_us_range        = self.create_publisher(Range, '/sensors/ultrasonic/range', 10) # backwards compat

        # Optional ROS 2 Subscriber fallback for range topics
        self.sub_tof_front = self.create_subscription(Range, '/sensors/tof_front', self._cb_tof_f, 10)
        self.sub_tof_rear  = self.create_subscription(Range, '/sensors/tof_rear', self._cb_tof_rear, 10)
        self.sub_tof_l     = self.create_subscription(Range, '/sensors/tof_left', self._cb_tof_l, 10)
        self.sub_tof_r     = self.create_subscription(Range, '/sensors/tof_right', self._cb_tof_r, 10)
        self.sub_us = self.create_subscription(Range, '/sensors/ultrasonic', self._cb_us, 10)

        # Non-blocking UDP socket to receive live packets directly from rover
        self.sock: Optional[socket.socket] = None
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.setblocking(False)
            self.get_logger().info(f"Listening for rover telemetry UDP on port {self.port}")
        except Exception as e:
            self.get_logger().warning(f"UDP bind on {self.port} skipped ({e}); using ROS topics.")

        # Periodic timer for publishing LaserScan and Range
        dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(dt, self._on_timer)

    def _cb_tof_f(self, msg: Range):
        if self.min_range <= msg.range <= self.max_range:
            self.cur_tof_f_m = msg.range
            self.cur_tof_l_m = msg.range
        else:
            self.cur_tof_f_m = 0.0
            self.cur_tof_l_m = 0.0

    def _cb_tof_rear(self, msg: Range):
        if self.min_range <= msg.range <= self.max_range:
            self.cur_tof_r_m = msg.range
        else:
            self.cur_tof_r_m = 0.0

    def _cb_tof_l(self, msg: Range):
        self._cb_tof_f(msg)

    def _cb_tof_r(self, msg: Range):
        self._cb_tof_rear(msg)

    def _cb_us(self, msg: Range):
        pass

    def _poll_udp(self):
        """Reads UDP telemetry packets from RP2040 rover."""
        if not self.sock:
            return
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                txt = data.decode('utf-8', errors='ignore').strip()
                if txt.startswith("R,"):
                    # Format: R,heading,ultraMm,tofMm[0],tofMm[1],mode
                    parts = txt.split(",")
                    if len(parts) >= 5:
                        tof_f_mm = float(parts[3])
                        tof_r_mm = float(parts[4])

                        if 25 <= tof_f_mm <= 2000:
                            self.cur_tof_f_m = tof_f_mm / 1000.0
                        else:
                            self.cur_tof_f_m = 0.0
                        self.cur_tof_l_m = self.cur_tof_f_m

                        if 25 <= tof_r_mm <= 2000:
                            self.cur_tof_r_m = tof_r_mm / 1000.0
                        else:
                            self.cur_tof_r_m = 0.0
            except (BlockingIOError, socket.error):
                break

    def _project_point(self, origin_x: float, origin_y: float, sensor_yaw_rad: float, dist_m: float) -> Tuple[float, float]:
        """Trigonometrically projects sensor beam into robot body frame."""
        x_hit = origin_x + dist_m * math.cos(sensor_yaw_rad)
        y_hit = origin_y + dist_m * math.sin(sensor_yaw_rad)
        range_from_center = math.hypot(x_hit, y_hit)
        angle_from_center = math.atan2(y_hit, x_hit)
        return range_from_center, angle_from_center

    def _on_timer(self):
        self._poll_udp()
        now = self.get_clock().now().to_msg()

        # 1. Publish sensor_msgs/msg/Range for ToF Front
        r_f = Range()
        r_f.header.stamp = now
        r_f.header.frame_id = 'tof_front_link'
        r_f.radiation_type = Range.INFRARED
        r_f.field_of_view = self.tof_fov
        r_f.min_range = self.min_range
        r_f.max_range = self.max_range
        r_f.range = self.cur_tof_f_m if self.cur_tof_f_m > 0 else float('inf')
        self.pub_tof_front_range.publish(r_f)
        self.pub_tof_l_range.publish(r_f)

        # 2. Publish sensor_msgs/msg/Range for ToF Rear
        r_r = Range()
        r_r.header.stamp = now
        r_r.header.frame_id = 'tof_rear_link'
        r_r.radiation_type = Range.INFRARED
        r_r.field_of_view = self.tof_fov
        r_r.min_range = self.min_range
        r_r.max_range = self.max_range
        r_r.range = self.cur_tof_r_m if self.cur_tof_r_m > 0 else float('inf')
        self.pub_tof_rear_range.publish(r_r)
        self.pub_tof_r_range.publish(r_r)

        # 3. Synthesize 360-degree 2D LaserScan projected onto /scan
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'scan'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2.0 * math.pi) / self.num_beams
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate_hz
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = [float('inf')] * self.num_beams

        def put_hit(orig_x, orig_y, yaw_rad, d_m, spread_rad=0.08):
            if d_m < self.min_range or d_m > self.max_range:
                return
            r, th = self._project_point(orig_x, orig_y, yaw_rad, d_m)
            idx = int(round((th - scan.angle_min) / scan.angle_increment))
            spread_beams = max(1, int(round(spread_rad / scan.angle_increment)))
            for offset in range(-spread_beams, spread_beams + 1):
                b_idx = (idx + offset) % self.num_beams
                if r < scan.ranges[b_idx]:
                    scan.ranges[b_idx] = r

        # Project Front ToF (0 deg, 11cm from center)
        if self.cur_tof_f_m > 0:
            put_hit(self.tof_front_x, self.tof_front_y, self.tof_front_yaw_rad, self.cur_tof_f_m, spread_rad=0.15)

        # Project Rear ToF (180 deg, -10cm from center)
        if self.cur_tof_r_m > 0:
            put_hit(self.tof_rear_x, self.tof_rear_y, self.tof_rear_yaw_rad, self.cur_tof_r_m, spread_rad=0.15)

        self.pub_scan.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = TofLaserScanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

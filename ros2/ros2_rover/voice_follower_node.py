#!/usr/bin/env python3
"""
Voice Follower Coordinator Node (Stop-Listen-Turn-Approach State Machine).
States:
  0. IDLE_LISTEN: Rover stopped, listening for voice. Ego-noise gating enforces
     rejection of any DOA if rover velocity != 0.
  1. COMPUTE_BEARING: Calculates global target bearing via atan2(sin(θ_r + α), cos(θ_r + α)).
  2. ROTATE_TO_SPEAKER: Pure in-place yaw rotation to face speaker within tolerance.
  3. RADAR_ACQUIRE: Forward 24GHz mmWave radar acquisition within ±60 deg FOV.
  4. APPROACH_HUMAN: Proportional approach stopping smoothly at 1.0 m.
  5. ARRIVED_HOLD: Arrived at speaker, holding position.
"""
import math
import sys
import time
from enum import Enum
from typing import Optional, Dict, Any

# Conditional ROS 2 imports for standalone mockability
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32, Bool
    from geometry_msgs.msg import Twist
    try:
        from nav_msgs.msg import Odometry
    except ImportError:
        Odometry = None
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    Odometry = None
    class Node:  # type: ignore
        def __init__(self, name: str):
            self.name = name
            self._parameters: Dict[str, Any] = {}
        def declare_parameter(self, name: str, default_value: Any):
            self._parameters[name] = default_value
        def get_parameter(self, name: str):
            class _Param:
                def __init__(self, v): self.value = v
            return _Param(self._parameters.get(name))
        def create_publisher(self, msg_type, topic, qos):
            return MockPublisher(topic, msg_type)
        def create_subscription(self, msg_type, topic, callback, qos):
            return MockSubscription(topic, msg_type, callback)
        def create_timer(self, period_sec, callback):
            return MockTimer(period_sec, callback)
        def get_logger(self):
            return MockLogger()
        def destroy_node(self): pass

    class MockPublisher:
        def __init__(self, topic: str, msg_type: Any):
            self.topic = topic
            self.msg_type = msg_type
            self.last_msg = None
        def publish(self, msg: Any):
            self.last_msg = msg

    class MockSubscription:
        def __init__(self, topic: str, msg_type: Any, callback: Any):
            self.topic = topic
            self.msg_type = msg_type
            self.callback = callback

    class MockTimer:
        def __init__(self, period_sec: float, callback: Any):
            self.period_sec = period_sec
            self.callback = callback
            self.is_active = True
        def cancel(self): self.is_active = False

    class MockLogger:
        def info(self, msg): print(f"[INFO] [voice_follower]: {msg}")
        def warn(self, msg): print(f"[WARN] [voice_follower]: {msg}")
        def error(self, msg): print(f"[ERROR] [voice_follower]: {msg}")

    class String:  # type: ignore
        def __init__(self, data: str = ""): self.data = data
    class Float32:  # type: ignore
        def __init__(self, data: float = 0.0): self.data = data
    class Bool:  # type: ignore
        def __init__(self, data: bool = False): self.data = data
    class Vector3:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x; self.y = y; self.z = z
    class Twist:  # type: ignore
        def __init__(self):
            self.linear = Vector3()
            self.angular = Vector3()


class FollowerState(Enum):
    IDLE_LISTEN = "IDLE_LISTEN"
    COMPUTE_BEARING = "COMPUTE_BEARING"
    ROTATE_TO_SPEAKER = "ROTATE_TO_SPEAKER"
    RADAR_ACQUIRE = "RADAR_ACQUIRE"
    APPROACH_HUMAN = "APPROACH_HUMAN"
    ARRIVED_HOLD = "ARRIVED_HOLD"


class VoiceFollowerStateMachine:
    """Pure logic state machine for deterministic testing without ROS."""
    def __init__(
        self,
        stop_dist_m: float = 1.0,
        kp_yaw: float = 1.2,
        kp_dist: float = 0.45,
        max_yaw_rate: float = 1.0,
        min_yaw_rate: float = 0.35,
        max_fwd_speed: float = 0.25,
        yaw_tolerance_rad: float = math.radians(5.0),
        doa_conf_threshold: float = 0.40,
        radar_timeout_sec: float = 2.0,
        hold_duration_sec: float = 4.0
    ):
        self.stop_dist_m = stop_dist_m
        self.kp_yaw = kp_yaw
        self.kp_dist = kp_dist
        self.max_yaw_rate = max_yaw_rate
        self.min_yaw_rate = min_yaw_rate
        self.max_fwd_speed = max_fwd_speed
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.doa_conf_threshold = doa_conf_threshold
        self.radar_timeout_sec = radar_timeout_sec
        self.hold_duration_sec = hold_duration_sec

        self.state = FollowerState.IDLE_LISTEN
        self.current_heading_rad = 0.0
        self.target_heading_rad = 0.0
        self.last_doa_azimuth_rad = 0.0
        self.last_doa_conf = 0.0
        self.radar_present = False
        self.radar_distance_m = 0.0

        # Ego-motion tracking
        self.current_vx = 0.0
        self.current_wz = 0.0

        # Timers
        self.state_enter_time = time.time()
        self.last_radar_seen_time = 0.0

    def set_ego_velocity(self, vx: float, wz: float):
        """Updates internal awareness of rover velocity for ego-noise gating."""
        self.current_vx = vx
        self.current_wz = wz

    def is_ego_moving(self) -> bool:
        """Returns True if rover is actively moving (ego-noise threshold)."""
        return abs(self.current_vx) > 0.01 or abs(self.current_wz) > 0.02

    def update_heading(self, heading_rad: float):
        self.current_heading_rad = heading_rad

    def on_doa_received(self, azimuth_deg: float, confidence: float, now: Optional[float] = None) -> bool:
        """
        Processes new DOA reading.
        Applies strict ego-noise gating: rejects any DOA if rover is currently moving.
        Returns True if accepted and triggered state transition.
        """
        now_ts = now if now is not None else time.time()

        # Ego-noise gating check: MUST be stationary
        if self.is_ego_moving():
            return False

        if confidence < self.doa_conf_threshold:
            return False

        if self.state not in (FollowerState.IDLE_LISTEN, FollowerState.ARRIVED_HOLD):
            return False

        self.last_doa_azimuth_rad = math.radians(azimuth_deg)
        self.last_doa_conf = confidence

        # Transition to COMPUTE_BEARING
        self.state = FollowerState.COMPUTE_BEARING
        self.state_enter_time = now_ts
        self._compute_target_bearing(now=now_ts)
        return True

    def on_radar_update(self, present: bool, distance_m: float, now: Optional[float] = None):
        """Updates radar presence and target distance."""
        now_ts = now if now is not None else time.time()
        self.radar_present = present
        self.radar_distance_m = distance_m
        if present:
            self.last_radar_seen_time = now_ts

    def _compute_target_bearing(self, now: Optional[float] = None):
        """
        Target heading conversion:
        θ_target = atan2(sin(θ_rover + α_doa), cos(θ_rover + α_doa))
        """
        now_ts = now if now is not None else time.time()
        total_angle = self.current_heading_rad + self.last_doa_azimuth_rad
        self.target_heading_rad = math.atan2(math.sin(total_angle), math.cos(total_angle))
        self.state = FollowerState.ROTATE_TO_SPEAKER
        self.state_enter_time = now_ts

    def step(self, now: Optional[float] = None) -> Tuple[float, float, str]:
        """
        Executes one step of the state machine.
        Returns (cmd_vx, cmd_wz, state_str).
        """
        now_ts = now if now is not None else time.time()
        vx = 0.0
        wz = 0.0

        if self.state == FollowerState.IDLE_LISTEN:
            vx = 0.0
            wz = 0.0

        elif self.state == FollowerState.COMPUTE_BEARING:
            self._compute_target_bearing(now=now_ts)

        elif self.state == FollowerState.ROTATE_TO_SPEAKER:
            # Heading error wrapped to [-pi, +pi]
            heading_err = math.atan2(
                math.sin(self.target_heading_rad - self.current_heading_rad),
                math.cos(self.target_heading_rad - self.current_heading_rad)
            )

            if abs(heading_err) <= self.yaw_tolerance_rad:
                # Aligned with speaker! Stop rotation and transition to radar acquisition
                vx = 0.0
                wz = 0.0
                self.state = FollowerState.RADAR_ACQUIRE
                self.state_enter_time = now_ts
            else:
                # Proportional rotation with stiction threshold
                raw_wz = self.kp_yaw * heading_err
                sign = 1.0 if raw_wz > 0 else -1.0
                mag = max(self.min_yaw_rate, min(self.max_yaw_rate, abs(raw_wz)))
                wz = sign * mag
                vx = 0.0

                # Rotation timeout (e.g. 7 seconds)
                if (now_ts - self.state_enter_time) > 7.0:
                    self.state = FollowerState.IDLE_LISTEN
                    wz = 0.0

        elif self.state == FollowerState.RADAR_ACQUIRE:
            vx = 0.0
            wz = 0.0
            # Check if radar sees human within FOV
            if self.radar_present and self.radar_distance_m > 0.3:
                self.state = FollowerState.APPROACH_HUMAN
                self.state_enter_time = now_ts
                self.last_radar_seen_time = now_ts
            elif (now_ts - self.state_enter_time) > self.radar_timeout_sec:
                # Radar failed to acquire human
                self.state = FollowerState.IDLE_LISTEN

        elif self.state == FollowerState.APPROACH_HUMAN:
            # Check for radar target presence
            if not self.radar_present or self.radar_distance_m <= 0.3:
                # Target temporarily not visible: pause forward motion
                vx = 0.0
                wz = 0.0
                if (now_ts - self.last_radar_seen_time) > 1.2:
                    # Target lost for over 1.2s: abort approach and return to listening
                    self.state = FollowerState.IDLE_LISTEN
            else:
                dist_error = self.radar_distance_m - self.stop_dist_m
                if dist_error <= 0.05:
                    # Within stopping margin (<= 1.05m): reached speaker
                    vx = 0.0
                    wz = 0.0
                    self.state = FollowerState.ARRIVED_HOLD
                    self.state_enter_time = now_ts
                else:
                    # Smooth proportional approach
                    raw_vx = self.kp_dist * dist_error
                    vx = max(0.0, min(self.max_fwd_speed, raw_vx))
                    wz = 0.0

        elif self.state == FollowerState.ARRIVED_HOLD:
            vx = 0.0
            wz = 0.0
            if (now_ts - self.state_enter_time) > self.hold_duration_sec:
                self.state = FollowerState.IDLE_LISTEN

        self.set_ego_velocity(vx, wz)
        return vx, wz, self.state.value


class VoiceFollowerNode(Node):
    """ROS 2 Node orchestrating the Stop-Listen-Turn-Approach pipeline."""
    def __init__(self):
        super().__init__('voice_follower_node')

        self.declare_parameter('stop_distance_m', 1.0)
        self.declare_parameter('kp_yaw', 1.2)
        self.declare_parameter('kp_dist', 0.45)
        self.declare_parameter('max_speed', 0.25)
        self.declare_parameter('control_rate_hz', 20.0)

        stop_dist = float(self.get_parameter('stop_distance_m').value)
        kp_yaw = float(self.get_parameter('kp_yaw').value)
        kp_dist = float(self.get_parameter('kp_dist').value)
        max_speed = float(self.get_parameter('max_speed').value)
        rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.fsm = VoiceFollowerStateMachine(
            stop_dist_m=stop_dist,
            kp_yaw=kp_yaw,
            kp_dist=kp_dist,
            max_fwd_speed=max_speed
        )

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/voice_follower/state', 10)

        # Subscriptions
        self.sub_azimuth = self.create_subscription(
            Float32, '/audio/doa_azimuth', self._on_doa_azimuth, 10
        )
        self.sub_confidence = self.create_subscription(
            Float32, '/audio/doa_confidence', self._on_doa_confidence, 10
        )
        self.sub_radar_pres = self.create_subscription(
            Bool, '/radar/presence', self._on_radar_pres, 10
        )
        self.sub_radar_dist = self.create_subscription(
            Float32, '/radar/distance', self._on_radar_dist, 10
        )
        self.sub_heading = self.create_subscription(
            Float32, '/sensors/heading', self._on_heading, 10
        )
        if Odometry is not None:
            self.sub_odom = self.create_subscription(
                Odometry, '/odom', self._on_odom, 10
            )

        self.cached_doa_az = 0.0
        self.cached_doa_conf = 0.0

        self.timer = self.create_timer(1.0 / rate_hz, self._loop)

    def _on_odom(self, msg):
        try:
            vx = float(msg.twist.twist.linear.x)
            wz = float(msg.twist.twist.angular.z)
            self.fsm.set_ego_velocity(vx, wz)
        except Exception:
            pass

    def _try_trigger_doa(self):
        if self.cached_doa_conf > 0.0:
            if self.fsm.on_doa_received(self.cached_doa_az, self.cached_doa_conf):
                self.cached_doa_conf = 0.0

    def _on_doa_azimuth(self, msg):
        self.cached_doa_az = float(msg.data)
        self._try_trigger_doa()

    def _on_doa_confidence(self, msg):
        self.cached_doa_conf = float(msg.data)
        self._try_trigger_doa()

    def _on_radar_pres(self, msg):
        self.fsm.on_radar_update(bool(msg.data), self.fsm.radar_distance_m)

    def _on_radar_dist(self, msg):
        self.fsm.on_radar_update(self.fsm.radar_present, float(msg.data))

    def _on_heading(self, msg):
        self.fsm.update_heading(math.radians(float(msg.data)))

    def _loop(self):
        vx, wz, state_str = self.fsm.step()

        # Publish Twist
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        self.pub_cmd_vel.publish(twist)

        # Publish State
        state_msg = String()
        state_msg.data = state_str
        self.pub_state.publish(state_msg)


def main(args=None):
    if RCLPY_AVAILABLE:
        rclpy.init(args=args)
        node = VoiceFollowerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[voice_follower_node] rclpy not installed. Mock node initialized.")
        node = VoiceFollowerNode()


if __name__ == '__main__':
    main()

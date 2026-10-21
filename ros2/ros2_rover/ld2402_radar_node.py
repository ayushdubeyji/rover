#!/usr/bin/env python3
"""
HLK-LD2402 24GHz FMCW Human Presence Radar ROS 2 Driver & Standalone Parser.
Specifications:
  - UART: 115200 8N1
  - Command framing: [0xFD 0xFC 0xFB 0xFA] [Len (2B)] [Cmd (2B)] [Payload] [0x04 0x03 0x02 0x01]
  - ACK framing: [0xFD 0xFC 0xFB 0xFA] [Len (2B)] [Cmd|0x0100 (2B)] [Status (2B)] [Data] [0x04 0x03 0x02 0x01]
  - Engineering data frame: [0xF4 0xF3 0xF2 0xF1] [Len=131 (2B)] [State (1B)] [Dist_cm (2B)] [32x Gates (128B)] [0xF8 0xF7 0xF6 0xF5]
  - ROS 2 topics:
      /radar/presence    (std_msgs/msg/Bool)
      /radar/distance    (std_msgs/msg/Float32 in meters)
      /radar/gate_energy (std_msgs/msg/Int32MultiArray)
"""
import struct
import sys
import time
from typing import Optional, Tuple, Dict, Any, List

# Conditional ROS 2 imports for standalone mockability
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, Int32MultiArray
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    # Mock base classes for standalone execution & unit testing
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

    class MockTimer:
        def __init__(self, period_sec: float, callback: Any):
            self.period_sec = period_sec
            self.callback = callback
            self.is_active = True
        def cancel(self):
            self.is_active = False

    class MockLogger:
        def info(self, msg): print(f"[INFO] [ld2402_radar]: {msg}")
        def warn(self, msg): print(f"[WARN] [ld2402_radar]: {msg}")
        def error(self, msg): print(f"[ERROR] [ld2402_radar]: {msg}")

    class Bool:  # type: ignore
        def __init__(self, data: bool = False): self.data = data
    class Float32:  # type: ignore
        def __init__(self, data: float = 0.0): self.data = data
    class Int32MultiArray:  # type: ignore
        def __init__(self, data: Optional[List[int]] = None): self.data = data or []


class LD2402Protocol:
    """Frame builder and parser for HLK-LD2402 radar sensor."""
    CMD_HEADER = bytes([0xFD, 0xFC, 0xFB, 0xFA])
    CMD_TAIL = bytes([0x04, 0x03, 0x02, 0x01])

    DATA_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
    DATA_TAIL = bytes([0xF8, 0xF7, 0xF6, 0xF5])

    # Command codes
    CMD_ENABLE_CONFIG = 0x00FF
    CMD_END_CONFIG = 0x00FE
    CMD_READ_FW_VER = 0x0000
    CMD_WRITE_PARAM = 0x0007
    CMD_READ_PARAM = 0x0008
    CMD_SET_OUTPUT_MODE = 0x0012
    CMD_SAVE_PARAMS = 0x00FD
    CMD_AUTO_GAIN = 0x00EE

    # Mode values
    MODE_ENGINEERING = 0x00000004
    MODE_NORMAL = 0x00000064

    # Target states
    STATE_NOBODY = 0x00
    STATE_MOVING = 0x01
    STATE_STATIONARY = 0x02

    @staticmethod
    def build_command_frame(cmd: int, payload: bytes = b'') -> bytes:
        """Constructs a command frame with header, 2B length, 2B command, payload, and tail."""
        length = 2 + len(payload)
        header_len_cmd = LD2402Protocol.CMD_HEADER + struct.pack('<HH', length, cmd)
        return header_len_cmd + payload + LD2402Protocol.CMD_TAIL

    @staticmethod
    def build_enable_config() -> bytes:
        """Unlock configuration mode (0x00FF, payload 0x0001)."""
        return LD2402Protocol.build_command_frame(
            LD2402Protocol.CMD_ENABLE_CONFIG,
            struct.pack('<H', 0x0001)
        )

    @staticmethod
    def build_end_config() -> bytes:
        """Exit configuration mode (0x00FE, no payload)."""
        return LD2402Protocol.build_command_frame(LD2402Protocol.CMD_END_CONFIG, b'')

    @staticmethod
    def build_engineering_mode(enable: bool = True) -> bytes:
        """Set output mode: 0x00000004 for engineering mode, 0x00000064 for normal."""
        mode_val = LD2402Protocol.MODE_ENGINEERING if enable else LD2402Protocol.MODE_NORMAL
        payload = struct.pack('<HI', 0x0000, mode_val)
        return LD2402Protocol.build_command_frame(LD2402Protocol.CMD_SET_OUTPUT_MODE, payload)

    @staticmethod
    def build_save_parameters() -> bytes:
        """Persist configuration into flash (0x00FD)."""
        return LD2402Protocol.build_command_frame(LD2402Protocol.CMD_SAVE_PARAMS, b'')

    @staticmethod
    def build_read_param(param_id: int) -> bytes:
        """Read configuration register (0x0008)."""
        return LD2402Protocol.build_command_frame(
            LD2402Protocol.CMD_READ_PARAM,
            struct.pack('<H', param_id)
        )

    @staticmethod
    def build_flash_persistence_sequence(enable_eng_mode: bool = True) -> List[bytes]:
        """Complete unlock -> configure -> read prerequisite (0x003F) -> persist -> end sequence."""
        return [
            LD2402Protocol.build_enable_config(),
            LD2402Protocol.build_engineering_mode(enable_eng_mode),
            LD2402Protocol.build_read_param(0x003F),  # Prerequisite verification
            LD2402Protocol.build_save_parameters(),
            LD2402Protocol.build_end_config()
        ]

    @staticmethod
    def parse_ack_frame(buffer: bytes) -> Optional[Tuple[Dict[str, Any], int]]:
        """
        Parses command ACK frame.
        Returns ({'cmd': int, 'status': int, 'data': bytes}, consumed_bytes) or None.
        """
        idx = buffer.find(LD2402Protocol.CMD_HEADER)
        if idx == -1:
            return None
        if len(buffer) < idx + 8:
            return None

        length, = struct.unpack('<H', buffer[idx + 4:idx + 6])
        total_frame_len = 4 + 2 + length + 4
        if len(buffer) < idx + total_frame_len:
            return None

        frame_data = buffer[idx:idx + total_frame_len]
        if not frame_data.endswith(LD2402Protocol.CMD_TAIL):
            return None

        # ACK format: Cmd|0x0100 (2B), Status (2B: 0=OK, 1=ERR), Data
        ack_cmd, status = struct.unpack('<HH', frame_data[6:10])
        ack_payload = frame_data[10:-4]
        original_cmd = ack_cmd & ~0x0100

        result = {
            'cmd': original_cmd,
            'ack_cmd': ack_cmd,
            'status': status,
            'ok': (status == 0),
            'data': ack_payload
        }
        return result, idx + total_frame_len

    @staticmethod
    def parse_engineering_frame(buffer: bytes) -> Optional[Tuple[Dict[str, Any], int]]:
        """
        Parses 131-byte engineering periodic data frame.
        Structure:
          Header: 0xF4 0xF3 0xF2 0xF1 (4B)
          Length: 0x83 0x00 (2B, 131)
          Target state: 1B (0=Nobody, 1=Moving, 2=Stationary)
          Distance: 2B uint16 LE (cm)
          Gates: 128B (32 x uint32 LE)
          Tail: 0xF8 0xF7 0xF6 0xF5 (4B)
        Returns (parsed_dict, consumed_bytes) or None.
        """
        idx = buffer.find(LD2402Protocol.DATA_HEADER)
        if idx == -1:
            return None

        if len(buffer) < idx + 6:
            return None

        length, = struct.unpack('<H', buffer[idx + 4:idx + 6])
        if length != 131:
            return None

        total_len = 4 + 2 + length + 4  # 141 bytes
        if len(buffer) < idx + total_len:
            return None

        frame = buffer[idx:idx + total_len]
        if frame[-4:] != LD2402Protocol.DATA_TAIL:
            return None

        target_state = frame[6]
        distance_cm, = struct.unpack('<H', frame[7:9])
        gates = list(struct.unpack('<32I', frame[9:137]))

        is_present = (target_state in (LD2402Protocol.STATE_MOVING, LD2402Protocol.STATE_STATIONARY))
        distance_m = distance_cm / 100.0 if is_present else 0.0

        state_names = {0: 'NOBODY', 1: 'MOVING', 2: 'STATIONARY'}

        result = {
            'presence': is_present,
            'target_state': target_state,
            'target_state_name': state_names.get(target_state, 'UNKNOWN'),
            'distance_cm': distance_cm,
            'distance_m': distance_m,
            'gate_energies': gates,
            'peak_gate_energy': max(gates) if gates else 0,
            'peak_gate_index': gates.index(max(gates)) if gates else -1
        }
        return result, idx + total_len


class LD2402RadarNode(Node):
    """ROS 2 Node handling HLK-LD2402 serial communication and publishing presence/ranging data."""
    def __init__(self, serial_port: Optional[str] = None):
        super().__init__('ld2402_radar_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('engineering_mode', True)
        self.declare_parameter('frame_rate_hz', 20.0)

        self.port_name = serial_port or str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.engineering_mode = bool(self.get_parameter('engineering_mode').value)
        self.frame_rate_hz = float(self.get_parameter('frame_rate_hz').value)

        # Publishers
        self.pub_presence = self.create_publisher(Bool, '/radar/presence', 10)
        self.pub_distance = self.create_publisher(Float32, '/radar/distance', 10)
        self.pub_energy = self.create_publisher(Int32MultiArray, '/radar/gate_energy', 10)

        self.rx_buffer = bytearray()
        self.serial_conn = None
        self.last_presence = False
        self.last_distance_m = 0.0
        self.last_gates: List[int] = [0] * 32

        self._init_serial()
        self.timer = self.create_timer(1.0 / self.frame_rate_hz, self._poll_radar)

    def _init_serial(self):
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=0.05
            )
            self.get_logger().info(f"Opened HLK-LD2402 on {self.port_name} @ {self.baudrate}")
            # Send config commands to ensure engineering mode
            if self.engineering_mode:
                self.configure_sensor(enable_engineering=True)
        except Exception as e:
            self.get_logger().warn(f"Could not open serial port {self.port_name}: {e}. Running mock/sim mode.")
            self.serial_conn = None

    def configure_sensor(self, enable_engineering: bool = True):
        """Sends command sequence to sensor."""
        if not self.serial_conn:
            return
        seq = LD2402Protocol.build_flash_persistence_sequence(enable_eng_mode=enable_engineering)
        for cmd_bytes in seq:
            self.serial_conn.write(cmd_bytes)
            time.sleep(0.04)

    def process_incoming_bytes(self, raw_bytes: bytes) -> List[Dict[str, Any]]:
        """Appends bytes to internal buffer and parses all complete frames."""
        self.rx_buffer.extend(raw_bytes)
        frames_parsed = []

        while True:
            # 1. Discard any leading junk before DATA_HEADER
            h_idx = self.rx_buffer.find(LD2402Protocol.DATA_HEADER)
            if h_idx > 0:
                del self.rx_buffer[:h_idx]
            elif h_idx == -1:
                if len(self.rx_buffer) > 16:
                    del self.rx_buffer[:-16]
                break

            # 2. Check header and length (minimum 6 bytes needed)
            if len(self.rx_buffer) < 6:
                break

            length, = struct.unpack('<H', self.rx_buffer[4:6])
            if length != 131:
                # Corrupted header or false sync word: advance past false header
                del self.rx_buffer[:4]
                continue

            # 3. Check full frame completeness
            total_len = 4 + 2 + length + 4  # 141 bytes
            if len(self.rx_buffer) < total_len:
                # Incomplete frame, wait for remaining bytes
                break

            res = LD2402Protocol.parse_engineering_frame(bytes(self.rx_buffer[:total_len]))
            if res is None:
                # Invalid frame tail or framing: advance past false header
                del self.rx_buffer[:4]
                continue

            data, consumed = res
            del self.rx_buffer[:consumed]
            frames_parsed.append(data)

            # Update cache
            self.last_presence = data['presence']
            self.last_distance_m = data['distance_m']
            self.last_gates = data['gate_energies']

            # Publish
            self._publish_data(data)

        return frames_parsed

    def _publish_data(self, data: Dict[str, Any]):
        msg_pres = Bool()
        msg_pres.data = data['presence']
        self.pub_presence.publish(msg_pres)

        msg_dist = Float32()
        msg_dist.data = float(data['distance_m'])
        self.pub_distance.publish(msg_dist)

        msg_energy = Int32MultiArray()
        msg_energy.data = [int(v) for v in data['gate_energies']]
        self.pub_energy.publish(msg_energy)

    def _poll_radar(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                available = self.serial_conn.in_waiting
                if available > 0:
                    chunk = self.serial_conn.read(available)
                    if chunk:
                        self.process_incoming_bytes(chunk)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")


def main(args=None):
    if RCLPY_AVAILABLE:
        rclpy.init(args=args)
        node = LD2402RadarNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[ld2402_radar_node] rclpy not installed. Running standalone mock.")
        node = LD2402RadarNode()
        print("Mock node initialized successfully.")


if __name__ == '__main__':
    main()

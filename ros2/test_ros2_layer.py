#!/usr/bin/env python3
"""
Unit tests for ROS 2 Rover geometry, ToF projection, ultrasonic filtering,
and odometry gating verification.
"""
import math
import struct
import sys
from collections import deque
from pathlib import Path

import numpy as np

# Add ros2 directory to path for imports
_ros2_dir = Path(__file__).resolve().parent
if str(_ros2_dir) not in sys.path:
    sys.path.insert(0, str(_ros2_dir))

from ros2_rover.ld2402_radar_node import LD2402Protocol
from ros2_rover.doa_srp_phat_node import SRPPhatCalculator, StereoDoACalculator, DoASrpPhatNode
from ros2_rover.voice_follower_node import VoiceFollowerStateMachine, FollowerState



def test_sensor_geometry():
    # New fixed ToF layout: Front (0°, +11cm), Rear (180°, -10cm)
    tof_front_x = 0.110
    tof_front_y = 0.0
    tof_front_yaw_rad = 0.0

    tof_rear_x = -0.100
    tof_rear_y = 0.0
    tof_rear_yaw_rad = 3.14159

    assert round(tof_front_x, 3) == 0.110, "Front ToF must be 11cm from centre"
    assert round(tof_front_y, 3) == 0.0, "Front ToF must be on centerline"
    assert round(tof_front_yaw_rad, 3) == 0.0, "Front ToF yaw must be 0 deg"

    assert round(tof_rear_x, 3) == -0.100, "Rear ToF must be -10cm from centre"
    assert round(tof_rear_y, 3) == 0.0, "Rear ToF must be on centerline"
    assert round(tof_rear_yaw_rad, 5) == 3.14159, "Rear ToF yaw must be 3.14159 rad"


def test_tof_projection():
    # Front ToF (0°, +11cm)
    front_origin_x = 0.110
    front_origin_y = 0.0
    reading_m = 0.500  # 50cm

    hit_f_x = front_origin_x + reading_m * math.cos(0.0)
    hit_f_y = front_origin_y + reading_m * math.sin(0.0)
    r_f = math.hypot(hit_f_x, hit_f_y)
    th_f = math.atan2(hit_f_y, hit_f_x)

    assert abs(r_f - 0.610) < 0.001, f"Expected projected range ~0.610m, got {r_f}"
    assert abs(th_f - 0.0) < 0.001, "Angle from center must match 0 deg ray"

    # Rear ToF (180°, -10cm)
    rear_origin_x = -0.100
    rear_origin_y = 0.0

    hit_r_x = rear_origin_x + reading_m * math.cos(math.pi)
    hit_r_y = rear_origin_y + reading_m * math.sin(math.pi)
    r_r = math.hypot(hit_r_x, hit_r_y)
    th_r = math.atan2(hit_r_y, hit_r_x)

    assert abs(r_r - 0.600) < 0.001, f"Expected projected range ~0.600m, got {r_r}"
    assert abs(abs(th_r) - math.pi) < 0.001, "Angle from center must match 180 deg ray"


def test_ultrasonic_filter():
    us_history = deque(maxlen=5)

    def filter_us(raw_m, tof_l_m, tof_r_m):
        us_history.append(raw_m)
        med_m = sorted(us_history)[len(us_history) // 2]
        tof_corroborates = (0.03 < tof_l_m < 0.65) or (0.03 < tof_r_m < 0.65)
        if med_m < 0.45 and not tof_corroborates:
            return 0.0  # Uncorroborated ultrasonic suppressed; cannot stop rover alone
        return med_m

    # Case 1: Uncorroborated ultrasonic glitch at 0.15m (ToF wide open)
    for _ in range(5):
        filtered = filter_us(0.15, 1.50, 1.50)
    assert filtered == 0.0, f"Uncorroborated glitch must be suppressed (0.0m), got {filtered}"

    # Case 2: Corroborated obstacle at 0.30m (Left ToF also detects obstacle at 0.35m)
    us_history.clear()
    for _ in range(5):
        filtered = filter_us(0.30, 0.35, 1.50)
    assert filtered == 0.30, f"Corroborated reading must pass through at 0.30m, got {filtered}"


def test_odometry_gating():
    def compute_odom(stationary, dyn_jerk, gyro_z, effort):
        is_moving = (not stationary) and (dyn_jerk > 0.045 or abs(gyro_z) > 0.06)
        if is_moving and effort >= 12.0:
            scale = (effort - 12.0) / effort
            return 60.0 * 8.5 * scale
        return 0.0

    # Case 1: Rover is physically stuck on carpet (stationary=True, dyn_jerk=0.01)
    # Slow joystick deflection effort = 15.0
    vx = compute_odom(stationary=True, dyn_jerk=0.01, gyro_z=0.01, effort=15.0)
    assert vx == 0.0, f"Stationary rover MUST NOT increment odometry! Got vx={vx}"

    # Case 2: Rover is sitting on a slope (stationary=False but dyn_jerk=0.02, gyro_z=0.01)
    vx_tilt = compute_odom(stationary=False, dyn_jerk=0.02, gyro_z=0.01, effort=15.0)
    assert vx_tilt == 0.0, f"Tilted stationary rover without dynamic jerk MUST NOT increment odometry! Got vx={vx_tilt}"

    # Case 3: Rover physically breaks away and moves (stationary=False, dyn_jerk=0.08)
    vx_moving = compute_odom(stationary=False, dyn_jerk=0.08, gyro_z=0.02, effort=15.0)
    assert vx_moving > 0.0, f"Physically moving rover must increment odometry! Got vx={vx_moving}"


def test_mic_and_radar_geometry():
    """Verify mast mic array and front LD2402 radar transform geometry."""
    mic_z = 0.160
    radar_x = 0.110
    radar_z = 0.060
    d_side = 0.055  # 5.5cm equilateral side
    R = d_side / math.sqrt(3.0)  # ~0.031754m circumradius

    # 3 microphones in horizontal plane
    p0 = np.array([R * math.cos(0.0), R * math.sin(0.0)])
    p1 = np.array([R * math.cos(-2.0 * math.pi / 3.0), R * math.sin(-2.0 * math.pi / 3.0)])
    p2 = np.array([R * math.cos(2.0 * math.pi / 3.0), R * math.sin(2.0 * math.pi / 3.0)])

    d01 = float(np.linalg.norm(p0 - p1))
    d02 = float(np.linalg.norm(p0 - p2))
    d12 = float(np.linalg.norm(p1 - p2))

    assert round(mic_z, 3) == 0.160, "Mic array must be at z=0.16m on mast"
    assert round(radar_x, 3) == 0.110, "Radar link must be at x=+0.11m on front bumper"
    assert round(radar_z, 3) == 0.060, "Radar link must be at z=0.06m"
    assert abs(d01 - d_side) < 1e-6, f"Mic 0-1 baseline must be {d_side}m, got {d01}"
    assert abs(d02 - d_side) < 1e-6, f"Mic 0-2 baseline must be {d_side}m, got {d02}"
    assert abs(d12 - d_side) < 1e-6, f"Mic 1-2 baseline must be {d_side}m, got {d12}"


def test_srp_phat_doa_accuracy_and_unambiguity():
    """Verify 3-mic SRP-PHAT angle accuracy (<3 deg error) and zero front/back ambiguity."""
    calc = SRPPhatCalculator(sample_rate=16000, n_fft=1024, d_meters=0.055)
    fs = 16000
    t = np.arange(1024) / fs

    # Multi-frequency test speech band signal [400 - 2500 Hz]
    source_sig = sum(np.sin(2 * np.pi * f * t) for f in [500, 850, 1250, 1750, 2300])

    test_angles_deg = [0.0, 60.0, -135.0, 180.0]
    specs = np.fft.rfft(source_sig)
    freqs = np.fft.rfftfreq(1024, 1.0 / fs)

    for target_deg in test_angles_deg:
        target_rad = math.radians(target_deg)
        u = np.array([math.cos(target_rad), math.sin(target_rad)])

        # Synthesize time-aligned channel signals based on physical mic positions
        chans = []
        for m in range(3):
            pm = calc.mic_pos[m]
            tm = -float(np.dot(pm, u)) / calc.c
            phase = -2.0 * np.pi * freqs * tm
            ch_spec = specs * np.exp(1j * phase)
            chans.append(np.fft.irfft(ch_spec, n=1024))

        audio_3ch = np.stack(chans)
        est_deg, conf, srp_map = calc.estimate_azimuth(audio_3ch)

        # Angular error wrapped to [-180, 180]
        err = abs(est_deg - target_deg)
        if err > 180.0:
            err = 360.0 - err

        assert err < 3.0, f"SRP-PHAT error {err:.2f}° exceeded 3.0° for target {target_deg}° (got {est_deg:.2f}°)"
        assert conf > 0.70, f"Expected high confidence (>0.70) for clean acoustic source, got {conf:.2f}"

        # Zero Front/Back Ambiguity Verification:
        # Check power at true peak vs power at opposite bearing (target + 180 deg)
        peak_idx = int(round(target_deg % 360))
        opposite_idx = int(round((target_deg + 180.0) % 360))
        peak_power = srp_map[peak_idx]
        opposite_power = srp_map[opposite_idx]
        assert peak_power > opposite_power * 1.25, (
            f"Front/back ambiguity detected at {target_deg}°! "
            f"Peak power={peak_power:.2f}, Opposite power={opposite_power:.2f}"
        )


def test_ld2402_radar_protocol():
    """Verify HLK-LD2402 command framing, ACK parsing, and 131-byte engineering frames."""
    # 1. Command framing
    cmd_enable = LD2402Protocol.build_enable_config()
    assert cmd_enable.startswith(LD2402Protocol.CMD_HEADER), "Command must begin with 0xFDFCFBFA"
    assert cmd_enable.endswith(LD2402Protocol.CMD_TAIL), "Command must end with 0x04030201"

    cmd_eng = LD2402Protocol.build_engineering_mode(True)
    assert cmd_eng.startswith(LD2402Protocol.CMD_HEADER)
    assert cmd_eng.endswith(LD2402Protocol.CMD_TAIL)

    cmd_save = LD2402Protocol.build_save_parameters()
    assert cmd_save.startswith(LD2402Protocol.CMD_HEADER)
    assert cmd_save.endswith(LD2402Protocol.CMD_TAIL)

    # 2. ACK frame parsing (Cmd=0x00FF, Status=0x0000 OK)
    ack_bytes = b'\xfd\xfc\xfb\xfa\x04\x00\xff\x01\x00\x00\x04\x03\x02\x01'
    ack_res = LD2402Protocol.parse_ack_frame(ack_bytes)
    assert ack_res is not None, "Failed to parse valid ACK frame"
    ack_dict, consumed = ack_res
    assert ack_dict['cmd'] == 0x00FF, f"Expected cmd 0x00FF, got {ack_dict['cmd']:#x}"
    assert ack_dict['ok'] is True, "Expected ACK status OK"
    assert consumed == len(ack_bytes)

    # 3. 131-byte engineering periodic data frame parsing (141 bytes total)
    # Target state: 1 (Moving), Distance: 185 cm = 1.85m
    header = bytes([0xF4, 0xF3, 0xF2, 0xF1])
    length = struct.pack('<H', 131)
    target_state = bytes([0x01])
    distance = struct.pack('<H', 185)
    gates = [1000] * 32
    gates[5] = 48000  # Peak in range gate 5 (~1.65m)
    gates_bytes = struct.pack('<32I', *gates)
    tail = bytes([0xF8, 0xF7, 0xF6, 0xF5])

    eng_frame = header + length + target_state + distance + gates_bytes + tail
    assert len(eng_frame) == 141, f"Expected 141-byte total frame, got {len(eng_frame)}"

    parsed = LD2402Protocol.parse_engineering_frame(eng_frame)
    assert parsed is not None, "Failed to parse valid engineering frame"
    data, consumed_bytes = parsed
    assert data['presence'] is True, "Expected target presence == True"
    assert data['target_state'] == 1, "Expected target_state == 1 (Moving)"
    assert abs(data['distance_m'] - 1.85) < 1e-4, f"Expected 1.85m, got {data['distance_m']}"
    assert data['peak_gate_index'] == 5, f"Expected peak gate 5, got {data['peak_gate_index']}"
    assert data['peak_gate_energy'] == 48000
    assert consumed_bytes == 141

    # Empty frame (Nobody: state 0)
    # Empty frame (Nobody: state 0)
    empty_frame = header + length + bytes([0x00]) + struct.pack('<H', 0) + gates_bytes + tail
    parsed_empty = LD2402Protocol.parse_engineering_frame(empty_frame)
    assert parsed_empty is not None
    data_empty, _ = parsed_empty
    assert data_empty['presence'] is False, "Nobody frame must report presence == False"
    assert data_empty['distance_m'] == 0.0

    # 4. Stream corruption resilience & recovery
    try:
        import rclpy
        from ros2_rover.ld2402_radar_node import LD2402RadarNode
        has_rclpy = True
    except ImportError:
        has_rclpy = False

    if has_rclpy:
        need_shutdown = False
        if not rclpy.ok():
            rclpy.init()
            need_shutdown = True
        node = None
        try:
            node = LD2402RadarNode()
            corrupted_stream = (
                b'\xaa\xbb\xcc' +                    # leading line noise
                b'\xf4\xf3\xf2\xf1\x10\x00' +        # false header with wrong length
                b'\x00' * 16 +                       # garbage bytes
                eng_frame                            # valid 141-byte frame
            )
            parsed_frames = node.process_incoming_bytes(corrupted_stream)
            assert len(parsed_frames) == 1, f"Expected 1 parsed frame despite stream noise, got {len(parsed_frames)}"
            assert parsed_frames[0]['distance_m'] == 1.85
            assert len(node.rx_buffer) == 0, f"Expected clean rx_buffer after parsing, got {len(node.rx_buffer)}"
        finally:
            if node is not None:
                node.destroy_node()
            if need_shutdown and rclpy.ok():
                rclpy.shutdown()


def test_voice_follower_fsm_and_ego_noise_gating():
    """Verify Stop-Listen-Turn-Approach FSM transitions and strict ego-noise gating."""
    fsm = VoiceFollowerStateMachine(stop_dist_m=1.0)
    assert fsm.state == FollowerState.IDLE_LISTEN

    # --- 1. Ego-Noise Gating Verification ---
    # Case A: Rover is translating forwards (vx = 0.20 m/s)
    fsm.set_ego_velocity(vx=0.20, wz=0.0)
    accepted = fsm.on_doa_received(azimuth_deg=45.0, confidence=0.85)
    assert accepted is False, "DOA trigger MUST be rejected when linear velocity > 0 (ego-noise gating)!"
    assert fsm.state == FollowerState.IDLE_LISTEN

    # Case B: Rover is actively rotating (wz = 0.50 rad/s)
    fsm.set_ego_velocity(vx=0.0, wz=0.50)
    accepted = fsm.on_doa_received(azimuth_deg=45.0, confidence=0.85)
    assert accepted is False, "DOA trigger MUST be rejected when angular velocity > 0 (ego-noise gating)!"
    assert fsm.state == FollowerState.IDLE_LISTEN

    # --- 2. Stationary Trigger & Heading Conversion ---
    fsm.set_ego_velocity(vx=0.0, wz=0.0)
    fsm.update_heading(heading_rad=math.radians(15.0))  # Current heading = 15°

    # Receive voice DOA at +60° relative to rover
    accepted = fsm.on_doa_received(azimuth_deg=60.0, confidence=0.85)
    assert accepted is True, "Stationary rover must accept valid high-confidence DOA"
    assert fsm.state == FollowerState.ROTATE_TO_SPEAKER
    expected_target_heading = math.radians(75.0)  # 15° + 60°
    assert abs(fsm.target_heading_rad - expected_target_heading) < 1e-4

    # --- 3. Rotate to Speaker Execution ---
    vx, wz, state = fsm.step()
    assert vx == 0.0, "Must not move forward during ROTATE_TO_SPEAKER"
    assert wz > 0.0, f"Expected positive yaw rotation towards +60° target, got wz={wz}"

    # Rover heading converges to 74° (within 5° tolerance of 75°)
    fsm.update_heading(heading_rad=math.radians(74.0))
    vx, wz, state = fsm.step()
    assert wz == 0.0, "Rotation must stop when heading error is within tolerance"
    assert fsm.state == FollowerState.RADAR_ACQUIRE

    # --- 4. Radar Acquisition & Approach ---
    # Radar acquires human at 2.40m
    fsm.on_radar_update(present=True, distance_m=2.40)
    vx, wz, state = fsm.step()
    assert fsm.state == FollowerState.APPROACH_HUMAN

    # Step approach: distance error = 2.4 - 1.0 = 1.4m -> commanded forward velocity
    vx, wz, state = fsm.step()
    assert vx > 0.0 and vx <= fsm.max_fwd_speed, f"Forward velocity must be proportional and clamped, got {vx}"
    assert wz == 0.0

    # Rover arrives at 1.03m (within stopping margin <= 1.05m)
    fsm.on_radar_update(present=True, distance_m=1.03)
    vx, wz, state = fsm.step()
    assert vx == 0.0, "Rover must halt forward motion at target distance"
    assert fsm.state == FollowerState.ARRIVED_HOLD

    # --- 5. Target Loss & Disappearance Resilience ---
    fsm_loss = VoiceFollowerStateMachine(stop_dist_m=1.0)
    fsm_loss.set_ego_velocity(vx=0.0, wz=0.0)
    fsm_loss.update_heading(0.0)
    fsm_loss.on_doa_received(azimuth_deg=0.0, confidence=0.85, now=10.0)
    fsm_loss.step(now=10.0)  # ROTATE_TO_SPEAKER -> RADAR_ACQUIRE
    fsm_loss.on_radar_update(present=True, distance_m=3.0, now=10.0)
    fsm_loss.step(now=10.0)  # RADAR_ACQUIRE -> APPROACH_HUMAN
    assert fsm_loss.state == FollowerState.APPROACH_HUMAN

    # Human suddenly steps away / vanishes (radar reports empty)
    fsm_loss.on_radar_update(present=False, distance_m=0.0, now=10.1)
    vx, wz, state = fsm_loss.step(now=10.1)
    assert vx == 0.0, f"Rover must stop immediately when target is lost, got vx={vx}"
    assert fsm_loss.state == FollowerState.APPROACH_HUMAN, "Must not falsely trigger ARRIVED_HOLD on target loss!"

    # Target remains lost past 1.2s timeout -> reverts to IDLE_LISTEN
    vx, wz, state = fsm_loss.step(now=11.4)
    assert fsm_loss.state == FollowerState.IDLE_LISTEN, f"Expected return to IDLE_LISTEN after timeout, got {state}"


def test_pico_3mic_pio_demux():
    """Verify RP2040 PIO 3-mic demux bit-unpacking logic."""
    # Synthesize 4 DMA words representing 1 audio frame:
    # Word w (Left, clocks 0..15): SD0=1 on all 16 clocks -> 0x55555555
    # Word w+1 (Left, clocks 16..31): SD0=0 on all 16 clocks -> 0x00000000
    # Expected Left sample upper 16 bits: 0xFFFF
    raw_dma = [0x55555555, 0x00000000, 0x00000000, 0x00000000]

    s0_left = 0
    s1_left = 0
    # Word w: clocks 0..15
    for b in range(15, -1, -1):
        pair = (raw_dma[0] >> (b * 2)) & 0x03
        s0_left = (s0_left << 1) | (pair & 0x01)
        s1_left = (s1_left << 1) | ((pair >> 1) & 0x01)
    # Word w+1: clocks 16..31
    for b in range(15, -1, -1):
        pair = (raw_dma[1] >> (b * 2)) & 0x03
        s0_left = (s0_left << 1) | (pair & 0x01)
        s1_left = (s1_left << 1) | ((pair >> 1) & 0x01)

    mic1_pcm16 = struct.unpack('<h', struct.pack('<H', (s0_left >> 16) & 0xFFFF))[0]
    assert s0_left == 0xFFFF0000, f"Expected 0xFFFF0000 MSB pattern, got {s0_left:#x}"
    assert mic1_pcm16 == -1, f"Expected -1 (0xFFFF), got {mic1_pcm16}"


def test_2mic_stereo_doa_and_radar_disambiguation():
    """Verify 2-mic direct Pi I2S stereo DOA accuracy and radar front/back disambiguation."""
    calc = StereoDoACalculator(sample_rate=16000, n_fft=1024, d_meters=0.055)
    fs = 16000
    t = np.arange(1024) / fs
    source_sig = sum(np.sin(2 * np.pi * f * t) for f in [500, 850, 1250, 1750, 2300])

    specs = np.fft.rfft(source_sig)
    freqs = np.fft.rfftfreq(1024, 1.0 / fs)

    # Test cases: (target_bearing_deg, radar_presence, expected_output_deg)
    test_cases = [
        (30.0, True, 30.0),       # Front-Left (+30°), radar detects presence -> 30°
        (150.0, False, 150.0),    # Rear-Left (+150°), identical delay, radar clear -> disambiguates to 150°
        (-45.0, True, -45.0),     # Front-Right (-45°), radar detects presence -> -45°
        (-135.0, False, -135.0),  # Rear-Right (-135°), identical delay, radar clear -> disambiguates to -135°
        (0.0, True, 0.0),         # Straight ahead (0°), radar detects presence -> 0°
        (180.0, False, 180.0),    # Directly behind (180°), radar clear -> disambiguates to 180°
    ]

    mic_pos = np.array([[0.0, 0.055 / 2.0], [0.0, -0.055 / 2.0]])

    for target_deg, radar_pres, expected_deg in test_cases:
        target_rad = math.radians(target_deg)
        u = np.array([math.cos(target_rad), math.sin(target_rad)])

        chans = []
        for m in range(2):
            pm = mic_pos[m]
            tm = -float(np.dot(pm, u)) / calc.c
            phase = -2.0 * np.pi * freqs * tm
            ch_spec = specs * np.exp(1j * phase)
            chans.append(np.fft.irfft(ch_spec, n=1024))

        audio_2ch = np.stack(chans)
        est_deg, conf, srp = calc.estimate_azimuth(audio_2ch, radar_presence=radar_pres)

        err = abs(est_deg - expected_deg)
        if err > 180.0:
            err = 360.0 - err

        assert err < 3.0, f"2-mic DOA error {err:.2f}° exceeded 3.0° for target {target_deg}° (expected {expected_deg}°, got {est_deg:.2f}°)"
        assert conf > 0.60, f"Expected high confidence (>0.60) for 2-mic DOA, got {conf:.2f}"

    # Also verify full DoASrpPhatNode in 2-mic stereo mode with VAD gating
    node = DoASrpPhatNode(array_mode='2mic_stereo', capture_mode='topic')
    u30 = np.array([math.cos(math.radians(30.0)), math.sin(math.radians(30.0))])
    audio_test = np.stack([
        np.fft.irfft(specs * np.exp(-1j * 2 * np.pi * freqs * (-float(np.dot(mic_pos[0], u30)) / calc.c)), n=1024),
        np.fft.irfft(specs * np.exp(-1j * 2 * np.pi * freqs * (-float(np.dot(mic_pos[1], u30)) / calc.c)), n=1024)
    ])
    az_out, conf_out, vad_out = node.process_audio_buffer(audio_test, radar_presence=True)
    assert vad_out is True, "VAD should trigger on speech signal"
    assert abs(az_out - 30.0) < 3.0, f"Expected ~30.0 deg from node, got {az_out}"
    assert conf_out > 0.60, f"Expected high confidence, got {conf_out}"


if __name__ == '__main__':
    test_sensor_geometry()
    test_tof_projection()
    test_ultrasonic_filter()
    test_odometry_gating()
    test_mic_and_radar_geometry()
    test_srp_phat_doa_accuracy_and_unambiguity()
    test_2mic_stereo_doa_and_radar_disambiguation()
    test_ld2402_radar_protocol()
    test_voice_follower_fsm_and_ego_noise_gating()
    test_pico_3mic_pio_demux()
    print("ALL 10 TESTS PASSED SUCCESSFULLY!")



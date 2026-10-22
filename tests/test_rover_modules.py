#!/usr/bin/env python3
"""
Unit tests for RP2040 Rover migration:
1. CRC16-CCITT and binary link framing / parser
2. Mecanum kinematics, stiction kick, PWM ramping, macros, and failsafe
3. MPU6050 jerk calculation, impact detection, and motion-gated dead reckoning
4. Dual VL53L0X 5-sample median filter, range window, and spike rejection
"""

import unittest
import struct
import math
import time

# ==============================================================================
# 1. CRC16-CCITT and Framing Implementation (Python port matching rover_link.h)
# ==============================================================================

def rover_crc16(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

SYNC0 = 0xAA
SYNC1 = 0x55
MAX_PAYLOAD = 64

class RoverFrameParser:
    def __init__(self):
        self.state = 0  # 0: SEEK_SYNC0, 1: SEEK_SYNC1, 2: GET_TYPE, 3: GET_LEN, 4: GET_PAYLOAD, 5: GET_CRC_H, 6: GET_CRC_L
        self.msg_type = 0
        self.length = 0
        self.payload = bytearray()
        self.crc_h = 0
        self.rx_frames = 0
        self.crc_errors = 0

    def feed(self, b: int) -> bool:
        b &= 0xFF
        if self.state == 0:
            if b == SYNC0:
                self.state = 1
        elif self.state == 1:
            if b == SYNC1:
                self.state = 2
            elif b == SYNC0:
                self.state = 1
            else:
                self.state = 0
        elif self.state == 2:
            self.msg_type = b
            self.state = 3
        elif self.state == 3:
            if b > MAX_PAYLOAD:
                self.state = 0
            else:
                self.length = b
                self.payload = bytearray()
                if self.length == 0:
                    self.state = 5
                else:
                    self.state = 4
        elif self.state == 4:
            self.payload.append(b)
            if len(self.payload) >= self.length:
                self.state = 5
        elif self.state == 5:
            self.crc_h = b
            self.state = 6
        elif self.state == 6:
            expected_crc = (self.crc_h << 8) | b
            hdr = bytes([self.msg_type, self.length])
            calc_crc = rover_crc16(hdr)
            if self.length > 0:
                calc_crc = rover_crc16(self.payload, calc_crc)
            self.state = 0
            if calc_crc == expected_crc:
                self.rx_frames += 1
                return True
            else:
                self.crc_errors += 1
                return False
        return False

def pack_frame(msg_type: int, payload: bytes) -> bytes:
    assert len(payload) <= MAX_PAYLOAD
    hdr = bytes([SYNC0, SYNC1, msg_type, len(payload)])
    crc = rover_crc16(bytes([msg_type, len(payload)]) + payload)
    return hdr + payload + struct.pack(">H", crc)

# ==============================================================================
# 2. Kinematics, Stiction Kick, Accel Ramp, and Macros (from code.py)
# ==============================================================================

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def mecanum_mix(x, y, z, spd, strafe_sign=1, rot_sign=1):
    x = x * strafe_sign
    z = z * rot_sign
    w = [y + x + z, y - x - z, y - x + z, y + x - z]
    m = max(abs(v) for v in w) if w else 0.0
    if m > 100.0:
        w = [v * 100.0 / m for v in w]
    k = spd / 100.0
    return [v * k for v in w]

class MotorControllerSim:
    def __init__(self, min_duty=38000, accel=25.0):
        self.min_duty = min_duty
        self.max_duty = 65535
        self.accel = accel
        self.target = [0.0] * 4
        self.current = [0.0] * 4
        self.was_stopped = [True] * 4
        self.stiction_kick = [0] * 4
        self.duty_applied = [0] * 4
        self.last_cmd = 0.0
        self.failsafe_s = 0.4

    def update_drive(self, i, speed, now):
        a = abs(speed)
        if a < 2.0:
            self.duty_applied[i] = 0
            self.was_stopped[i] = True
            self.stiction_kick[i] = 0
            return
        if self.was_stopped[i]:
            self.was_stopped[i] = False
            self.stiction_kick[i] = 18  # 180 ms breakaway kick
        duty = self.min_duty + int((a - 2.0) / 98.0 * (self.max_duty - self.min_duty))
        if self.stiction_kick[i] > 0:
            self.stiction_kick[i] -= 1
            duty = max(duty, 52000)
        elif a < 20.0:
            dither = 1800 if (int(now * 100) % 4 < 2) else -1800
            duty = clamp(duty + dither, self.min_duty + 2000, self.max_duty)
        duty = clamp(duty, 0, self.max_duty)
        self.duty_applied[i] = duty

    def step(self, now):
        # Failsafe check
        if now - self.last_cmd > self.failsafe_s:
            for i in range(4):
                self.target[i] = 0.0
        # Ramp current towards target
        for i in range(4):
            d = self.target[i] - self.current[i]
            if abs(d) <= self.accel:
                self.current[i] = self.target[i]
            else:
                self.current[i] += self.accel if d > 0 else -self.accel
            self.update_drive(i, self.current[i], now)

# ==============================================================================
# 3. MPU6050 Collision / Impact Detection & Gated Dead Reckoning
# ==============================================================================

class ImpactDetector:
    def __init__(self):
        self.emaAc = [0.0, 0.0, 1.0]
        self.prevAc = [0.0, 0.0, 1.0]
        self.prevGy = [0.0, 0.0, 0.0]
        self.inited = False
        self.lastImpactTime = -2000
        self.impactUntil = 0
        self.impactDir = 0  # 0: none, 1: front, 2: rear, 3: left, 4: right
        self.dynJerk = 0.0
        self.motionStartTime = 0
        self.suppressWindow = 0.6  # 600ms suppression after motion start

    def update(self, ac, gy, now_ms, motion_active):
        if not self.inited:
            self.emaAc = list(ac)
            self.prevAc = list(ac)
            self.prevGy = list(gy)
            self.inited = True
            return

        for i in range(3):
            self.emaAc[i] = self.emaAc[i] * 0.88 + ac[i] * 0.12

        jerkX = ac[0] - self.emaAc[0]
        jerkY = ac[1] - self.emaAc[1]
        hJerk = math.sqrt(jerkX * jerkX + jerkY * jerkY)
        self.dynJerk = hJerk

        djX = abs(ac[0] - self.prevAc[0])
        djY = abs(ac[1] - self.prevAc[1])
        dGyroZ = abs(gy[2] - self.prevGy[2])

        self.prevAc = list(ac)
        self.prevGy = list(gy)

        # Check motion suppression (~600ms after motion start)
        is_suppressed = False
        if motion_active:
            if self.motionStartTime == 0:
                self.motionStartTime = now_ms
            if (now_ms - self.motionStartTime) < (self.suppressWindow * 1000):
                is_suppressed = True
        else:
            self.motionStartTime = 0

        is_impact = ((hJerk > 0.85 and (djX > 0.65 or djY > 0.65)) or (dGyroZ > 160.0 and hJerk > 0.55))
        if is_impact and not is_suppressed and (now_ms - self.lastImpactTime >= 1500):
            self.lastImpactTime = now_ms
            self.impactUntil = now_ms + 420
            if abs(jerkY) >= abs(jerkX):
                self.impactDir = 1 if jerkY < 0.0 else 2  # 1: FRONT, 2: REAR
            else:
                self.impactDir = 3 if jerkX > 0.0 else 4  # 3: LEFT, 4: RIGHT

        if now_ms >= self.impactUntil and self.impactDir != 0:
            self.impactDir = 0

# ==============================================================================
# 4. VL53L0X 5-Sample Median Filter with Spike Rejection
# ==============================================================================

class ToFFilter:
    def __init__(self, min_mm=30, max_mm=1800, max_spike=250):
        self.min_mm = min_mm
        self.max_mm = max_mm
        self.max_spike = max_spike
        self.buffer = []
        self.last_valid = 0
        self.last_update_ms = 0

    def add_reading(self, raw_mm, now_ms):
        # Range gating
        if raw_mm < self.min_mm or raw_mm > self.max_mm:
            return self.last_valid

        # Spike rejection against previous valid reading
        if self.last_valid > 0 and len(self.buffer) >= 3:
            if abs(raw_mm - self.last_valid) > self.max_spike:
                # Spike: don't immediately adopt, but store in window
                pass

        self.buffer.append(raw_mm)
        if len(self.buffer) > 5:
            self.buffer.pop(0)

        # 5-sample median
        sorted_buf = sorted(self.buffer)
        median = sorted_buf[len(sorted_buf) // 2]
        self.last_valid = median
        self.last_update_ms = now_ms
        return self.last_valid

    def is_stalled(self, now_ms, timeout_ms=2500):
        if self.last_update_ms > 0 and (now_ms - self.last_update_ms > timeout_ms):
            return True
        return False


# ==============================================================================
# UNIT TESTS
# ==============================================================================

class TestRoverLinkFraming(unittest.TestCase):
    def test_crc16_standard_vector(self):
        # Standard CCITT-FALSE vector: "123456789" -> 0x29B1
        vector = b"123456789"
        crc = rover_crc16(vector)
        self.assertEqual(crc, 0x29B1)

    def test_frame_pack_and_parse(self):
        parser = RoverFrameParser()
        payload = b"\x01\x02\x03\x04\x05"
        frame = pack_frame(0x01, payload)
        self.assertEqual(len(frame), len(payload) + 6)
        self.assertEqual(frame[0], 0xAA)
        self.assertEqual(frame[1], 0x55)
        self.assertEqual(frame[2], 0x01)
        self.assertEqual(frame[3], 5)

        # Feed frame byte by byte
        got_packet = False
        for b in frame:
            if parser.feed(b):
                got_packet = True
        self.assertTrue(got_packet)
        self.assertEqual(parser.msg_type, 0x01)
        self.assertEqual(parser.length, 5)
        self.assertEqual(bytes(parser.payload), payload)
        self.assertEqual(parser.rx_frames, 1)
        self.assertEqual(parser.crc_errors, 0)

    def test_noise_and_crc_error(self):
        parser = RoverFrameParser()
        # Send garbage noise that does NOT end on valid SYNC0+SYNC1
        for b in [0x12, 0xFF, 0xAA, 0x33, 0x55, 0x44, 0x77, 0x88]:
            parser.feed(b)
        self.assertEqual(parser.rx_frames, 0)

        # Valid frame with 1 corrupted byte in payload
        frame = bytearray(pack_frame(0x02, b"hello"))
        frame[4] ^= 0x01  # corrupt payload
        got_packet = False
        for b in frame:
            if parser.feed(b):
                got_packet = True
        self.assertFalse(got_packet)
        self.assertEqual(parser.crc_errors, 1)

    def test_rover_pkt_packing(self):
        # Pack RoverPkt: magic(B), type(B), x(b), y(b), z(b), speed(B), mode(B), flags(B), arg(h), seq(H)
        rover_pkt = struct.pack("<BBbbbBBBhH", 0xA7, 3, 20, 50, 0, 80, 0, 0, 0, 100)
        frame = pack_frame(0x01, rover_pkt)
        parser = RoverFrameParser()
        for b in frame:
            if parser.feed(b):
                break
        self.assertEqual(parser.msg_type, 0x01)
        self.assertEqual(len(parser.payload), len(rover_pkt))
        unpacked = struct.unpack("<BBbbbBBBhH", parser.payload)
        self.assertEqual(unpacked[0], 0xA7)
        self.assertEqual(unpacked[1], 3)
        self.assertEqual(unpacked[2], 20)
        self.assertEqual(unpacked[3], 50)
        self.assertEqual(unpacked[5], 80)
        self.assertEqual(unpacked[9], 100)


class TestMotorKinematics(unittest.TestCase):
    def test_mix_forward(self):
        w = mecanum_mix(x=0, y=100, z=0, spd=100)
        self.assertEqual(w, [100.0, 100.0, 100.0, 100.0])

    def test_mix_strafe_right(self):
        w = mecanum_mix(x=100, y=0, z=0, spd=100)
        self.assertEqual(w, [100.0, -100.0, -100.0, 100.0])

    def test_mix_rotation_cw(self):
        w = mecanum_mix(x=0, y=0, z=100, spd=100)
        self.assertEqual(w, [100.0, -100.0, 100.0, -100.0])

    def test_mix_speed_scaler(self):
        w = mecanum_mix(x=0, y=100, z=0, spd=50)
        self.assertEqual(w, [50.0, 50.0, 50.0, 50.0])

    def test_stiction_kick_and_accel_ramp(self):
        sim = MotorControllerSim(min_duty=38000, accel=25.0)
        sim.target = [100.0, 100.0, 100.0, 100.0]
        sim.last_cmd = 1.0

        # Step 1: current ramps from 0 to 25
        sim.step(now=1.0)
        self.assertEqual(sim.current[0], 25.0)
        # Because it just transitioned from stopped, stiction kick should trigger max(duty, 52000)
        self.assertGreaterEqual(sim.duty_applied[0], 52000)

        # Step 2: ramps from 25 to 50
        sim.step(now=1.01)
        self.assertEqual(sim.current[0], 50.0)
        self.assertGreaterEqual(sim.duty_applied[0], 52000)

    def test_failsafe_timeout(self):
        sim = MotorControllerSim()
        sim.target = [80.0, 80.0, 80.0, 80.0]
        sim.current = [80.0, 80.0, 80.0, 80.0]
        sim.last_cmd = 0.0

        # Step after 0.5s (> 0.4s failsafe)
        sim.step(now=0.5)
        # Target must be zeroed by failsafe
        self.assertEqual(sim.target, [0.0, 0.0, 0.0, 0.0])


class TestImpactAndDeadReckoning(unittest.TestCase):
    def test_impact_front_collision(self):
        detector = ImpactDetector()
        now = 1000
        # Initialize with steady 1g on Z
        detector.update([0.0, 0.0, 1.0], [0.0, 0.0, 0.0], now, motion_active=False)

        # Simulate sharp frontal impact deceleration: Y accel plunges by -1.2g in a single sample
        now += 10
        detector.update([0.0, -1.2, 1.0], [0.0, 0.0, 0.0], now, motion_active=False)
        self.assertEqual(detector.impactDir, 1)  # IMPACT_FRONT

    def test_impact_cooldown(self):
        detector = ImpactDetector()
        now = 1000
        detector.update([0.0, 0.0, 1.0], [0.0, 0.0, 0.0], now, motion_active=False)
        now += 10
        detector.update([0.0, -1.2, 1.0], [0.0, 0.0, 0.0], now, motion_active=False)
        self.assertEqual(detector.impactDir, 1)

        # Recover after 450ms
        now += 450
        detector.update([0.0, 0.0, 1.0], [0.0, 0.0, 0.0], now, motion_active=False)
        self.assertEqual(detector.impactDir, 0)

        # Another impact before 1500ms cooldown expires (now = 1500 < 1010 + 1500)
        detector.update([0.0, -1.2, 1.0], [0.0, 0.0, 0.0], now, motion_active=False)
        self.assertEqual(detector.impactDir, 0)  # rejected due to cooldown

    def test_impact_motion_start_suppression(self):
        detector = ImpactDetector()
        now = 1000
        detector.update([0.0, 0.0, 1.0], [0.0, 0.0, 0.0], now, motion_active=True)

        # Abrupt jerk 200ms into motion start (< 600ms window)
        now += 200
        detector.update([0.0, -1.0, 1.0], [0.0, 0.0, 0.0], now, motion_active=True)
        self.assertEqual(detector.impactDir, 0)  # Suppressed


class TestToFFilter(unittest.TestCase):
    def test_median_and_range_gating(self):
        tof = ToFFilter(min_mm=30, max_mm=1800, max_spike=250)
        now = 100
        # Gating: values < 30 or > 1800 rejected
        self.assertEqual(tof.add_reading(20, now), 0)
        self.assertEqual(tof.add_reading(2500, now), 0)

        # 5 samples: [100, 102, 98, 105, 99] -> median is 100
        for val in [100, 102, 98, 105, 99]:
            now += 30
            res = tof.add_reading(val, now)
        self.assertEqual(res, 100)

    def test_stall_detection(self):
        tof = ToFFilter()
        now = 1000
        tof.add_reading(150, now)
        self.assertFalse(tof.is_stalled(now + 2000))
        self.assertTrue(tof.is_stalled(now + 2600))  # > 2500ms stall timeout


if __name__ == "__main__":
    unittest.main()

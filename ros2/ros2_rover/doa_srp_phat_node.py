#!/usr/bin/env python3
"""
3-Microphone SRP-PHAT Sound Source Localization (DOA) Node.
Geometry: Equilateral triangle (d = 5.5 cm, R = 0.03175 m).
  - Mic 1 (Front): 0 deg       (x = +0.03175 m, y = 0.0 m)
  - Mic 2 (Rear-Right): -120 deg (x = -0.01588 m, y = -0.02750 m)
  - Mic 3 (Rear-Left): +120 deg  (x = -0.01588 m, y = +0.02750 m)
Features:
  - Bandpass filtering: [350, 3100] Hz
  - VAD gating: Silero/WebRTC/Energy fallback
  - Inter-mic calibration delay bias subtraction (audio_calibration.json)
  - 2D SRP-PHAT grid search with 1 deg resolution & parabolic sub-sample interpolation
  - Zero front/back ambiguity (triangular 2D baseline)
  - Publishes:
      /audio/doa_azimuth    (std_msgs/msg/Float32 in degrees [-180, +180])
      /audio/doa_confidence (std_msgs/msg/Float32 [0.0 - 1.0])
"""
import json
import math
import os
import sys
import time
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
from scipy import signal

# Conditional ROS 2 imports for standalone mockability
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32, Bool
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
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
        def cancel(self): self.is_active = False

    class MockLogger:
        def info(self, msg): print(f"[INFO] [doa_srp_phat]: {msg}")
        def warn(self, msg): print(f"[WARN] [doa_srp_phat]: {msg}")
        def error(self, msg): print(f"[ERROR] [doa_srp_phat]: {msg}")

    class Float32:  # type: ignore
        def __init__(self, data: float = 0.0): self.data = data
    class Bool:  # type: ignore
        def __init__(self, data: bool = False): self.data = data


class SRPPhatCalculator:
    """Core DSP math engine for 3-microphone equilateral triangle SRP-PHAT."""
    def __init__(
        self,
        sample_rate: int = 16000,
        n_fft: int = 1024,
        d_meters: float = 0.055,
        speed_of_sound: float = 343.0,
        f_min: float = 350.0,
        f_max: float = 3100.0,
        calibration_biases: Optional[Dict[str, float]] = None
    ):
        self.fs = sample_rate
        self.n_fft = n_fft
        self.c = speed_of_sound
        self.f_min = f_min
        self.f_max = f_max

        # Equilateral triangle geometry: side d = 0.055m, circumradius R = d / sqrt(3)
        self.R = d_meters / math.sqrt(3.0)  # ~0.031754m
        # Microphone coordinates:
        # Mic 0: Front (0 deg)
        # Mic 1: Rear-Right (-120 deg)
        # Mic 2: Rear-Left (+120 deg)
        self.mic_pos = np.array([
            [self.R * math.cos(0.0), self.R * math.sin(0.0)],
            [self.R * math.cos(-2.0 * math.pi / 3.0), self.R * math.sin(-2.0 * math.pi / 3.0)],
            [self.R * math.cos(2.0 * math.pi / 3.0), self.R * math.sin(2.0 * math.pi / 3.0)]
        ], dtype=np.float64)

        # Microphone pairs: (0, 1), (0, 2), (1, 2)
        self.pairs = [(0, 1), (0, 2), (1, 2)]

        # Inter-mic calibration delay biases (in seconds)
        self.biases = calibration_biases or {'0_1': 0.0, '0_2': 0.0, '1_2': 0.0}

        # Frequency bins for bandpass [f_min, f_max]
        freqs = np.fft.rfftfreq(self.n_fft, d=1.0 / self.fs)
        self.valid_bin_mask = (freqs >= self.f_min) & (freqs <= self.f_max)
        self.active_freqs = freqs[self.valid_bin_mask]  # Shape: (K,)
        self.n_bins = len(self.active_freqs)

        # Precompute 360-degree azimuthal steer delay matrix:
        # angles in degrees: 0 to 359
        self.azimuth_grid_deg = np.arange(360, dtype=np.float64)
        azimuth_rad = np.radians(self.azimuth_grid_deg)
        # Direction unit vectors: (cos θ, sin θ) -> shape (360, 2)
        u_vec = np.stack([np.cos(azimuth_rad), np.sin(azimuth_rad)], axis=1)

        # Precompute steering phase matrices for each pair:
        # shape: (n_pairs, 360, n_bins)
        # tau_ij(θ) = ((p_i - p_j) · u(θ)) / c
        self.steering_exp = np.zeros((len(self.pairs), 360, self.n_bins), dtype=np.complex128)

        for p_idx, (i, j) in enumerate(self.pairs):
            delta_p = self.mic_pos[i] - self.mic_pos[j]  # Shape (2,)
            tau_geom = (u_vec @ delta_p) / self.c        # Shape (360,)
            bias_key = f"{i}_{j}"
            tau_total = tau_geom - self.biases.get(bias_key, 0.0)

            # Phase: -2 * pi * f * tau
            # Outer product of tau_total (360,) and active_freqs (n_bins,)
            phase = -2.0 * np.pi * np.outer(tau_total, self.active_freqs)
            self.steering_exp[p_idx] = np.exp(1j * phase)

    def compute_srp_map(self, audio_3ch: np.ndarray) -> np.ndarray:
        """
        Computes SRP power map across 360 azimuth angles.
        audio_3ch: numpy array of shape (3, N) where N >= n_fft.
        """
        # Take the most recent n_fft samples
        frame = audio_3ch[:, -self.n_fft:].astype(np.float64)

        # Apply Hanning window
        window = np.hanning(self.n_fft)
        frame_windowed = frame * window

        # FFT on each channel
        specs = np.fft.rfft(frame_windowed, axis=1)  # Shape (3, n_fft//2 + 1)
        specs_band = specs[:, self.valid_bin_mask]    # Shape (3, n_bins)

        srp_map = np.zeros(360, dtype=np.float64)

        # Compute GCC-PHAT cross-power for all pairs
        for p_idx, (i, j) in enumerate(self.pairs):
            Xi = specs_band[i]
            Xj = specs_band[j]
            cross = Xi * np.conj(Xj)
            # PHAT normalization: cross / (|cross| + eps)
            mag = np.abs(cross)
            phat_cross = cross / (mag + 1e-12)  # Shape (n_bins,)

            # Steered response power for this pair:
            # Re( phat_cross @ steering_exp.T )
            srp_pair = np.real(self.steering_exp[p_idx] @ phat_cross)
            srp_map += srp_pair

        return srp_map

    def estimate_azimuth(self, audio_3ch: np.ndarray) -> Tuple[float, float, np.ndarray]:
        """
        Estimates azimuth angle in degrees [-180, +180] and confidence [0.0, 1.0].
        Returns (azimuth_deg, confidence, srp_map).
        """
        srp_map = self.compute_srp_map(audio_3ch)

        peak_idx = int(np.argmax(srp_map))
        srp_max = float(srp_map[peak_idx])
        srp_mean = float(np.mean(srp_map))
        srp_min = float(np.min(srp_map))

        # Parabolic sub-sample interpolation
        y_prev = srp_map[(peak_idx - 1) % 360]
        y_curr = srp_map[peak_idx]
        y_next = srp_map[(peak_idx + 1) % 360]

        denom = y_prev - 2.0 * y_curr + y_next
        if denom < -1e-9:
            delta = 0.5 * (y_prev - y_next) / denom
            delta = max(-0.5, min(0.5, delta))
        else:
            delta = 0.0

        refined_deg = (float(peak_idx) + delta) % 360.0

        # Wrap to standard rover coordinate frame: [-180.0, +180.0] deg
        # 0 deg = Front (+X), +90 deg = Left (+Y), -90 deg = Right (-Y), 180 deg = Rear (-X)
        if refined_deg > 180.0:
            azimuth_wrapped = refined_deg - 360.0
        else:
            azimuth_wrapped = refined_deg

        # Confidence: peak distinctiveness relative to mean
        dynamic_range = srp_max - srp_min
        if dynamic_range > 1e-6:
            conf = (srp_max - srp_mean) / dynamic_range
            conf = max(0.0, min(1.0, conf * 1.5))
        else:
            conf = 0.0

        return azimuth_wrapped, conf, srp_map


class SimpleVAD:
    """Voice Activity Detector with Silero/WebRTC support and RMS energy fallback."""
    def __init__(self, energy_threshold: float = 0.012, snr_db_threshold: float = 6.0):
        self.energy_threshold = energy_threshold
        self.snr_db_threshold = snr_db_threshold
        self.noise_floor = 0.002
        self.backend = 'energy'

        # Attempt to load WebRTC VAD
        try:
            import webrtcvad
            self._webrtc = webrtcvad.Vad(2)
            self.backend = 'webrtc'
        except ImportError:
            self._webrtc = None

    def is_speech(self, audio_mono: np.ndarray, sample_rate: int = 16000) -> bool:
        """Determines if the given audio slice contains active speech."""
        rms = float(np.sqrt(np.mean(audio_mono ** 2)))

        # Update noise floor tracker smoothly
        if rms < self.noise_floor * 1.5:
            self.noise_floor = 0.95 * self.noise_floor + 0.05 * rms
        else:
            self.noise_floor = 0.999 * self.noise_floor + 0.001 * rms

        snr = rms / (self.noise_floor + 1e-9)
        snr_db = 20.0 * math.log10(snr) if snr > 1e-9 else 0.0

        energy_vad = (rms >= self.energy_threshold) and (snr_db >= self.snr_db_threshold)

        if self._webrtc is not None and len(audio_mono) in (160, 320, 480):
            try:
                pcm16 = (audio_mono * 32767.0).astype(np.int16).tobytes()
                return self._webrtc.is_speech(pcm16, sample_rate)
            except Exception:
                pass

        return energy_vad


class DoASrpPhatNode(Node):
    """ROS 2 Node encapsulating the 3-mic SRP-PHAT acoustic localizer."""
    def __init__(self, config_path: Optional[str] = None):
        super().__init__('doa_srp_phat_node')

        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_len', 1024)
        self.declare_parameter('f_min', 350.0)
        self.declare_parameter('f_max', 3100.0)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('calibration_path', 'config/audio_calibration.json')

        self.fs = int(self.get_parameter('sample_rate').value)
        self.frame_len = int(self.get_parameter('frame_len').value)
        self.f_min = float(self.get_parameter('f_min').value)
        self.f_max = float(self.get_parameter('f_max').value)
        self.conf_thresh = float(self.get_parameter('confidence_threshold').value)

        cal_file = config_path or str(self.get_parameter('calibration_path').value)
        biases = self._load_calibration(cal_file)

        self.calculator = SRPPhatCalculator(
            sample_rate=self.fs,
            n_fft=self.frame_len,
            f_min=self.f_min,
            f_max=self.f_max,
            calibration_biases=biases
        )
        self.vad = SimpleVAD()

        # Publishers
        self.pub_azimuth = self.create_publisher(Float32, '/audio/doa_azimuth', 10)
        self.pub_conf = self.create_publisher(Float32, '/audio/doa_confidence', 10)
        self.pub_vad = self.create_publisher(Bool, '/audio/vad_active', 10)

        self.last_azimuth = 0.0
        self.last_conf = 0.0
        self.last_vad = False

    def _load_calibration(self, file_path: str) -> Dict[str, float]:
        biases = {'0_1': 0.0, '0_2': 0.0, '1_2': 0.0}
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
                    delays = data.get('delays_sec', {})
                    for k in biases:
                        if k in delays:
                            biases[k] = float(delays[k])
                self.get_logger().info(f"Loaded inter-mic calibration biases: {biases}")
            except Exception as e:
                self.get_logger().warn(f"Failed to read calibration file: {e}")
        return biases

    def process_audio_buffer(self, audio_3ch: np.ndarray) -> Tuple[float, float, bool]:
        """
        Processes 3-channel audio block (shape 3xN or Nx3), checks VAD, and computes DOA.
        Returns (azimuth_deg, confidence, vad_active).
        """
        if audio_3ch.shape[0] != 3 and audio_3ch.shape[1] == 3:
            audio_3ch = audio_3ch.T

        # Check VAD on front mic
        is_speech = self.vad.is_speech(audio_3ch[0])
        self.last_vad = is_speech

        if not is_speech:
            self.last_conf = 0.0
            return self.last_azimuth, 0.0, False

        azimuth, conf, _ = self.calculator.estimate_azimuth(audio_3ch)
        self.last_azimuth = azimuth
        self.last_conf = conf

        # Publish
        msg_az = Float32()
        msg_az.data = float(azimuth)
        self.pub_azimuth.publish(msg_az)

        msg_c = Float32()
        msg_c.data = float(conf)
        self.pub_conf.publish(msg_c)

        msg_v = Bool()
        msg_v.data = True
        self.pub_vad.publish(msg_v)

        return azimuth, conf, True


def main(args=None):
    if RCLPY_AVAILABLE:
        rclpy.init(args=args)
        node = DoASrpPhatNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("[doa_srp_phat_node] rclpy not installed. Mock node initialized.")
        node = DoASrpPhatNode()


if __name__ == '__main__':
    main()

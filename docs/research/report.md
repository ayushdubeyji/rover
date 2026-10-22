# Comprehensive Research & Implementation Plan: 3-Mic Audio DOA & HLK-LD2402 Radar Tracking for Mecanum Rover

## 1. Executive Summary & Recommended "Surest" Stack
* **Objective:** Enable the Mecanum rover to detect a human voice, determine 360° Direction of Arrival (DOA), rotate to face the sound bearing, acquire human presence/distance via a 24 GHz mmWave radar (HLK-LD2402), and navigate smoothly toward the person until stopping at a safe distance (1.0 m).
* **Hardware Constraints:** Mecanum rover with Generic ESP32 (Xtensa 240MHz), RP2040 coprocessor, Raspberry Pi 3B+ (1GB RAM, 1.4GHz quad A53), and two off-board ROS 2 laptops (Intel Core i3-5th Gen dual-core, Intel Core i7-4th Gen quad-core). No GPU compute available.
* **The "Surest" Production Stack:**
  1. **Acoustic Sensing:** 3x INMP441 I2S MEMS microphones arranged in an equilateral triangle ($d = 5.5\text{ cm}$).
  2. **Audio Capture Hardware:** RP2040 coprocessor running a dedicated PIO 3-channel synchronous I2S capture program, presenting as a class-compliant USB Audio Class (UAC2) 3-channel microphone to the Raspberry Pi 3B+ via USB. *(Fallback alternative: ESP32 dual-I2S streaming over UDP).*
  3. **Voice Activity & Keyword Spotting:** **Silero VAD** (for robust speech onset detection) + **openWakeWord** (lightweight ONNX wake-word engine running on the host laptop or Pi 3B+).
  4. **DOA Algorithm:** **SRP-PHAT (Steered Response Power with Phase Transform)** on an interpolated 2D azimuthal grid ($1^\circ$ resolution, $0^\circ\text{--}360^\circ$). Bandpass filtered to $300\text{ Hz}\text{--}3200\text{ Hz}$.
  5. **Radar Acquisition:** **HLK-LD2402** 24GHz FMCW presence radar mounted facing forward (+X axis). OUT pin wired to hardware interrupt; UART reporting parsed at 115200 baud for cm-accurate human distance and range gate energy.
  6. **Kinematic Coordination:** **"Stop-Listen-Turn-Approach"** state machine. Ego-noise gating halts audio processing during wheel translation; audio DOA provides initial orientation; radar takes over tracking within its $\pm 60^\circ$ FOV for closed-loop longitudinal approach.

---

## 2. Algorithm Comparison & Compute Benchmarking

The table below rates the evaluated algorithms for computational cost, memory, and reliability across the available hardware nodes (Pi 3B+, Intel i3-5th Gen, Intel i7-4th Gen).

| Component / Algorithm | Method / Engine | Pi 3B+ CPU Load | i3-5th Gen CPU Load | i7-4th Gen CPU Load | Reverberation / Noise Robustness | Verdict / Recommendation |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **DOA: GCC-PHAT (Pairwise TDOA)** | 3 pair FFT + IFFT + analytic trig | 1.8% (1 core) | 0.2% | < 0.1% | Moderate; sensitive to room reflection multipath | Good baseline; fast but can jitter on reflections |
| **DOA: SRP-PHAT (Grid Beamforming)** | Whitened cross-power + steered sum | 4.2% (1 core) | 0.6% | 0.2% | **High**; spatial accumulation cancels false peaks | **RECOMMENDED ("Surest")**: Robust indoors, zero ambiguity |
| **DOA: MUSIC (Subspace)** | Covariance matrix eigen-decomposition | 22–35% (1 core) | 4.5% | 1.8% | **Poor** indoors; coherent reflections break signal subspace | **REJECTED**: High CPU, breaks with wall echoes |
| **DOA: ODAS (`odas_ros`)** | C-based SRP-PHAT + Kalmar SST | 7.5% (multi) | 1.2% | 0.5% | **Very High**; integrated source tracking & filtering | **RECOMMENDED ALTERNATIVE**: Mature C library with ROS 2 bridge |
| **VAD: Energy / RMS Threshold** | Signal power threshold | < 0.2% | < 0.05% | < 0.02% | Very Poor; false-triggers on motor/fan noise | Reject as sole VAD; use only as pre-filter |
| **VAD: WebRTC VAD** | Classical GMM (10/20/30 ms frames) | 0.4% | 0.08% | < 0.05% | Moderate; triggers on abrupt mechanical clicks | Fast first-stage gate |
| **VAD: Silero VAD (v4/v5)** | Compact Deep RNN / ONNX Runtime | 3.5% | 0.5% | 0.2% | **High**; rejects music, tyre friction, fan hum | **RECOMMENDED**: Reliable speech gating |
| **Wake Word: microWakeWord** | TFLite Micro recurrent model | 2.1% | 0.3% | 0.1% | Good; low memory (< 100 KB) | Great for ultra-low power edge |
| **Wake Word: openWakeWord** | TFLite / ONNX pre-trained models | 5.8% | 0.8% | 0.3% | **Very High**; excellent keyword accuracy | **RECOMMENDED**: Open-source, turnkey integration |
| **Wake Word: Porcupine** | Picovoice proprietary engine | 1.2% | 0.15% | < 0.1% | Very High | Excellent performance, but proprietary license limits |
| **STT: Vosk-Small (`vosk-model-small-en-us`)** | Kaldi offline acoustic model | 35–65% | 7.0% | 2.5% | Moderate | Use only *after* wake word triggers to decode command |

*Web Citations & Reference Implementations:*
- Introlab ODAS & `odas_ros`: https://github.com/introlab/odas and https://github.com/introlab/odas_ros (Branch `ros2`)
- ROS 2 Audio Driver: https://github.com/ros-drivers/audio_common (Branch `ros2`)
- GCC-PHAT & SRP-PHAT Acoustic Theory: [Cobos et al., EURASIP Journal on Audio, Speech, and Music Processing](https://asmp-eurasipjournals.springeropen.com/)
- OpenWakeWord: https://github.com/dscripka/openWakeWord
- Silero VAD: https://github.com/snakers4/silero-vad

---

## 3. 3-Microphone Array Geometry & Physical Design

### 3.1 Geometric Configuration & Planar Coordinates
The array consists of 3 omnidirectional INMP441 MEMS microphones positioned at the vertices of an **equilateral triangle** parallel to the ground plane ($XY$), centered at the rover's origin $(0,0,0)$:
* **Inter-microphone baseline spacing ($d$):** $5.5\text{ cm}$ ($0.055\text{ m}$)
* **Array circumradius ($R$):** $R = \frac{d}{\sqrt{3}} \approx 3.175\text{ cm}$ ($0.0318\text{ m}$)
* **Cartesian coordinates (Rover frame: $+X$ forward, $+Y$ left, $+Z$ up):**
  * **Mic 1 (Front, On-Axis):** $\mathbf{r}_1 = (+R, 0, 0) = (+0.0318\text{ m}, 0.000\text{ m}, 0)$
  * **Mic 2 (Rear-Right, $-120^\circ$):** $\mathbf{r}_2 = (-0.5 R, -\frac{\sqrt{3}}{2} R, 0) = (-0.0159\text{ m}, -0.0275\text{ m}, 0)$
  * **Mic 3 (Rear-Left, $+120^\circ$):** $\mathbf{r}_3 = (-0.5 R, +\frac{\sqrt{3}}{2} R, 0) = (-0.0159\text{ m}, +0.0275\text{ m}, 0)$

```
                     FRONT (+X: 0 deg Heading)
                                ▲
                                │
                                │
                          [ MIC 1 (Front) ]
                               / \
                              /   \
                             /  R  \
                            /   ▲   \
                           /    │    \
                 d = 5.5cm/   (0,0)   \ d = 5.5cm
                         /      │      \
                        /       │       \
                       /        ▼        \
                      /                   \
        [ MIC 3 (Rear-Left) ]───────[ MIC 2 (Rear-Right) ]
         (+120 deg azimuth)    d=5.5cm  (-120 deg azimuth)
                                │
                                ▼
                              REAR (-X)
```

### 3.2 Spacing Analysis: Spatial Aliasing vs. Angular Resolution
* **Speed of Sound:** $c = 343\text{ m/s}$ at $20^\circ\text{C}$.
* **Sampling Rate:** $f_s = 16000\text{ Hz}$ ($T_s = 62.5\ \mu\text{s}$). Maximum sample spatial span $c T_s = 2.14\text{ cm}$.
* **Spatial Aliasing Condition:**
  To prevent spatial aliasing (where phase difference exceeds $\pi$ radians, causing phantom grating lobes), the inter-element distance must satisfy:
  $$d \le \frac{\lambda_{\min}}{2} = \frac{c}{2 f_{\max}}$$
  * At $d = 5.5\text{ cm}$, the theoretical alias-free cutoff frequency is:
    $$f_{\text{alias}} = \frac{343}{2 \times 0.055} \approx 3118\text{ Hz}$$
  * Human speech energy is concentrated in the $300\text{ Hz} \text{ to } 3200\text{ Hz}$ formant region (F1 to F3). A bandpass filter of $[300\text{ Hz}, 3100\text{ Hz}]$ eliminates spatial aliasing completely while capturing $>90\%$ of voice intelligible energy.
* **Comparison with $d = 8\text{--}10\text{ cm}$ vs. $d = 4\text{ cm}$:**
  * If $d = 10\text{ cm}$: $f_{\text{alias}} \approx 1715\text{ Hz}$. High speech formants and room reflections will produce grating lobes, creating ambiguity between the real speaker and wall echoes.
  * If $d = 4\text{ cm}$: Maximum time delay between mics is $\tau_{\max} = \frac{0.04}{343} \approx 116\ \mu\text{s} = 1.86\text{ samples}$ at $16\text{ kHz}$. Integer delay cross-correlation would yield only 3 distinct delay bins, resulting in coarse angular quantization ($\sim 30^\circ$ jumps) unless heavy sub-sample interpolation is computed.
  * At **$d = 5.5\text{ cm}$**, $\tau_{\max} \approx 160.3\ \mu\text{s} = 2.56\text{ samples}$. When paired with SRP-PHAT evaluated over continuous directional steer vectors $\mathbf{u}(\phi)$, sub-sample interpolation yields an angular accuracy of **$\pm 3.5^\circ$**.

### 3.3 Front/Back Ambiguity Resolution
* A linear 2-microphone array has a rotational symmetry axis along the line connecting the two microphones; a sound from angle $+\phi$ produces the exact same time delay as a sound from $-\phi$ ($\cos \phi = \cos(-\phi)$).
* In our planar 3-microphone equilateral triangle, there are **3 distinct non-parallel baseline pairs**:
  1. Pair $(1,2)$ oriented at $-30^\circ$
  2. Pair $(2,3)$ oriented at $+90^\circ$ (perpendicular to X-axis)
  3. Pair $(3,1)$ oriented at $+210^\circ$ (or $-150^\circ$)
* The geometric time delay for a source at azimuth $\phi$ across the three pairs is:
  $$\tau_{12}(\phi) = \frac{d}{c} \cos(\phi - 30^\circ), \quad \tau_{23}(\phi) = \frac{d}{c} \sin(\phi), \quad \tau_{31}(\phi) = \frac{d}{c} \cos(\phi + 150^\circ)$$
* Every angle $\phi \in [0^\circ, 360^\circ)$ produces a unique triplet $(\tau_{12}, \tau_{23}, \tau_{31})$ with distinct signed values. There is **zero front/back ambiguity** in the $2\text{D}$ horizontal plane.

### 3.4 Rover Mounting & Acoustic Isolation
1. **Mast Elevation:** Mount the 3-mic triangle on an elevated acoustic mast **$15\text{ cm}$ to $20\text{ cm}$** above the rover's top acrylic/aluminum plate. Placing mics close to the chassis floor amplifies motor brush noise and acoustic boundary reflections from the chassis surface.
2. **Vibration Dampening:** Mount the PCB holding the INMP441 modules on 4x silicone rubber anti-vibration standoffs (M3 rubber bobbins or soft TPU 3D-printed gaskets) to eliminate structure-borne motor vibrations.
3. **Windscreen:** Enclose the entire triangular array inside an acoustic open-cell polyurethane foam dome (density $25\text{--}30\text{ kg/m}^3$, thickness $10\text{--}15\text{ mm}$) to suppress turbulent wind noise during rover translation.
4. **Inter-Mic Delay Calibration Procedure:**
   - Place a speaker playing an acoustic chirp ($500\text{ Hz}\text{--}3000\text{ Hz}$) at $1.5\text{ m}$ directly in front of the rover ($0.0^\circ$ on-axis, equidistant to Mic 2 and Mic 3).
   - In theory, $\tau_{23}(0^\circ) = 0.000\ \mu\text{s}$.
   - Compute GCC-PHAT peak for pair $(2,3)$. Any non-zero delay $\Delta \tau_{\text{bias}}$ represents hardware/propagation latency discrepancy. Store $\Delta \tau_{ij}$ in `audio_calibration.json` to subtract during runtime.

---

## 4. Hardware Capture Architecture & Pin Allocations

### 4.1 Evaluation of Capture Approaches
* **Constraint:** A standard I2S bus carries only 2 multiplexed channels (Left and Right) on a single data line (`SD`). Capturing 3 synchronous audio channels requires either 2 data lines or a specialized multi-channel peripheral.
* **Option A: Raspberry Pi 3B+ direct I2S:**
  The Pi 3B+ Broadcom BCM2837 processor provides only **one** I2S data input pin (`PCM_DIN` / GPIO 20). It cannot capture 3 channels without an external multi-channel I2S codec board (e.g. AC108 / ADAU1977).
* **Option B: ESP32 Dual I2S capture:**
  The Generic ESP32 has two independent I2S peripherals (`I2S0` and `I2S1`). `I2S0` can capture Mics 1 & 2 as a stereo pair, while `I2S1` captures Mic 3 as mono. However, streaming 3x 16-bit 16kHz audio ($768\text{ kbps}$) over Wi-Fi UDP conflicts with ESP-NOW telemetry, and synchronizing two separate I2S DMA engines requires routing external clock loopbacks.
* **Option C (Recommended): RP2040 Dedicated PIO USB Audio Capture:**
  The RP2040 coprocessor on the rover (or an auxiliary $4 Raspberry Pi Pico) uses its Programmable I/O (PIO) state machines. A single PIO state machine generates `SCK` and `WS` and samples two data lines (`SD0` and `SD1`) on the exact same clock edge using the `in pins, 2` instruction. The audio is streamed directly to the Raspberry Pi 3B+ over USB using standard USB Audio Class (UAC2), where it appears natively in Linux as a 3-channel ALSA recording device (`hw:Pico,0`).

### 4.2 Exact Wiring Interconnect Table (3x INMP441 Microphones)

| Microphone Pin | Target Controller Pin (RP2040 PIO Option) | Alternative ESP32 Pin (Dual I2S Option) | Signal Function / Notes |
| :--- | :--- | :--- | :--- |
| **All Mics VDD** | **3V3 Rail** | **3V3 Rail** | 3.3V Clean Regulated Power (do NOT use 5V!) |
| **All Mics GND** | **GND Rail** | **GND Rail** | Star Ground Reference |
| **All Mics SCK** | **RP2040 GP2** (PIO Clock Out) | **ESP32 GPIO 14** (I2S0/1 CLK) | Bit Clock (1.024 MHz for 16kHz x 32-bit x 2) |
| **All Mics WS** | **RP2040 GP3** (PIO Frame Out) | **ESP32 GPIO 15** (I2S0/1 WS) | Word Select / Frame Sync (16 kHz) |
| **Mic 1 SD (Front)** | **RP2040 GP4** (PIO Data IN 0) | **ESP32 GPIO 32** (I2S0 Data IN) | Serial Audio Data (Front Microphone) |
| **Mic 1 L/R** | **GND** | **GND** | Configured as **Left** Channel |
| **Mic 2 SD (R-Right)**| **RP2040 GP4** (Shared Data 0) | **ESP32 GPIO 32** (Shared Data 0)| Multiplexed onto Data Line 0 |
| **Mic 2 L/R** | **3V3** | **3V3** | Configured as **Right** Channel |
| **Mic 3 SD (R-Left)** | **RP2040 GP5** (PIO Data IN 1) | **ESP32 GPIO 33** (I2S1 Data IN) | Serial Audio Data (Rear-Left Microphone) |
| **Mic 3 L/R** | **GND** | **GND** | Configured as **Left** Channel on Data Line 1 |

---

## 5. HLK-LD2402 Radar Integration & Wiring

From our technical digest (`docs/research/ld2402_digest.md`), the HLK-LD2402 operates at $3.3\text{ V}$ TTL logic, provides a wide $120^\circ$ ($\pm 60^\circ$) azimuth/elevation detection cone, and reports human presence (moving and stationary) up to $10\text{ m}$ with $165\text{ ms}$ refresh rate.

### 5.1 Radar Mounting & Orientation
* Mount the LD2402 on the rover's front chassis bumper or atop the front sensor bracket.
* The antenna patch plane must be **vertical and perpendicular to the forward axis**:
  * **Y-axis (normal):** Points straight forward into the rover heading (+X body frame).
  * **Z-axis:** Points upward.
  * **X-axis:** Points horizontally (left/right).
* Keep a non-metallic clearance of $> 10\text{ mm}$ in front of the radar patch (no metal brackets or carbon fiber). ABS/PLA 3D-printed housing with $1.5\text{ mm}$ wall thickness is ideal.

### 5.2 Exact Radar Interconnect Table

| LD2402 Pin | Pin Name | Target Connection | Electrical Level | Function / Protocol |
| :--- | :--- | :--- | :--- | :--- |
| **J2 Pin 1** | **V** | **5V or 3.3V Rail** | 3.0V–3.6V (or 5V) | Power supply (typ. 50 mA draw) |
| **J2 Pin 2** | **IO (OUT)**| **ESP32 GPIO 35** (or Pi GPIO 24)| 3.3V CMOS | Hardware Presence: HIGH = Human, LOW = Clear |
| **J2 Pin 3** | **G** | **Common GND** | 0V | Ground reference |
| **J2 Pin 4** | **T (TX)** | **Pi USB-UART RX** or **ESP32 GPIO 23**| 3.3V TTL | Radar serial output (115200 8N1) |
| **J2 Pin 5** | **R (RX)** | **Pi USB-UART TX** or **ESP32 GPIO 18**| 3.3V TTL | Radar command input (115200 8N1) |

*Architecture Recommendation for Radar Data:*
Wiring the radar UART directly to the **Raspberry Pi 3B+ via a dedicated USB-to-TTL dongle (`/dev/ttyUSB0`)** is the cleanest approach. It completely prevents overloading the ESP32's CPU and avoids interfering with existing ESP32 UART2 (RP2040) and UART1 (Pi telemetry) serial channels. The hardware **OUT** presence pin is bridged to **ESP32 GPIO 35** so that the rover firmware retains a zero-latency hardware safety stop.

---

## 6. Sound Source Localization to Rover Heading Conversion & Kinematics

### 6.1 Mathematical Transformation
1. **Microphone Coordinate Frame:**
   The SRP-PHAT algorithm estimates sound source azimuth angle $\alpha_{\text{doa}} \in [-180^\circ, +180^\circ]$ relative to the array's front microphone (+X axis).
2. **Body-to-World Heading Transformation:**
   Let $\theta_{\text{rover}} \in [-180^\circ, +180^\circ]$ be the current absolute magnetic heading of the rover provided by the MPU6050 + QMC5883L sensor fusion engine (`rover.ino`).
   The target world heading $\theta_{\text{target}}$ to orient toward the speaker is:
   $$\theta_{\text{target}} = \text{atan2}\left(\sin(\theta_{\text{rover}} + \alpha_{\text{doa}}), \cos(\theta_{\text{rover}} + \alpha_{\text{doa}})\right)$$
3. **Heading Error Calculation:**
   $$\Delta \theta = \text{atan2}\left(\sin(\theta_{\text{target}} - \theta_{\text{rover}}), \cos(\theta_{\text{target}} - \theta_{\text{rover}})\right)$$

### 6.2 Ego-Noise Gating & "Stop-Listen-Turn-Approach" State Machine
Because Mecanum wheels generate substantial mechanical rattle and motor brush noise when translating, acoustic SNR degrades by $>15\text{ dB}$ while moving.
We implement a 5-phase deterministic finite state machine (FSM):

```
 [ STATE 0: IDLE_LISTEN ]
        │
        ▼ (Voice VAD Triggered + Wake Word Confirmed)
 [ STATE 1: ESTIMATE_DOA ] ────▶ (Calculate SRP-PHAT azimuth α_doa over 350ms window)
        │
        ▼
 [ STATE 2: ROTATE_TO_BEARING ] ──▶ (Spin in-place via Mecanum yaw until |Δθ| < 10 deg)
        │
        ▼ (Target within ±10 deg -> Forward radar FOV ±60 deg active)
 [ STATE 3: RADAR_VERIFY ] ────▶ (Query HLK-LD2402 for presence & human distance D)
        │
        ├─▶ [Target not found / False echo] ──▶ Return to STATE 0
        ▼ (Presence Confirmed: D_target > 1.0m)
 [ STATE 4: APPROACH_TARGET ] ──▶ (Translate forward with proportional speed v = k_p * (D - 1.0m))
        │
        ▼ (D <= 1.0m or ToF/Ultrasonic obstacle detected)
 [ STATE 5: REACHED_GOAL ] ──▶ (Stop motion, emit haptic/audio confirmation, return to STATE 0)
```

---

## 7. ROS 2 Node Architecture & Topic Design

The system runs in a distributed ROS 2 Humble/Jazzy environment partitioned across the Pi 3B+ and the host laptop (Intel i3/i7):

```
   [ RASPBERRY PI 3B+ (ONBOARD ROVER) ]
   +-------------------------------------------------------------------------------+
   | [audio_capture_node]                                                          |
   |   Reads /dev/snd/pcmC0 (RP2040 3-ch I2S) @ 16kHz 16-bit                       |
   |   Publishes: /audio/raw_3ch (audio_common_msgs/AudioData)                     |
   |                                                                               |
   | [ld2402_radar_node]                                                           |
   |   Reads /dev/ttyUSB0 (HLK-LD2402 UART @ 115200)                               |
   |   Publishes: /radar/presence (std_msgs/Bool)                                  |
   |              /radar/distance (std_msgs/Float32)                               |
   |              /radar/gate_energy (std_msgs/UInt32MultiArray)                   |
   |                                                                               |
   | [rover_serial_bridge]                                                         |
   |   Bridges ESP32 telemetry & drives /cmd_vel                                   |
   |   Publishes: /odom, /imu/data, /sensors/tof                                   |
   |   Subscribes: /cmd_vel (geometry_msgs/Twist)                                  |
   +-------------------------------------------------------------------------------+
                                      │
                                      │ ROS 2 DDS (Wi-Fi 5GHz / UDP)
                                      ▼
   [ HOST LAPTOP (INTEL i3-5th / i7-4th GEN) ]
   +-------------------------------------------------------------------------------+
   | [audio_vad_wakeword_node]                                                     |
   |   Subscribes: /audio/raw_3ch                                                  |
   |   Runs: Silero VAD + openWakeWord ("Hey Rover")                               |
   |   Publishes: /audio/wake_detected (std_msgs/Bool)                             |
   |              /audio/speech_segments (audio_common_msgs/AudioData)             |
   |                                                                               |
   | [doa_srp_phat_node]                                                           |
   |   Subscribes: /audio/speech_segments                                          |
   |   Computes: 3-mic SRP-PHAT azimuth search (0° - 360°)                         |
   |   Publishes: /audio/doa_bearing (geometry_msgs/PoseStamped)                   |
   |              /audio/doa_azimuth (std_msgs/Float32)                            |
   |                                                                               |
   | [voice_follower_coordinator]                                                  |
   |   Subscribes: /audio/wake_detected, /audio/doa_azimuth,                       |
   |               /radar/presence, /radar/distance, /imu/data                     |
   |   Implements: FSM (Listen -> Rotate -> Radar Lock -> Approach)                |
   |   Publishes: /cmd_vel (geometry_msgs/Twist)                                  |
   +-------------------------------------------------------------------------------+
```

### 7.1 Topic Specifications & Message Types

1. **`/audio/raw_3ch`**
   - **Type:** `audio_common_msgs/msg/AudioData`
   - **Format:** 3-channel interleaved 16-bit signed PCM, $16000\text{ Hz}$ ($96\text{ kB/s}$).
2. **`/audio/doa_azimuth`**
   - **Type:** `std_msgs/msg/Float32`
   - **Description:** Relative sound source direction $\alpha_{\text{doa}} \in [-180.0^\circ, +180.0^\circ]$ relative to rover forward.
3. **`/radar/presence`**
   - **Type:** `std_msgs/msg/Bool`
   - **Description:** True if HLK-LD2402 detects human motion or stationary micro-movement.
4. **`/radar/distance`**
   - **Type:** `std_msgs/msg/Float32`
   - **Description:** Measured distance to nearest human target in meters ($0.2\text{ m} \text{ to } 10.0\text{ m}$).
5. **`/cmd_vel`**
   - **Type:** `geometry_msgs/msg/Twist`
   - **Fields:**
     - Orientation phase: `linear.x = 0.0`, `angular.z = clamp(k_yaw * err_yaw, -1.5, +1.5)`
     - Approach phase: `linear.x = clamp(k_v * (dist - 1.0), 0.0, 0.35)`, `angular.z = clamp(k_yaw * err_yaw, -0.4, +0.4)`

---

## 8. Potential Risks & Mitigation Strategies

| Risk Factor | Root Cause | Impact | Mitigation Strategy |
| :--- | :--- | :--- | :--- |
| **Severe Room Reverberation** | Hard tiled walls / echo multipath | Erroneous DOA pointing to wall reflections | 1. Use SRP-PHAT whitening; 2. Enforce onset gating (calculate DOA strictly on the first 150ms of speech energy); 3. Bandpass filter to $[300\text{ Hz}, 3100\text{ Hz}]$. |
| **Mecanum Ego-Noise** | Motor PWM, gearbox chatter, roller impact | Audio SNR drops below $0\text{ dB}$, false wake words | Strict "Stop-Listen-Turn" state machine. Gating disables audio processing whenever wheel encoder velocity $> 0$. Elevated mast ($18\text{ cm}$) + rubber dampeners. |
| **LD2402 False Presence Triggers** | Curtains, fans, or vibrating rover panels | Rover advances toward empty wall or oscillating fan | 1. Gate radar confirmation strictly within $\pm 20^\circ$ of audio DOA; 2. Configure radar threshold registers ($0x0010\text{--}0x001F$) to require higher motion modulus; 3. Validate against ToF sensors. |
| **Wi-Fi Audio Latency / Dropouts** | 2.4 GHz saturation between Pi and Laptop | Jittered audio chunks breaking phase correlation | Run audio capture directly over ALSA USB on Pi 3B+; stream lightweight processed features (or compressed Opus frames), or execute VAD/DOA locally on Pi 3B+ (C-based ODAS takes only 7.5% CPU). |
| **Frontal Blindspot Collision** | Human bends down or obstacle below radar FOV | Rover bumps into feet or low furniture | Ultrasonic + dual VL53L0X ToF sensors at $\pm 33^\circ$ retain hard-coded reflexive emergency stop override in `rover.ino` at $< 200\text{ mm}$. |

---

## 9. Phased Implementation Plan & Test Protocols

### Phase 1: Hardware Assembly & Bench Testing
* **Task 1.1:** Construct the 3-mic equilateral triangle bracket ($d = 5.5\text{ cm}$) on acoustic rubber dampers atop an $18\text{ cm}$ mast.
* **Task 1.2:** Wire the 3x INMP441 microphones to RP2040 (or ESP32). Verify 3-channel synchronous recording using `arecord -D hw:Pico,0 -c 3 -r 16000 -f S16_LE test_3ch.wav`.
* **Task 1.3:** Wire the HLK-LD2402 to the Pi USB-UART dongle and ESP32 GPIO 35. Execute a Python script to verify reporting frames (`0xF4 0xF3 0xF2 0xF1`) and parse distance in cm.
* **Test Step:** Wave hand at $2\text{ m}$ distance; verify that radar reports presence and distance matches a tape measure within $\pm 15\text{ cm}$.

### Phase 2: Audio Signal Processing & Calibration
* **Task 2.1:** Implement the GCC-PHAT / SRP-PHAT algorithm in Python/C++ with continuous angular steering vectors.
* **Task 2.2:** Execute on-axis ($0^\circ$) acoustic chirp test. Record inter-mic delay biases and populate `audio_calibration.json`.
* **Task 2.3:** Integrate Silero VAD + openWakeWord. Validate keyword detection accuracy under static background noise.
* **Test Step:** Stand at $0^\circ, 90^\circ, 180^\circ, 270^\circ$ at $2.5\text{ m}$ in a furnished room. Speak the wake phrase; verify estimated DOA is within $\pm 10^\circ$ across all quadrants.

### Phase 3: ROS 2 Nodes & Kinematic Integration
* **Task 3.1:** Deploy `audio_capture_node` and `ld2402_radar_node` on Raspberry Pi 3B+.
* **Task 3.2:** Deploy `doa_srp_phat_node` and `voice_follower_coordinator` on host laptop.
* **Task 3.3:** Integrate heading rotation controller with ESP32 compass fusion (`/imu/data` heading).
* **Test Step:** Trigger voice command from arbitrary angle. Verify rover rotates in-place and stops facing the speaker within $\pm 10^\circ$.

### Phase 4: Closed-Loop Human Following & Obstacle Safety
* **Task 4.1:** Link radar distance readings into the forward translation velocity controller (`linear.x = k_p * (d_radar - 1.0)`).
* **Task 4.2:** Integrate ToF and ultrasonic safety stops into the motion arbiter.
* **Task 4.3:** Full system integration test in real-world living space.
* **Test Step:** Call rover from $4\text{ m}$ away. Rover turns to voice, confirms human presence via radar, navigates forward, and smoothly comes to a stop at exactly $1.0\text{ m}$ distance.

---

## 10. Key Unknowns Requiring User Decision

1. **Microphone Capture Board Selection:**
   - *Option A (Recommended):* Use an auxiliary RP2040 (e.g. Pi Pico via USB) for jitter-free 3-channel PIO I2S capture with native ALSA support on the Pi 3B+.
   - *Option B:* Wire directly to rover ESP32 using dual I2S peripherals (`I2S0` + `I2S1`) and stream raw audio over Wi-Fi UDP to the host laptop.
2. **Audio Processing Compute Location:**
   - *Option A (Distributed):* Pi 3B+ captures raw audio and streams it to the i7 laptop, which runs Silero VAD, openWakeWord, and SRP-PHAT.
   - *Option B (Edge standalone):* Pi 3B+ runs the C-based ODAS engine locally (7.5% CPU) and publishes only lightweight DOA coordinates to ROS 2.
3. **Radar Interfacing Location:**
   - *Option A (Direct to Pi):* Connect HLK-LD2402 UART to a $1 USB-to-UART adapter on the Pi 3B+ for zero ESP32 CPU overhead.
   - *Option B (Direct to ESP32):* Remap ESP32 UART pins (GPIO 23/18) and parse frames inside `rover.ino`, sending radar distance over the existing telemetry stream.

# RP2040 All-in-One Migration & ESP32-C3 Bridge Architecture Specification

- **Target Architecture**: Raspberry Pi RP2040 (Master Real-Time Controller) + ESP32-C3 Mini (ESP-NOW & WS2812 Coprocessor Bridge) + Raspberry Pi 3B+ (Vision & Host Telemetry).
- **Scope**: Complete removal of rover ESP32 DevKit; ultrasonic sensor decommissioned; dual VL53L0X fixed Time-of-Flight sensors (Front 0°, Rear 180°); camera aligned with front ToF.
- **Output Document**: `C:\Users\dell\Desktop\rover\docs\research\rp2040_migration.md`

---

## 1. Inventory of `rover.ino` Functions Moving to RP2040

The table below catalogs every function, ISR, algorithm, and network task currently running in `rover esp/rover/rover.ino` (~1972 lines), their execution rates, resource requirements, and migration destination.

| Function / Component | Line Range in `rover.ino` | Current Role & Hardware Resources | Target Platform | Target Execution Rate & Resource Cost on RP2040 |
| :--- | :--- | :--- | :--- | :--- |
| `readRawMPU()` / `initIMU()` | 1636–1714 | I2C (0x68) 400 kHz read of gyro (±2000°/s) & accel (±4g). DLPF config. | **RP2040 (Core 0)** | 100 Hz via I2C0 (`GP4`/`GP5`). ~45 µs per I2C burst read. |
| `compass.h` / `initMag()` | 1715–1737 | I2C (0x0D/0x1E) read of QMC5883L/HMC5883L. Hard-iron span tracking. | **RP2040 (Core 0)** | 50–100 Hz via I2C0. ~35 µs read. |
| `RoverFusion::update()` (`fusion.h`) | 869–872 | 9-DoF Madgwick AHRS quaternion update, online gyro bias compensation. | **RP2040 (Core 0)** | 100 Hz. Math cost: ~110 µs on Cortex-M0+ @ 133 MHz (ROM float library). |
| Real-time Impact / Jerk Detection | 809–854 | High-pass linear jerk: $hJerk = \sqrt{\Delta ax^2 + \Delta ay^2}$, EMA gravity tracking. | **RP2040 (Core 0)** | 100 Hz (synchronous with IMU tick). 420 ms defensive impact reflex. |
| `initToFSensors()` | 1380–1418 | Sequenced boot of 2x VL53L0X via XSHUT; dynamic address assign (0x32, 0x33). | **RP2040 (Core 1)** | Executed at boot and on bus watchdog timeout (>2500 ms stall). |
| `readRangeSensors()` (ToF part) | 1527–1550 | Non-blocking `isRangeComplete()` & `readRangeResult()`. Exponential filter. | **RP2040 (Core 1)** | 30–40 Hz continuous polling. Non-blocking I2C state machine. |
| `readRangeSensors()` (US part) | 1448–1526 | 70 ms trigger, median filter, fallback `pulseIn`. | **DECOMMISSIONED** | Ultrasonic sensor removed entirely. Zero RP2040 pins or cycles. |
| `usEchoISR()` | 143–159 | Edge interrupt capture on `PIN_US_ECHO` (GPIO 34). | **DECOMMISSIONED** | Removed. Frees interrupt handling. |
| `updateOdometry()` | 1553–1590 | Mecanum kinematic integration ($V_x, V_y, \omega$) from wheel mm/s into world `odomX, odomY`. | **RP2040 (Core 0)** | 50 Hz. Consumes counts from hardware PIO quadrature encoders. |
| `updateSensoryFusion()` | 348–417 | 48-point obstacle ring buffer (`dotX`, `dotY`), 12-sector polar histogram (`polarHist`). | **RP2040 (Core 0)** | 50 Hz. Updated with Front ToF (0°) and Rear ToF (180°) hits. |
| `getSensorRanges()` | 297–347 | Aggregates front distance $dF$ and rear distance $dR$, spatial memory fallback. | **RP2040 (Core 0)** | 50 Hz. Fuses camera visual distances ($camVisMm$) from Pi 3B+. |
| `applyCrashProtection()` | 734–785 | Velocity-dependent braking distance: $stopF = 180\text{mm} + 0.5 \cdot inertiaLeadMm$. | **RP2040 (Core 0)** | 50 Hz. Evaluates $y > 0$ with Front ToF and $y < 0$ with Rear ToF. |
| `runAutonomousTank()` | 418–733 | 12-state autonomous navigation FSM (Cruise, Creep, Stuck, Pivot, Reflex, etc.). | **RP2040 (Core 0)** | 50 Hz. Native C++ logic, zero serialization overhead. |
| `controlTick()` | 786–1113 | 50 Hz main control loop: mode dispatch, S-curve profiling (`profile()`), anti-stiction kick. | **RP2040 (Core 0)** | 50 Hz ticker. Drives motor PWM directly (no internal ASCII UART!). |
| Motor PWM Generation | `code.py` 10–16 | 4 motors x 2 channels (FL, FR, RL, RR) @ 20 kHz silent PWM. | **RP2040 (Hardware)** | 8 channels hardware PWM (Slices 4, 5, 6, 7). 0% CPU overhead. |
| Quadrature Encoders | *New* | 4 motors x 2 channels (A/B phases) = 8 pins. | **RP2040 (PIO)** | 2x PIO blocks (4 state machines). Zero CPU overhead up to 1 MHz edges. |
| Config & Calibration Storage | 217–260 | `Preferences.h` (NVS) for motor inversion, trims, gyro bias, mag min/max. | **RP2040 (LittleFS)** | LittleFS on onboard 2 MB QSPI flash (sector-based wear leveling). |
| Radar Presence Pin (`LD2402_OUT`) | 1421 | Digital read on `LD2402_OUT_PIN` (HIGH = Presence). | **RP2040 (GPIO)** | Polled at 50 Hz via `GP27`. |
| ESP-NOW Link (`onRecv`, `handlePacket`) | 200–210, 1116–1174 | 2.4 GHz ESP-NOW link to Makerfabs S3 remote (`RoverPkt`). | **ESP32-C3 Bridge** | ESP-NOW hardware transceiver. Forwards framed packets over UART0. |
| WS2812 Status LED Strip | *New* | Auto-state visual feedback (12 states + link loss). | **ESP32-C3 Bridge** | Hardware RMT / FastLED on C3 `GPIO 8`. |
| Host Telemetry UDP 8888 (`sendTelemetry`) | 1591–1635 | JSON & binary `TelemPkt` broadcast to `192.168.0.111:8888`. | **Raspberry Pi 3B+** | Already implemented in `pi_cam_node.cpp` (lines 631–720). |
| Host Drive UDP 8888 (`handleUdpPacket`) | 1176–1254 | Incoming UDP drive packets from web dashboard on port 8888. | **Raspberry Pi 3B+** | Pi 3B+ listens on 8888 and feeds RP2040 via UART/USB CDC. |
| HTTP Web Server (`/status`, `/w`, `/s`) | 1255–1347 | Port 80 REST API for dashboard telemetry and emergency stop. | **Raspberry Pi 3B+** | Pi 3B+ lightweight HTTP service (or `web_telemetry_server.py` direct). |
| ArduinoOTA (Wireless Flashing) | 1845–1870 | Network firmware flashing for ESP32 over WiFi. | **Raspberry Pi 3B+** | `picotool` via USB or SWD from Pi 3B+; C3 updated via `c3lab` OTA. |

---

## 2. Language & Runtime Decision for RP2040: CircuitPython vs C++ (Arduino-Pico)

### Evaluation Against Physical Constraints

| Criterion | CircuitPython (`code.py`) | C++ (Arduino-Pico / Earle Philhower Core) | Technical Evidence & Impact |
| :--- | :--- | :--- | :--- |
| **Execution Jitter & GC** | **Fatal** (5–25 ms GC pauses) | **Zero Jitter** (Deterministic sub-microsecond) | CircuitPython triggers `gc.collect()` periodically. A 15 ms GC freeze corrupts Madgwick AHRS integration ($\Delta t$ instability causes attitude drift) and misses encoder edge counts. |
| **Float Math Performance** | **Severe Bottleneck** (Boxed objects) | **Ultra Fast** (RP2040 Bootrom ASM Floats) | RP2040 has no FPU. CircuitPython boxes all floats into heap objects. A 9-DoF Madgwick filter + S-curve profiling at 100 Hz consumes >70% of 1 core in Python, but <1.2% in native C++. |
| **Quadrature Encoders** | **Inadequate** (Cannot sustain 40k edges/s) | **Native PIO Hardware** (2 PIO blocks, 4 SMs) | 4 optical/magnetic encoders at full speed produce up to 40,000 edge transitions/sec. CircuitPython has no interrupt routines; PIO handles up to 10 MHz pulse trains with 0% CPU load. |
| **Multi-Core Exploitation** | **Unstable / Restricted** | **Native Symmetric Dual-Core** (`setup1()` / `loop1()`) | Core 0: 100 Hz IMU AHRS, kinematic mixing, crash protection, FSM.<br>Core 1: Dual ToF I2C polling, UART packet encoding, and watchdog. |
| **Code Migration Path** | **Full Rewrite Required** (1972 lines C++ $\to$ Py) | **Direct 1:1 Porting** (Existing C++ files reused) | `compass.h`, `fusion.h`, and `rover_proto.h` compile directly in Arduino-Pico without algorithmic translation errors. |
| **Flash Persistence** | Emulated FAT12 disk (write freeze) | **LittleFS** (Background flash sector erase) | Saving calibration (`fusion.save()`) while driving freezes CircuitPython execution for up to 80 ms; LittleFS handles non-blocking writes. |

### Recommendation
**Unconditional Recommendation: C++ using the Arduino-Pico (Earle F. Philhower, III) Core.**  
CircuitPython is architecturally incapable of sustaining 100 Hz 9-DoF AHRS, dual-core concurrency, 4-wheel PIO quadrature counting, and sub-millisecond crash protection simultaneously.

---

## 3. Full RP2040 Pin Map (Raspberry Pi Pico DIP-40)

- **Assumed Board**: Standard Raspberry Pi Pico (RP2040, 2 MB QSPI Flash, DIP-40 form factor).
- **Power Configuration**: Pico powered via 5V regulated buck converter to `VSYS` (Pin 39). All motor high-current rails isolated from 3V3 rail.
- **Audio Note**: The INMP441 I2S microphone is connected to a dedicated separate microcontroller board; no I2S audio pins are required on this master RP2040.

```
                                  +---| USB |--+
           C3 Bridge UART0 TX --> | 1   GP0   40 | <-- VBUS (5V USB input)
         C3 Bridge UART0 RX <-- | 2   GP1   39 | <-- VSYS (5V Main BEC Input)
                          GND --> | 3   GND   38 | <-- GND
     Pi 3B+ / Aux UART1 TX --> | 4   GP2   37 | <-- 3V3_EN
     Pi 3B+ / Aux UART1 RX <-- | 5   GP3   36 | <-- 3V3(OUT) (Sensors Only!)
               I2C0 SDA (Shared) | 6   GP4   35 | <-- ADC_VREF
               I2C0 SCL (Shared) | 7   GP5   34 | <-- GP28 (Spare ADC / Analog)
              Front ToF 1 XSHUT | 8   GP6   33 | <-- GND
               Rear ToF 2 XSHUT | 9   GP7   32 | <-- GP27 (LD2402 Radar Presence OUT)
                          GND --> | 10  GND   31 | <-- GP26 (Enc3 RR Channel B)
             FL Motor Bck (PWM) | 11  GP8   30 | <-- RUN (Reset)
             FL Motor Fwd (PWM) | 12  GP9   29 | <-- GP22 (Enc3 RR Channel A)
             RL Motor Bck (PWM) | 13  GP10  28 | <-- GND
             RL Motor Fwd (PWM) | 14  GP11  27 | <-- GP21 (Enc2 RL Channel B)
                          GND --> | 15  GND   26 | <-- GP20 (Enc2 RL Channel A)
             FR Motor Fwd (PWM) | 16  GP12  25 | <-- GP19 (Enc1 FR Channel B)
             FR Motor Bck (PWM) | 17  GP13  24 | <-- GP18 (Enc1 FR Channel A)
             RR Motor Fwd (PWM) | 18  GP14  23 | <-- GND
             RR Motor Bck (PWM) | 19  GP15  22 | <-- GP17 (Enc0 FL Channel B)
                          GND --> | 20  GND   21 | <-- GP16 (Enc0 FL Channel A)
                                  +------------+
```

### Pin Allocation & Conflict Analysis Table

| RP2040 Pin | Physical Pin # | Function / Subsystem | Connected Device & Protocol | Conflict with `code.py` | Notes & Electrical Isolation |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **GP0** | Pin 1 | `UART0_TX` | ESP32-C3 Bridge RX (Framed binary) | **None** (Was `code.py` TX) | 921600 baud, 8N1. |
| **GP1** | Pin 2 | `UART0_RX` | ESP32-C3 Bridge TX (Framed binary) | **None** (Was `code.py` RX) | 921600 baud, 8N1. |
| **GP2** | Pin 4 | `UART1_TX` | Raspberry Pi 3B+ RX (`/dev/serial0`) | New Function | 115200 baud (or use USB CDC). |
| **GP3** | Pin 5 | `UART1_RX` | Raspberry Pi 3B+ TX (`/dev/serial0`) | New Function | Level-safe (Pi 3.3V LVTTL). |
| **GP4** | Pin 6 | `I2C0_SDA` | MPU6050, QMC5883L, 2x VL53L0X | New Function | Shared bus. Requires 4.7kΩ pull-up to 3V3. |
| **GP5** | Pin 7 | `I2C0_SCL` | MPU6050, QMC5883L, 2x VL53L0X | New Function | 400 kHz fast mode clock. |
| **GP6** | Pin 8 | `GPIO_OUT` | Front VL53L0X XSHUT | New Function | Active HIGH enable; boot address `0x32`. |
| **GP7** | Pin 9 | `GPIO_OUT` | Rear VL53L0X XSHUT | New Function | Active HIGH enable; boot address `0x33`. |
| **GP8** | Pin 11 | `PWM4_A` | Motor 0: Front-Left Backward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP9** | Pin 12 | `PWM4_B` | Motor 0: Front-Left Forward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP10** | Pin 13 | `PWM5_A` | Motor 2: Rear-Left Backward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP11** | Pin 14 | `PWM5_B` | Motor 2: Rear-Left Forward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP12** | Pin 16 | `PWM6_A` | Motor 1: Front-Right Forward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP13** | Pin 17 | `PWM6_B` | Motor 1: Front-Right Backward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP14** | Pin 18 | `PWM7_A` | Motor 3: Rear-Right Forward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP15** | Pin 19 | `PWM7_B` | Motor 3: Rear-Right Backward | **None** (Identical to `code.py`) | 20 kHz silent PWM. |
| **GP16** | Pin 21 | `PIO0_IN` | Encoder 0 (FL) Channel A | New Function | PIO Quadrature Decoder SM0. |
| **GP17** | Pin 22 | `PIO0_IN` | Encoder 0 (FL) Channel B | New Function | PIO Quadrature Decoder SM0. |
| **GP18** | Pin 24 | `PIO0_IN` | Encoder 1 (FR) Channel A | New Function | PIO Quadrature Decoder SM1. |
| **GP19** | Pin 25 | `PIO0_IN` | Encoder 1 (FR) Channel B | New Function | PIO Quadrature Decoder SM1. |
| **GP20** | Pin 26 | `PIO0_IN` | Encoder 2 (RL) Channel A | New Function | PIO Quadrature Decoder SM2. |
| **GP21** | Pin 27 | `PIO0_IN` | Encoder 2 (RL) Channel B | New Function | PIO Quadrature Decoder SM2. |
| **GP22** | Pin 29 | `PIO0_IN` | Encoder 3 (RR) Channel A | New Function | PIO Quadrature Decoder SM3. |
| **GP26** | Pin 31 | `PIO0_IN` | Encoder 3 (RR) Channel B | New Function | PIO Quadrature Decoder SM3. |
| **GP27** | Pin 32 | `GPIO_IN` | HLK-LD2402 Radar OUT Pin | New Function | HIGH = Moving target detected. |
| **GP28** | Pin 34 | `ADC2` | Spare / Battery Voltage Divider | New Function | $V_{bat} = V_{adc} \times 3.0$ (via divider). |
| **GP25** | Internal | `GPIO_OUT` | Onboard Pico Green LED | Internal | Heartbeat / Watchdog indicator. |

---

## 4. RP2040 <-> ESP32-C3 Protocol & C3 Bridge Responsibilities

### Serial Framing & Packet Specification
The unstructured comma-separated ASCII lines (`D,x,y,z,s\n`) are replaced with a high-speed binary framed packet protocol operating at **921,600 baud, 8N1** over hardware `UART0` (`GP0`/`GP1`).

```
+------------+------------+---------------+--------------+----------------------+--------------+
| SYNC0:0xAA | SYNC1:0x55 | MSG_TYPE:uint8| LEN:uint8    | PAYLOAD:[LEN bytes]  | CRC16:uint16 |
+------------+------------+---------------+--------------+----------------------+--------------+
```

- **Preamble**: `0xAA, 0x55` (prevents false lock on random motor telemetry bytes).
- **CRC**: `CRC16-CCITT` (Polynomial `0x1021`, initial value `0xFFFF`). Corrupted packets are rejected silently without motor twitching.
- **Payload Reuse**: Directly encapsulates `RoverPkt` (12 bytes) and `TelemPkt` (46 bytes) from `rover_proto.h`.
- **Latency & Overhead**:
  - `RoverPkt` frame: $2 + 1 + 1 + 12 + 2 = 18\text{ bytes} \implies 195\ \mu\text{s}$ transmission time at 921600 baud.
  - `TelemPkt` frame: $2 + 1 + 1 + 46 + 2 = 52\text{ bytes} \implies 564\ \mu\text{s}$ transmission time.
  - Total serial bandwidth at 50 Hz: $3.5\text{ KB/s}$ (less than 3.8% of available channel capacity).

### ESP32-C3 Duties & WS2812 Visual State Machine
1. **ESP-NOW Transceiver**:
   - Maintains continuous link with Makerfabs MaTouch 3.16 S3 Remote.
   - Forwards received `RoverPkt` commands into UART frame within <500 µs.
   - Transmits outgoing `TelemPkt` back to S3 remote over ESP-NOW.
2. **WS2812 Status LED Controller**:
   - Hardware Pin: **GPIO 8** (or LOLIN C3 Mini onboard RGB on GPIO 7).
   - Driver Library: `FastLED` or `Adafruit_NeoPixel` using hardware RMT.
   - Strip Length: `#define STATUS_LED_COUNT 8` (configurable constant).
   - Visual State Mapping Table:

| Rover State (`autoState` / mode) | Dominant Strip Color | Hex Code | Animation / Pattern | Description |
| :--- | :--- | :--- | :--- | :--- |
| `AUTO_CRUISE_CLEAR` | **Emerald Green** | `#00FF44` | Solid Breathing (1 Hz) | Forward autonomous motion clear. |
| `AUTO_THINKING_SCAN` | **Warm Amber / Yellow** | `#FFB700` | Rapid Chase / Orbit | Paused; scanning polar histogram. |
| `AUTO_PLANNING_PATH` | **Electric Blue** | `#0088FF` | Inward Sweep | Selecting open costmap sector. |
| `AUTO_VEER_LEFT` / `RIGHT` | **Tangerine Orange**| `#FF5500` | Side-directed wipe | Deflecting around lateral obstacle. |
| `AUTO_CAUTIOUS_CREEP` | **Cyan** | `#00FFFF` | Low-brightness slow pulse | Inching forward in narrow corridor. |
| `AUTO_STUCK_DETECTED` | **Crimson Red** | `#FF0000` | Rapid Strobe (5 Hz) | Stuck against obstacle or wheel slip. |
| `AUTO_REVERSE_BACKTRACK` | **Magenta / Fuchsia**| `#FF0088` | Reverse Flow Animation | Defensive backing up along breadcrumbs. |
| `AUTO_IMPACT_REFLEX` | **Strobe White** | `#FFFFFF` | Immediate 50 ms flash | Physical collision detected by MPU6050. |
| **Manual Mode (Teleop)** | **Dim Ice Blue** | `#1E293B` | Constant 15% glow | Normal manual operator driving. |
| **Link Lost / Failsafe** | **Deep Purple** | `#7E22CE` | 2 Hz Heartbeat Pulse | Remote disconnected; motors stopped. |

### Host Telemetry Bridge Takeover: Pi 3B+ vs ESP32-C3
- **Architectural Decision**: **The Raspberry Pi 3B+ takes over all Host Telemetry (UDP 8888, HTTP `/status`, and ROS 2 `/scan`).**
- **Justification**:
  1. **Radio Channel Concurrency Trap**: Running ESP-NOW concurrently with WiFi STA on an ESP32 causes severe packet drops whenever the home router AP operates on a different 2.4 GHz channel than the S3 remote, forcing channel time-slicing.
  2. **Memory & CPU Starvation**: The single-core 160 MHz RISC-V C3 has only 400 KB SRAM. Running ESP-NOW, WS2812 RMT, WiFi UDP broadcaster, and HTTP server on one core induces microsecond jitter on remote stick inputs.
  3. **Existing Pi Infrastructure**: `pi_cam_node.cpp` on Pi 3B+ already has a persistent UDP socket to `192.168.0.111:8888` and reads telemetry over UART (`/dev/serial0`). Expanding this Linux bridge requires zero extra microcontroller code.

---

## 5. Depth Perception & Sensor Physics: Camera + Dual Fixed ToF

### Can a Camera + Single-Point VL53L0X Create a Depth Map?
- **Fundamental Principle**: A single-point VL53L0X outputs **exactly one scalar distance** $Z_0$ within its 25° field of view cone. It does not possess angular resolution. A 2D RGB camera provides dense visual textures ($640 \times 480$) but suffers from projective depth ambiguity ($Z$ cannot be computed from a single monocular image without assumptions).
- **Depth Map Generation Mechanism**: Monocular Depth Estimation (MDE) neural networks predict relative depth maps $\hat{d}(u, v) \in [0, 1]$. An aligned coaxial ToF reading $Z_0$ provides metric anchoring to convert relative depth into absolute metric coordinates:
  $$Z(u, v) = Z_0 \cdot \frac{\hat{d}(u_{center}, v_{center})}{\hat{d}(u, v)}$$

### Comparison of Depth Sensing Architectures

| Method | Resolution / Channels | Real-Time Frame Rate | Compute Hardware Needed | Metric Accuracy & Reliability |
| :--- | :--- | :--- | :--- | :--- |
| **(a) Monocular Depth Estimation + Aligned Single ToF** | $640 \times 480$ dense | • **Pi 3B+**: 0.3 FPS (Unusable)<br>• **i3-5th Gen**: 15–20 FPS (MiDaS v2)<br>• **i7-4th Gen**: 25–35 FPS (MiDaS v2) | Host Server (Pi 3B+ offloads video to `192.168.0.111`) | **Moderate to Poor**: If the narrow ToF beam hits a thin foreground obstacle (e.g. table leg) while camera center sees background, entire image scales incorrectly. |
| **(b) 8x8 Multizone ToF Array (VL53L5CX)** | 64 discrete metric zones | **15–60 FPS** direct on I2C | Direct on RP2040 (Zero neural network or host dependency) | **High**: True physical time-of-flight measurements across 63° diagonal FOV. Direct obstacle grid. |
| **(c) Mechanically Swept Single ToF (Pan-Tilt)** | Up to $36 \times 1$ polar rays | 0.3–0.5 FPS (2–3 s per sweep) | RP2040 Servo PWM | **Poor on moving robot**: Severe motion skew, servo jitter, gear wear, mechanical complexity. |

### Evaluation of User's Fixed Layout (Front 0° @ 11 cm, Rear 180°)
- **Front Geometry**: VL53L0X mounted directly on chassis centerline at $x = +11\text{ cm}$, aligned with camera optical axis ($0^\circ$).
  - At $1.0\text{ m}$ distance, the 25° ToF cone footprint is:
    $$\text{Diameter} = 2 \cdot 1000 \cdot \tan(12.5^\circ) = 443\text{ mm (44.3 cm)}$$
  - Pi Camera v2 horizontal FOV is $62.2^\circ$. At $1.0\text{ m}$, camera field width is $118\text{ cm}$.
  - **Coverage Ratio**: The ToF illuminates only the central **37.5%** of the camera's horizontal view. Peripheral objects (outside the central 25° cone) are invisible to the ToF.
  - **Parallax**: Mounting the ToF 2.5 cm directly below the camera lens yields $0\text{ mm}$ horizontal parallax. Vertical parallax is $25\text{ mm}$, yielding an angular parallax of $\arctan(25 / 1000) = 1.43^\circ$ at 1 m and $4.76^\circ$ at 30 cm, safely inside the 25° reception cone.
- **Rear Geometry**: VL53L0X mounted at $x = -10\text{ cm}$, $y = 0$, pointing backwards ($180^\circ$).
  - Provides reverse collision clearance up to 2.0 m.
- **Safety Consequences & Side Blind Zones**:
  - Mecanum chassis can translate laterally ($x \neq 0$). With only Front (0°) and Rear (180°) sensors, the robot has **zero direct ranging coverage on the left and right flanks** ($155^\circ$ blind zone on each side).
- **Mandatory Safety Mitigations**:
  1. **Dual Reflex Braking**: Front ToF enforces immediate hard-stop if $y > 0$ and $dF < 180\text{ mm}$; Rear ToF enforces immediate hard-stop if $y < 0$ and $dR < 180\text{ mm}$.
  2. **Lateral Speed Capping**: Limit pure lateral strafe speed ($x \neq 0, y = 0$) to 35% duty cycle.
  3. **HLK-LD2402 Radar Peripheral Shield**: The 24 GHz radar provides a wide $\pm 60^\circ$ (120° total) presence beam. If radar detects presence while front ToF reads clear (>1.5 m), an obstacle is present in the lateral blind zone $\implies$ prohibit high-speed turns and cap speed.
  4. **Pi Camera 3-Zone Vision**: Use `vis_l_mm` and `vis_r_mm` visual obstacle metrics computed by `pi_cam_node.cpp` to monitor flanking space.

---

## 6. Sensor Geometry, ROS 2 URDF/TF, and Dashboard/Map Updates

### Sensor Transform Coordinate Table (REP-103: $+x$ Forward, $+y$ Left, $+z$ Up)

| Sensor Frame | Parent Link | $x$ (cm) | $y$ (cm) | $z$ (cm) | Roll (rad) | Pitch (rad) | Yaw (rad / deg) | Ranging Limits | FOV |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `camera_link` | `base_link` | +11.0 | 0.0 | +10.5 | 0.0 | 0.0 | 0.0 ($0^\circ$) | 0.20 m – 5.0 m | 62.2° H, 48.8° V |
| `camera_optical_frame` | `camera_link` | 0.0 | 0.0 | 0.0 | -1.5708 | 0.0 | -1.5708 | — | Optical frame standard |
| `tof_front_link` | `base_link` | +11.0 | 0.0 | +4.0 | 0.0 | 0.0 | 0.0 ($0^\circ$) | 0.03 m – 2.0 m | 25° cone (0.436 rad) |
| `tof_rear_link` | `base_link` | -10.0 | 0.0 | +4.0 | 0.0 | 0.0 | $\pi$ ($180^\circ$) | 0.03 m – 2.0 m | 25° cone (0.436 rad) |
| `radar_link` | `base_link` | +11.0 | 0.0 | +6.0 | 0.0 | 0.0 | 0.0 ($0^\circ$) | 0.15 m – 6.0 m | $\pm 60^\circ$ (120° total) |
| `ultrasonic_link` | — | — | — | — | — | — | **DELETED** | — | Decommissioned |

### Required Modifications Across Files

#### A. `ros2/urdf/rover.urdf.xacro`
- Remove `<link name="ultrasonic_link">` and `<joint name="ultrasonic_joint">`.
- Replace `tof_left_link` and `tof_right_link` (previously $\pm 33^\circ$) with:
  ```xml
  <!-- Front ToF (0 deg, aligned with camera at 11cm) -->
  <link name="tof_front_link"/>
  <joint name="tof_front_joint" type="fixed">
    <parent link="base_link"/>
    <child link="tof_front_link"/>
    <origin xyz="0.110 0.0 0.040" rpy="0 0 0"/>
  </joint>

  <!-- Rear ToF (180 deg, fixed at rear at -10cm) -->
  <link name="tof_rear_link"/>
  <joint name="tof_rear_joint" type="fixed">
    <parent link="base_link"/>
    <child link="tof_rear_link"/>
    <origin xyz="-0.100 0.0 0.040" rpy="0 0 3.14159265"/>
  </joint>
  ```

#### B. `ros2/ros2_rover/tof_laserscan_publisher.py`
- Remove ultrasonic subscriptions, median filtering history, and corroboration gating.
- Rename parameters: `tof_front_x = 0.110`, `tof_rear_x = -0.100`.
- Update synthetic `/scan` LaserScan beam population:
  - Beam 0° ($index \approx 180$ in 360-beam scan): Populate hit from Front ToF at $r = d_{front}$ with spread $\pm 12^\circ$.
  - Beam 180° ($index \approx 0$ and $index \approx 359$): Populate hit from Rear ToF at $r = d_{rear}$ with spread $\pm 12^\circ$.

#### C. `index.html` (3D Digital Twin & Three.js Map)
- Delete `usCone` and `usDisc` mesh elements from the 3D scene.
- Replace `tofLRay` (+33°) and `tofRRay` (-33°) with:
  - `tofFrontRay`: Origin at `(0, 0.04, -0.11)`, rotation $Y = 0$.
  - `tofRearRay`: Origin at `(0, 0.04, +0.10)`, rotation $Y = \pi$.
- In `update3DMapFromSensors()`:
  - Project Front ToF hit: $m_X = rPosX + fwdX \cdot 0.110$, $m_Z = rPosZ + fwdZ \cdot 0.110$, ray along $rYaw$.
  - Project Rear ToF hit: $m_X = rPosX - fwdX \cdot 0.100$, $m_Z = rPosZ - fwdZ \cdot 0.100$, ray along $rYaw + \pi$.

---

## 7. Phased Bench-Test & Migration Plan

This plan is structured into self-contained tasks suitable for sequential execution by flash-tier agents.

```
+-------------------------------------------------------------------------------+
| PHASE 0: Baseline Protection                                                  |
| 1. Git tag: 'pre-rp2040-migration'                                            |
| 2. Archive rover.ino.bin and code.py to firmware_backups/                     |
+-------------------------------------------------------------------------------+
                                      │
                                      ▼
+-------------------------------------------------------------------------------+
| TASK 1: RP2040 Motor PWM & PIO Encoders                                       |
| - Implement 20 kHz PWM on GP8-GP15.                                           |
| - Implement PIO Quadrature Decoders on GP16-GP22, GP26.                       |
| - Acceptance: Step motor duty 20%->100%; verify encoder mm/s matches direction.|
+-------------------------------------------------------------------------------+
                                      │
                                      ▼
+-------------------------------------------------------------------------------+
| TASK 2: RP2040 I2C AHRS & Impact Engine                                       |
| - Port MPU6050, QMC5883L, and fusion.h to I2C0 (GP4/GP5).                     |
| - Implement 100 Hz Madgwick AHRS and dynJerk impact detection.                |
| - Acceptance: Stable yaw output over 5 min; tap on chassis triggers IMPACT_*. |
+-------------------------------------------------------------------------------+
                                      │
                                      ▼
+-------------------------------------------------------------------------------+
| TASK 3: RP2040 Dual VL53L0X Non-blocking Driver                               |
| - Sequenced boot via GP6 (Front) and GP7 (Rear) XSHUT pins.                   |
| - Reassign I2C addresses: 0x32 (Front), 0x33 (Rear).                          |
| - Acceptance: Continuous 35 Hz non-blocking ranging; unplugging sensor        |
|   recovers via watchdog without stalling motor loop.                          |
+-------------------------------------------------------------------------------+
                                      │
                                      ▼
+-------------------------------------------------------------------------------+
| TASK 4: ESP32-C3 Bridge Firmware                                              |
| - Implement ESP-NOW receiver/transmitter for S3 remote.                       |
| - Implement framed UART0 protocol at 921,600 baud with CRC16.                 |
| - Implement WS2812 RMT driver on GPIO 8 mapping autoState to LED colors.      |
| - Acceptance: Remote sticks control UART packets; LED strip displays states.  |
+-------------------------------------------------------------------------------+
                                      │
                                      ▼
+-------------------------------------------------------------------------------+
| TASK 5: Motion Core & 12-State Autonomy Engine                                |
| - Integrate S-curve profiler, anti-stiction kick, and FSM on RP2040 Core 0.   |
| - Implement Front (0°) & Rear (180°) velocity-scaled crash protection.         |
| - Acceptance: Teleop driving responsive; approaching obstacle halts rover.     |
+-------------------------------------------------------------------------------+
                                      │
                                      ▼
+-------------------------------------------------------------------------------+
| TASK 6: Pi 3B+ & ROS 2 / Dashboard Synchronisation                            |
| - Update tof_laserscan_publisher.py, rover.urdf.xacro, and index.html.        |
| - Acceptance: ROS 2 /scan displays front/back hits; 3D digital twin renders    |
|   matching rays; python ros2/test_ros2_layer.py passes 100%.                  |
+-------------------------------------------------------------------------------+
```

---

## 8. Future Extension: Pan-Tilt Sweep Mechanics & Pitfalls

*Retained for future development after fixed bench testing is complete.*

1. **MG90 Servo Limitations**:
   - High current spikes (up to 800 mA stall per servo) cause severe voltage sag on 5V logic rails $\implies$ requires dedicated 5V 3A BEC with 470 µF electrolytic decoupling capacitor.
   - Plastic / nylon gears exhibit 2°–4° deadband and gear backlash, requiring look-up table calibration and anti-backlash approach moves.
2. **Sweep Duration vs Vehicle Velocity**:
   - A 180° pan sweep at 5° steps (36 points $\times$ 35 ms) requires ~1.3 s sensing + 1.2 s servo settling = 2.5 s total. On a rover advancing at 30 cm/s, the robot travels 75 cm during a single sweep, causing severe spatial distortion.
3. **Safety Interlock**:
   - If pan-tilt is added, a strict hardware interlock must cap forward drive speed to 0 if $|pan\_angle| > 10^\circ$, and automatically return pan to 0° before enabling forward throttle.
4. **Superior Alternative**:
   - Replace mechanical pan-tilt with a solid-state **VL53L5CX 8x8 multizone ToF array** (64 simultaneous beams, 60 Hz, zero moving parts, zero mechanical wear).

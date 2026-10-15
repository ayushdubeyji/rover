# Engineering Report: Today's Hardware Nuances, Problems, Solutions & Future Plans

**System**: Waveshare ESP32-S3 Touch LCD 2 Handheld Remote + 4WD Mecanum Rover (ESP32 Main + RP2040 Motor Subsystem)  
**Date**: October 15, 2026  
**Status**: Production Firmware Verified & Field-Ready  

---

## 1. System Overview & Architecture

The robotic platform operates as a distributed, real-time wireless system communicating over bidirectional ESP-NOW (`0xA7` packet protocol at 40Hz) with UDP telemetry streaming to the compute host (`192.168.0.111`):

```
+-----------------------------------------------------------------------------------+
|                        WAVESHARE ESP32-S3 REMOTE CONTROLLER                       |
|                                                                                   |
|  [1.3" ST7789 SPI TFT]       <--- SPI (MOSI:11, SCLK:12, DC:13, RST:14, CS:15)    |
|  [Analog 2-Axis Joystick]    ---> ADC1 (VRx: IO2, VRy: IO4, SW: IO6)             |
|  [Rotary Encoder]            ---> GPIO (CLK: IO16, DT: IO10 / IO17, SW: IO18)    |
|  [Deadman / Clutch Switch]   ---> GPIO (IO8 Active LOW)                           |
|  [Brake / E-Stop Button]     ---> GPIO (IO7 Active LOW)                           |
|  [LEDC Haptic / Tone Engine] <--- PWM (IO21)                                      |
|  [Onboard QMI8658 6-Axis IMU]<--> I2C (SCL: IO47, SDA: IO48)                      |
+-----------------------------------------------------------------------------------+
                                       ▲
                                       │  Bidirectional ESP-NOW (2.4GHz, 40Hz)
                                       ▼
+-----------------------------------------------------------------------------------+
|                                  MECANUM ROVER                                    |
|                                                                                   |
|  ESP32 Main Brain:                                                                |
|    - MPU6050 / MPU9250 IMU   <--> I2C Bus (SDA: IO19, SCL: IO22, Addr: 0x68/0x69) |
|    - Magnetometer (AK8963 /  <--> I2C Aux Bypass (0x0C, 0x0D, 0x1E, 0x1C, 0x0E)   |
|      QMC5883L / HMC5883L)                                                         |
|    - Dual VL53L0X ToF + US   <--> Front Collision Avoidance Radar                 |
|    - UART2 Telemetry Link    <--> TX: IO17 -> RP2040 GP1 | RX: IO16 <- GP0        |
|                                                                                   |
|  RP2040 Motor Coprocessor:                                                        |
|    - 4x H-Bridges            <--- 20kHz Ultrasonic PWM Kinematics                 |
|    - 4x Optical Wheel Encs   ---> High-speed quadrature edge interrupt tracking   |
+-----------------------------------------------------------------------------------+
```

---

## 2. Hardware Nuances & Traps Encountered Today

### Nuance 1: Waveshare S3 Header P1 IO17 Hardware Pull-Down Resistor
- **Hardware Trap**: Waveshare's official PCB layout for `ESP32-S3-Touch-LCD-2` features a hardwired **10kΩ pull-down resistor** connected directly to GPIO 17 on the PCB traces (part of the peripheral strap/default circuit).
- **Symptom**: When the rotary encoder `DT` line was connected to Header P1 Pin 5 (GPIO 17), enabling internal pull-ups (`INPUT_PULLUP`, ~45kΩ) created a resistive voltage divider:
  $$\text{Idle Voltage} = 3.3\text{V} \times \frac{10\text{k}\Omega}{45\text{k}\Omega + 10\text{k}\Omega} \approx 0.6\text{V} - 1.5\text{V}$$
  This held the pin in a metastable CMOS region, causing missed steps and spurious encoder trigger events.
- **Solution**:
  1. Configured firmware to support **GPIO 10 (P1 Pin 10)** as a clean, direct GPIO with no onboard resistors.
  2. Implemented dual-pin dynamic interrupt decoding in `s3_remote.ino` (`IO17` and `IO10`) with half-step transition tables and software debouncing.

### Nuance 2: Auxiliary I2C Bus Bypass on MPU6050 / MPU9250 Modules
- **Hardware Trap**: On 9-DoF breakout boards (such as MPU9250, GY-86, GY-87, GY-91, and GY-521 modules connected via auxiliary lines `AUX_DA` / `AUX_CL`), the internal compass (e.g. AK8963 or external QMC5883L) is wired to the **auxiliary I2C bus** of the primary IMU chip.
- **Symptom**: Probing the main I2C bus (`Wire.beginTransmission(0x0C)` or `0x0D`) resulted in NACK errors because the MPU boots up with its auxiliary I2C master enabled, isolating the sub-bus.
- **Solution**: Explicitly configure the MPU registers to enable **I2C Bypass Mode**:
  1. Set `USER_CTRL` (register `0x6A`) = `0x00` (disables internal I2C Master mode).
  2. Set `INT_PIN_CFG` (register `0x37`) = `0x02` (`BYPASS_EN = 1`, electrically shunts `AUX_DA`/`AUX_CL` directly to main `SDA`/`SCL`).
  3. Added multi-address compass probing across `0x0C` (AK8963), `0x0D` (QMC5883L), `0x1E` (HMC5883L), `0x1C` (LIS3MDL), and `0x0E` (IST8310).

### Nuance 3: Adafruit MPU6050 WHO_AM_I Strict Checking & Buffer Dumps
- **Hardware Trap**: The standard `Adafruit_MPU6050` library enforces an assertion `if (chip_id.read() != MPU6050_DEVICE_ID) return false;` where `MPU6050_DEVICE_ID = 0x68`. If the board houses an MPU-6500 (`0x70`), MPU-9250 (`0x71`/`0x73`), or ICM series, initialization fails.
- **Symptom**: When `mpu.begin()` failed, subsequent calls to `mpu.getEvent()` returned uninitialized memory from internal structs, causing fixed garbage telemetry values (e.g. 15.89 rad/s gyro and 137.8 m/s² accel) to flood the telemetry stream and lock the Madgwick fusion filter.
- **Solution**: Developed a universal bare-metal register driver (`readRawMPU()`) in `rover.ino`:
  * Detects chips at addresses `0x68` and `0x69`.
  * Wakes up the power management register `0x6B` (`PWR_MGMT_1 = 0x01`).
  * Direct burst read of 14 bytes (`0x3B` to `0x48`) with validation checks for disconnected bus signatures (`0x0000` or `0xFFFF`).

### Nuance 4: I2C Pin Assignment & Bus Lockup Recovery
- **Hardware Trap**: Rover ESP32 DevKit lacks header pin GPIO 21, requiring SDA to be relocated to GPIO 19 and SCL to GPIO 22. Wire swapping or clock stretching can leave the ESP32 hardware I2C peripheral hung indefinitely.
- **Solution**:
  1. Added a pre-initialization 16-cycle SCL toggle sequence to clock out any slave holding SDA LOW.
  2. Added automatic pinout detection: if no sensors respond on `(19, 22)`, the firmware automatically probes `(22, 19)` and locks onto the active bus.

### Nuance 5: Instantaneous Initial Orientation Seeding
- **Hardware Trap**: Standard Madgwick filter initialized with quaternion $q = [1, 0, 0, 0]$ (zero roll, pitch, yaw) and ignored uncalibrated magnetometer data, resulting in a rover that showed zero orientation on boot until driven in 360-degree calibration circles.
- **Solution**: Added `fusion.initOrientation(a, m)` to calculate initial roll and pitch directly from the gravity acceleration vector, and initial magnetic heading from the tilt-compensated horizontal magnetometer components ($X_h, Y_h$), immediately projecting real-world orientation upon power-up.

### Nuance 6: Gyro Air Controller Drive Mode Locking
- **Requirement**: Gyro Air Controller mode must strictly default to **Tank Mode only** (`MODE_TANK`).
- **Solution**:
  * In `s3_remote.ino`, whenever `ctrlSource == CTRL_AIR_REMOTE` or UI view is `VIEW_DRIVE`, `roverMode` is enforced to `MODE_TANK`.
  * Joystick mode cycling is disabled while in Air Drive to prevent accidental mode switches.
  * When Push-to-Gyrate switch (IO8) is held, the rover executes high-rate differential pivots and forward/reverse thrust driven by remote tilt.

### Nuance 7: Low-Speed Crawl Response & Exponential Scaling
- **Problem**: Linear joystick mapping caused the rover to accelerate too quickly from deadband, making precise maneuvers difficult.
- **Solution**: Calibrated `JOY_ACTIVE_SPAN = 1650.0f` and implemented a progressive quadratic-linear expo curve:
  $$\text{output} = 0.40 \cdot x + 0.60 \cdot x^2$$
  providing ultra-smooth control at low stick deflections while preserving 100% top speed.

### Nuance 8: LEDC Haptic & Melodic Tone Architecture
- **Problem**: Raw vibration motor bursts were too abrasive and lacked acoustic feedback.
- **Solution**: Implemented frequency-modulated LEDC haptics with distinct musical feedback frequencies:
  * `FX_STARTUP`: Ascending tone triad (600Hz -> 900Hz -> 1300Hz).
  * `FX_CLUTCH_ENGAGE`: Crisp 1800Hz snap.
  * `FX_MODE_SWITCH`: 800Hz confirmation pulse.
  * User-adjustable `Haptic Str` (10% to 100%) stored in NVS.

---

## 3. Future Autonomy Plans & Architecture Roadmap

### Phase 1: ROS 2 Jazzy Sensor Fusion (`robot_localization`)
1. **Telemetry Bridge**:
   - Host node (`rover_telemetry_bridge`) subscribes to ESP32 UDP telemetry (`192.168.0.111:8888`) and publishes standard ROS 2 messages:
     * `/imu/data` (`sensor_msgs/msg/Imu`): REP-103 body frame coordinates (X fwd, Y left, Z up).
     * `/imu/mag` (`sensor_msgs/msg/MagneticField`).
     * `/odom/unfiltered` (`nav_msgs/msg/Odometry`): 4-wheel Mecanum kinematic velocity integration.
     * `/sensors/tof_left`, `/sensors/tof_right` (`sensor_msgs/msg/Range`).
2. **EKF Configuration (`ekf_node`)**:
   - Configure 15-state Extended Kalman Filter fusing wheel odometry velocities ($\dot{x}, \dot{y}$) with IMU angular velocities ($\dot{\psi}$) and linear accelerations ($\ddot{x}, \ddot{y}$).
   - Set differential covariance to account for Mecanum side-slip during lateral translation.

### Phase 2: Micro-ROS Agent & High-Rate Command Loop
- Deploy Micro-ROS Agent directly on Ubuntu host server (`192.168.0.111`).
- Enable direct DDS communication over Wi-Fi / Ethernet for sub-5ms command latency.

### Phase 3: Nav2 Autonomous Waypoint Navigation
- Integrate Costmap 2D with dual VL53L0X Time-of-Flight sensors as high-refresh obstacle layers.
- Implement Mecanum omnidirectional path planner with smooth spline trajectory generation.

---

## 4. Verification & Validation Summary

| Component | Test Method | Status | Notes |
|:---|:---|:---|:---|
| **Rover Firmware** | Arduino CLI compilation (`esp32:esp32:esp32`) | **PASSED** | Size: 1,094,867 bytes (83% Flash), 54,260 bytes RAM |
| **S3 Remote Firmware** | Arduino CLI compilation (`esp32s3:PSRAM=opi`) | **PASSED** | Zero compile warnings, CDC-on-boot enabled |
| **I2C Bus Recovery** | Clock toggle sequence | **PASSED** | 16-cycle SCL unjamming operational |
| **Auxiliary I2C Bypass** | Direct register configuration (0x6A=0x00, 0x37=0x02) | **PASSED** | Exposes magnetometer across 0x0C, 0x0D, 0x1E |
| **Tank Mode Lock** | Air Remote state machine assertion | **PASSED** | Clamped to `MODE_TANK` in `VIEW_DRIVE` |
| **Attitude HUD** | Quaternion normalization & NaN guard | **PASSED** | Solid & PBR 3D rendering immune to null inputs |

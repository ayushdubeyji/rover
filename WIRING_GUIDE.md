# Comprehensive Wiring & Hardware Integration Guide
## 4-Wheel Mecanum Rover & Waveshare ESP32-S3 Remote Control System

---

### Table of Contents
1. [System Overview & Architecture](#1-system-overview--architecture)
2. [Waveshare ESP32-S3 Remote Controller Pinout & Wiring](#2-waveshare-esp32-s3-remote-controller-pinout--wiring)
   - [P1 and P2 Expansion Header Layout](#p1-and-p2-expansion-header-layout)
   - [1.3-Inch ST7789 SPI Display Wiring (Clustered on P2)](#13-inch-st7789-spi-display-wiring-clustered-on-p2)
   - [Analog Joystick Wiring (ADC1 Wi-Fi Safe)](#analog-joystick-wiring-adc1-wi-fi-safe)
   - [Rotary Encoder Wiring](#rotary-encoder-wiring)
   - [Tactile Push Buttons Wiring](#tactile-push-buttons-wiring)
   - [Haptic Vibration Motor Driver](#haptic-vibration-motor-driver)
   - [Onboard Hardwired Peripherals](#onboard-hardwired-peripherals)
3. [Rover ESP32 Pinout & Wiring](#3-rover-esp32-pinout--wiring)
   - [ESP32 to RP2040 UART Link](#esp32-to-rp2040-uart-link)
   - [MPU6050 6-Axis IMU Wiring](#mpu6050-6-axis-imu-wiring)
   - [Magnetometer (QMC5883L / HMC5883L / GY-271) Wiring](#magnetometer-qmc5883l--hmc5883l--gy-271-wiring)
   - [Shared I2C Bus Topology & Address Map](#shared-i2c-bus-topology--address-map)
4. [Power Architecture, Grounds & Brownout Prevention](#4-power-architecture-grounds--brownout-prevention)
5. [Pre-Flight Bench Testing & Verification Checklist](#5-pre-flight-bench-testing--verification-checklist)

---

### 1. System Overview & Architecture

The system consists of two coordinated subsystems communicating over low-latency **ESP-NOW** (bidirectional packet protocol `0xA7`):

1. **Handheld Remote Controller**:
   - Board: Waveshare ESP32-S3-Touch-LCD-2 (ESP32-S3R8, 8MB Octal PSRAM, 16MB Flash).
   - Display: 1.3-inch SPI ST7789 TFT (240x240 resolution).
   - Controls: Analog thumbstick (ADC1 channels), 360° Rotary Encoder with push-button, 2 auxiliary tactical buttons (Deadman trigger & Emergency Stop).
   - Air Remote Capability: Built-in 6-axis IMU (QMI8658) provides tilt-gesture driving.
   - Haptics: Discrete LEDC PWM vibration motor engine.

2. **Mecanum Rover Core**:
   - Primary Communications & Navigation: ESP32 DevKit running FreeRTOS/Arduino.
   - Motor & Kinematics Coprocessor: Raspberry Pi RP2040 running CircuitPython (`code.py`) driving 4 H-Bridges at 20kHz silent PWM.
   - Sensor Fusion: MPU6050 6-Axis Gyro/Accelerometer + QMC5883L/HMC5883L Magnetometer on I2C bus (GPIO 21 SDA / GPIO 22 SCL). Provides tilt-compensated magnetic heading, closed-loop drift-free Heading Lock, and absolute Field-Oriented driving.

```
+---------------------------------------------------------------------------------------+
|                               HANDHELD S3 REMOTE CONTROLLER                          |
|                                                                                       |
|  [1.3" ST7789 Display]  <--- SPI (MOSI:11, SCLK:12, DC:13, RST:14, CS:15, BL:9)       |
|  [Analog Joystick]      ---> ADC1 (VRx: IO2, VRy: IO4, SW: IO6)                       |
|  [Rotary Encoder]       ---> GPIO (CLK: IO16, DT: IO17, SW: IO18)                     |
|  [Tactile Buttons]      ---> GPIO (Btn1 Deadman: IO7, Btn2 E-Stop: IO8)               |
|  [Haptic Motor FET]     <--- LEDC PWM (IO21)                                          |
|  [Onboard QMI8658 IMU]  <--> I2C (SCL: IO47, SDA: IO48)                               |
+---------------------------------------------------------------------------------------+
                                        ▲
                                        │  Bidirectional ESP-NOW (2.4GHz, 40Hz)
                                        ▼
+---------------------------------------------------------------------------------------+
|                                    MECANUM ROVER                                      |
|                                                                                       |
|  ESP32 Main Controller:                                                               |
|    - MPU6050 6-Axis IMU         <--> I2C Bus (SDA: IO21, SCL: IO22, Addr: 0x68)      |
|    - Magnetometer (QMC/HMC)     <--> I2C Bus (SDA: IO21, SCL: IO22, Addr: 0x0D/0x1E)  |
|    - UART2 Serial Link          <--> TX: IO17 -> RP2040 GP1 | RX: IO16 <- RP2040 GP0 |
|                                                                                       |
|  RP2040 Motor Coprocessor:                                                            |
|    - Front-Left Motor  <--- PWM (GP9 Fwd, GP8 Bck)                                    |
|    - Front-Right Motor <--- PWM (GP12 Fwd, GP13 Bck)                                  |
|    - Rear-Left Motor   <--- PWM (GP11 Fwd, GP10 Bck)                                  |
|    - Rear-Right Motor  <--- PWM (GP14 Fwd, GP15 Bck)                                  |
+---------------------------------------------------------------------------------------+
```

---

### 2. Waveshare ESP32-S3 Remote Controller Pinout & Wiring

The Waveshare ESP32-S3 board features two 14-pin dual headers: **P1** (Left) and **P2** (Right).

#### P1 and P2 Expansion Header Layout

```
         +--------------------------------------------------------+
         |                 WAVESHARE ESP32-S3 PCB                 |
         |                                                        |
   P1    |                                                        |    P2
+--------+                                                        +--------+
| 1: IO2 (ADC1_CH1)  - Joy VRx     |     3V3        - VCC 3.3V  : 1 | Display VCC
| 2: IO4 (ADC1_CH3)  - Joy VRy     |     GND        - Ground    : 2 | Display GND
| 3: IO6 (ADC1_CH5)  - Joy SW      |     IO43(TXD0) - UART TX   : 3 |
| 4: IO16            - Enc CLK     |     IO44(RXD0) - UART RX   : 4 |
| 5: IO17            - Enc DT      |     IO47       - IMU SCL   : 5 | Onboard IMU
| 6: IO18            - Enc SW      |     IO48       - IMU SDA   : 6 | Onboard IMU
| 7: IO21            - Haptic PWM  |     IO15       - LCD CS    : 7 | Display CS
| 8: IO8 (ADC1_CH7)  - Btn2 E-Stop |     IO13       - LCD DC    : 8 | Display DC
| 9: IO7 (ADC1_CH6)  - Btn1 Trigger|     IO11       - LCD MOSI  : 9 | Display SDA
| 10:IO10            - Spare       |     IO12       - LCD SCLK  : 10| Display SCL
| 11:IO20            - Spare       |     IO14       - LCD RST   : 11| Display RES
| 12:IO19            - Spare       |     IO9        - LCD BL    : 12| Display BLK
| 13:GND             - Common GND  |     GND        - Ground    : 13|
| 14:5V              - 5V Rail     |     VBAT       - Battery + : 14|
+----------------------------------+--------------------------------+------+
```

---

#### 1.3-Inch ST7789 SPI Display Wiring (Clustered on P2)

The 1.3-inch ST7789 display (240x240, 7 or 8 pin module) wires **entirely to header P2** in a single consolidated block:

| 1.3" ST7789 Pin | Wire Label | Waveshare S3 Connection | Header Location | Description |
|:---|:---|:---|:---|:---|
| **VCC** | Power | **3V3** | **P2 Pin 1** | 3.3V Logic & Backlight Power |
| **GND** | Ground | **GND** | **P2 Pin 2** | Power Ground |
| **SCL / SCK** | Clock | **GPIO 12** | **P2 Pin 10** | SPI Serial Clock |
| **SDA / MOSI**| Data In | **GPIO 11** | **P2 Pin 9** | SPI Master-Out-Slave-In |
| **RES / RST** | Reset | **GPIO 14** | **P2 Pin 11** | Hardware Display Reset |
| **DC** | Data/Cmd | **GPIO 13** | **P2 Pin 8** | D/C Command Selection |
| **CS** | Chip Sel | **GPIO 15** | **P2 Pin 7** | SPI Chip Select (Active LOW) |
| **BLK / BL** | Backlight| **GPIO 9** | **P2 Pin 12** | Backlight PWM Brightness Control |

> **Note on Backlight**: If your ST7789 module has no `BLK` pin or it is internally tied HIGH, you can leave GPIO 9 unconnected. If present, the firmware automatically provides PWM brightness control on GPIO 9.

---

#### Analog Joystick Wiring (ADC1 Wi-Fi Safe)

> **CRITICAL ESP32-S3 HARDWARE CONSTRAINT**:
> Never connect analog inputs to **ADC2** when Wi-Fi / ESP-NOW is active. ADC2 is shared with the Wi-Fi SAR ADC controller and will return `0` or corrupted noise.
> We strictly route joystick analog potentiometers to **ADC1**:
> - VRx: **GPIO 2** (ADC1_CH1)
> - VRy: **GPIO 4** (ADC1_CH3)

| Joystick Module Pin | Waveshare S3 Connection | Header Location | Notes |
|:---|:---|:---|:---|
| **VCC** | **3V3** (or 5V) | **P2 Pin 1** (or P1 Pin 14) | Use 3.3V for 0-3.3V ADC matching |
| **GND** | **GND** | **P1 Pin 13** | Common Ground |
| **VRx** (X-Axis) | **GPIO 2** | **P1 Pin 1 (ADC1_CH1)** | Full 12-bit analog input (0-4095) |
| **VRy** (Y-Axis) | **GPIO 4** | **P1 Pin 2 (ADC1_CH3)** | Full 12-bit analog input (0-4095) |
| **SW** (Push Switch) | **GPIO 6** | **P1 Pin 3** | Active LOW (Firmware uses internal pull-up) |

---

#### Rotary Encoder Wiring

The rotary encoder allows scrolling through settings menus and fine-tuning speed limits.

| Encoder Pin | Waveshare S3 Connection | Header Location | Notes |
|:---|:---|:---|:---|
| **+ / VCC** | **3V3** | **P2 Pin 1** | Logic High supply |
| **GND** | **GND** | **P1 Pin 13** | Common Ground |
| **CLK (Phase A)** | **GPIO 16** | **P1 Pin 4** | Hardware ISR quadrature channel |
| **DT (Phase B)** | **GPIO 17** | **P1 Pin 5** | Direction decode channel |
| **SW (Push Switch)** | **GPIO 18** | **P1 Pin 6** | Menu Select (Active LOW, internal pull-up) |

---

#### Tactile Push Buttons Wiring

Two momentary push buttons wired directly to ground (no external pull-up resistors needed; firmware configures internal pull-ups):

| Button Function | Pin on Switch | Waveshare S3 Connection | Header Location | Logic Behavior |
|:---|:---|:---|:---|:---|
| **Button 1 (Deadman / Turbo)** | Pin A | **GPIO 7** | **P1 Pin 9** | Press to engage Air Tilt / Hold for Turbo |
| | Pin B | **GND** | **P1 Pin 13** | Ground return |
| **Button 2 (Emergency Stop)** | Pin A | **GPIO 8** | **P1 Pin 8** | Immediate stop command broadcast |
| | Pin B | **GND** | **P1 Pin 13** | Ground return |

---

#### Haptic Vibration Motor Driver

The ESP32-S3 GPIO outputs 3.3V at max 40mA, which is **not** sufficient to drive a DC coin vibration motor directly. Use an NPN transistor (2N2222 / S8050) or N-channel MOSFET (2N7000 / AO3400):

```
       5V (P1 Pin 14) or 3V3 (P2 Pin 1)
              │
              ├───[ + Motor - ]───┐
              │     ▲       │     │
              │     │ 1N4148│     │
              │     └───┤◀──┘     │
              │   (Flyback Diode) │
              │                   │
GPIO 21 ──────[ 1kΩ ]───┤ B (Base)   Collector
(P1 Pin 7)              │ 2N2222 NPN
                        │ E (Emitter)
                        └───┬─────
                            │
                           GND (P1 Pin 13)
```

| Component | Connected To | Header Location |
|:---|:---|:---|
| **Transistor Base** (via 1k resistor) | **GPIO 21** | **P1 Pin 7** |
| **Transistor Emitter** | **GND** | **P1 Pin 13** |
| **Motor Positive (+)** | **5V** or **3V3** | **P1 Pin 14** or **P2 Pin 1** |
| **Motor Negative (-)** | **Transistor Collector** | (Breadboard / Protoboard) |
| **Flyback Diode (1N4148)** | Across Motor terminals | Cathode to 5V/3V3, Anode to Collector |

---

#### Onboard Hardwired Peripherals

These are connected directly on the Waveshare PCB traces:
- **QMI8658 6-Axis IMU**:
  * I2C Clock: **GPIO 47**
  * I2C Data: **GPIO 48**
  * Interrupt 1: **GPIO 3**
  * I2C Address: **0x6B**
- **Battery Sense Voltage Divider**:
  * Voltage divider: 200kΩ (high side to VBAT) / 100kΩ (low side to GND).
  * Sense pin: **GPIO 5** (ADC1_CH4).
  * Multiplier formula: `Vbat = Vpin * (200k + 100k) / 100k = Vpin * 3.0`.

---

### 3. Rover ESP32 Pinout & Wiring

The Rover uses a standard ESP32 DevKit connected to an RP2040 motor coprocessor, with added **MPU6050** and **Magnetometer** modules.

```
                                  +-----------------------+
                                  |     ESP32 DEVKIT      |
                                  |                       |
RP2040 GP0 (TX) ──────────────────┤ GPIO 16 (RX2)         |
RP2040 GP1 (RX) ◀─────────────────┤ GPIO 17 (TX2)         |
                                  |                       |
MPU6050 SDA  ─────┬───────────────┤ GPIO 21 (I2C SDA)     |
MAG SDA      ─────┘               |                       |
                                  |                       |
MPU6050 SCL  ─────┬───────────────┤ GPIO 22 (I2C SCL)     |
MAG SCL      ─────┘               |                       |
                                  |                       |
3.3V Power Rail ──────────────────┤ 3V3                   |
Common Ground   ──────────────────┤ GND                   |
                                  +-----------------------+
```

---

#### ESP32 to RP2040 UART Link

The ESP32 communicates with the RP2040 over hardware `Serial2` at 115200 baud:

| ESP32 Pin | RP2040 Pin | Signal Name | Notes |
|:---|:---|:---|:---|
| **GPIO 17 (TX2)** | **GP1 (RX)** | ESP32 TX -> RP2040 RX | Motor drive commands (`D,x,y,z,spd`) |
| **GPIO 16 (RX2)** | **GP0 (TX)** | ESP32 RX <- RP2040 TX | Telemetry line (`T,w0,w1,w2,w3,macro`) |
| **GND** | **GND** | Common Ground | Essential reference potential |

---

#### MPU6050 6-Axis IMU Wiring

The MPU6050 provides high-rate angular velocity (`gyroZ`) and acceleration vectors for pitch/roll tilt compensation:

| MPU6050 Pin | ESP32 Pin | Wire Color Recommendation | Notes |
|:---|:---|:---|:---|
| **VCC** | **3V3** | Red | Power from ESP32 3.3V regulator |
| **GND** | **GND** | Black | Ground |
| **SCL** | **GPIO 22** | Yellow | Hardware I2C Clock |
| **SDA** | **GPIO 21** | Green | Hardware I2C Data |
| **AD0** | **GND** | Black | Sets I2C Address to **0x68** |
| **INT** | *Not Connected* | - | Polled by FreeRTOS tick loop |

---

#### Magnetometer (QMC5883L / HMC5883L / GY-271) Wiring

The magnetometer module provides magnetic heading for drift-free field orientation:

| Magnetometer Pin | ESP32 Pin | Wire Color Recommendation | Notes |
|:---|:---|:---|:---|
| **VCC** | **3V3** | Red | Power from ESP32 3.3V regulator |
| **GND** | **GND** | Black | Ground |
| **SCL** | **GPIO 22** | Yellow | Shared I2C Bus with MPU6050 |
| **SDA** | **GPIO 21** | Green | Shared I2C Bus with MPU6050 |
| **DRDY** | *Not Connected* | - | Not required |

---

#### Shared I2C Bus Topology & Address Map

Both the MPU6050 and the Magnetometer connect **in parallel** to GPIO 21 and GPIO 22:

```
ESP32 GPIO 21 (SDA) ──────┬──────────────────────┐
                          │                      │
                     [MPU6050 SDA]         [MAG SDA]
                          │                      │
ESP32 GPIO 22 (SCL) ──────┼──────────────────────┤
                          │                      │
                     [MPU6050 SCL]         [MAG SCL]
                          │                      │
ESP32 3V3 ────────────────┼──────────────────────┤
                          │                      │
                     [MPU6050 VCC]         [MAG VCC]
                          │                      │
ESP32 GND ────────────────┴──────────────────────┘
                     [MPU6050 GND]         [MAG GND]
```

**I2C Device Address Map (No Collision)**:
- `0x68`: MPU6050 Accel/Gyro
- `0x0D`: QMC5883L Compass (detected automatically by `RoverCompass`)
- `0x1E`: HMC5883L Compass (detected automatically by `RoverCompass`)

> **Pull-Up Resistors**: Both GY-521 (MPU6050) and GY-271/273 (Magnetometer) breakout boards include onboard 4.7kΩ pull-up resistors to 3.3V. When wired together in parallel, the effective pull-up resistance is ~2.35kΩ, which is ideal for high-speed 400kHz I2C operation without adding external resistors.

---

### 4. Power Architecture, Grounds & Brownout Prevention

1. **Dedicated Motor Power Rail vs. Logic Rail**:
   - The RP2040 and motors must be powered from the high-current battery pack.
   - The ESP32 logic controller should have a 100µF–470µF electrolytic decoupling capacitor across its 3.3V and GND rails to absorb voltage sags during motor acceleration spikes.
   - **All grounds MUST be bonded together**: ESP32 GND, RP2040 GND, Motor Driver GND, and Battery (-) must meet at a central star ground.

2. **ADC1 Only Rule on Remote**:
   - ESP32-S3 has two ADCs: ADC1 (GPIO 1–10) and ADC2 (GPIO 11–20).
   - During Wi-Fi/ESP-NOW packet transmission, the Wi-Fi modem locks the ADC2 controller. Any reading from ADC2 will return either 0 or corrupted spikes.
   - For this reason, the joystick potentiometers are wired strictly to **GPIO 2 (ADC1_CH1)** and **GPIO 4 (ADC1_CH3)**.

3. **Remote Battery Circuit**:
   - Connect a 3.7V LiPo cell to the JST battery connector or `VBAT` (P2 Pin 14) and `GND`.
   - The onboard charger charges via the USB-C port.
   - The onboard voltage divider on GPIO 5 automatically monitors cell voltage.

---

### 5. Pre-Flight Bench Testing & Verification Checklist

- [ ] **Step 1: Check Continuity & Pin Polarity**:
  Verify 3.3V and GND are not shorted before applying battery or USB power.
- [ ] **Step 2: Flash Rover Firmware**:
  Upload `C:\Users\dell\Desktop\rover\rover esp\rover\rover.ino`.
  Open Serial Monitor at 115200 baud. Check for output:
  `[IMU] MPU6050 ready.`
  `[MAG] Magnetometer ready: QMC5883L` (or `HMC5883L`).
- [ ] **Step 3: Flash Remote Firmware**:
  Upload `C:\Users\dell\Desktop\rover\s3_remote\s3_remote.ino` (or open `s3 remote\s3 remote.ino`).
  Verify ST7789 display lights up with the dark HUD theme, reticle circle, battery gauge, and mode badge.
- [ ] **Step 4: Check Link Acquisition**:
  Turn on both Rover and Remote. Within 500ms, the Remote status badge changes from `OFFLINE` (red) to `LINK OK` (green) with a haptic double-click.
- [ ] **Step 5: Test Joystick & Crosshair**:
  Move joystick. Observe the cyan dot inside the reticle moving smoothly.
- [ ] **Step 6: Test Air Remote Tilt Mode**:
  Click the Rotary Encoder knob to open the settings menu. Change `Control Input` from `Joy` to `AirTilt`. Press and hold Button 1 (GPIO 7). Tilt the remote forward/backward and side to side; verify Mecanum wheels rotate accordingly. Release Button 1 to confirm instant failsafe stop.
- [ ] **Step 7: Test Heading Lock & Field-Oriented Mode**:
  Switch mode to `Heading Lock` or `Field-Oriented`. Rotate the rover by hand; verify heading telemetry updates on the remote HUD (`HD: xxx deg`) and that the rover corrects its heading.

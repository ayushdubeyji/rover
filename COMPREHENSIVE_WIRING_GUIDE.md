# Comprehensive Hardware & Wiring Guide: Mecanum Rover & S3 Air Remote

This guide provides complete, pin-by-pin wiring instructions and schematics for both ends of the system:
1. **Waveshare ESP32-S3 Remote Controller** (1.3" ST7789 Display, Analog Joystick, Rotary Encoder, Buttons, Haptics, and Onboard QMI8658 IMU).
2. **Rover Generic ESP32** (MPU6050 6-Axis IMU, QMC5883L/HMC5883L Magnetometer Compass, and RP2040 Motor Controller UART).

---

## Part 1: Waveshare ESP32-S3-Touch-LCD-2 Remote Controller

### Board Overview & Header Layout
The original fried 2.0-inch capacitive display has been removed. The remote uses the two 14-pin 2.54mm pitch breakout headers (**P1** and **P2**) located on the board.

```
       +--------------------------------------------------------+
       |             Waveshare ESP32-S3-Touch-LCD-2             |
       |                   (Display Removed)                    |
       |                                                        |
       |   P1 (14-Pin Header)              P2 (14-Pin Header)   |
       |  --------------------            --------------------  |
       |  Pin  1: IO2   (ADC1_CH1)        Pin  1: 3V3           |
       |  Pin  2: IO4   (ADC1_CH3)        Pin  2: GND           |
       |  Pin  3: IO6   (ADC1_CH5)        Pin  3: IO43 (TXD0)   |
       |  Pin  4: IO16  (ADC2_CH5)        Pin  4: IO44 (RXD0)   |
       |  Pin  5: IO17  (ADC2_CH6)        Pin  5: IO47 (IMU SCL)|
       |  Pin  6: IO18  (ADC2_CH7)        Pin  6: IO48 (IMU SDA)|
       |  Pin  7: IO21                    Pin  7: IO15          |
       |  Pin  8: IO8   (ADC1_CH7)        Pin  8: IO13          |
       |  Pin  9: IO7   (ADC1_CH6)        Pin  9: IO11          |
       |  Pin 10: IO10  (ADC1_CH9)        Pin 10: IO12          |
       |  Pin 11: IO20  (USB D+)          Pin 11: IO14          |
       |  Pin 12: IO19  (USB D-)          Pin 12: IO9  (ADC1_CH8)|
       |  Pin 13: GND                     Pin 13: GND           |
       |  Pin 14: 5V                      Pin 14: VBAT          |
       +--------------------------------------------------------+
```

---

### Component Wiring Table (S3 Remote)

#### 1. 1.3-inch SPI TFT Display (ST7789 240×240)
All display signals are grouped on **Header P2** for clean, short ribbon-cable routing:

| Display Pin Label | Target Header & Pin | ESP32-S3 GPIO | Function / Notes |
| :--- | :--- | :--- | :--- |
| **GND** | **P2 Pin 2** | **GND** | Ground |
| **VCC** | **P2 Pin 1** | **3.3V** | Power (3.3V) |
| **SCL / SCK** | **P2 Pin 10** | **GPIO 12** | SPI Clock |
| **SDA / MOSI** | **P2 Pin 9** | **GPIO 11** | SPI Data |
| **RES / RST** | **P2 Pin 11** | **GPIO 14** | Display Reset (active LOW) |
| **DC** | **P2 Pin 8** | **GPIO 13** | Data / Command Select |
| **BLK / BL** | **P2 Pin 12** | **GPIO 9** | Backlight brightness (PWM) |
| *(CS Pin)* | *None* | *None* | **Tied to GND internally on 7-pin modules** (GPIO 15 is free) |

> [!NOTE]
> Standard 1.3" ST7789 7-pin breakout boards have their CS line tied directly to GND on the module PCB, so the display is permanently chip-selected. The firmware is configured with `LCD_CS_PIN = -1`, so no CS connection is needed. P2 Pin 7 (GPIO 15) remains completely free!

---

#### 2. Analog 2-Axis Joystick + Switch
> [!IMPORTANT]
> **ADC1 Wi-Fi Immunity Rule:**
> On the ESP32-S3, **ADC2 is shared with the Wi-Fi / ESP-NOW radio**. Reading ADC2 channels during transmission causes brownouts and noisy jitter. Joystick axes are strictly mapped to **ADC1** pins on Header P1.

| Joystick Pin | Target Header & Pin | ESP32-S3 GPIO | Function / Notes |
| :--- | :--- | :--- | :--- |
| **VCC** | **P2 Pin 1** | **3.3V** | Analog reference supply |
| **GND** | **P1 Pin 13** | **GND** | Analog ground |
| **VRx (X-Axis)** | **P1 Pin 1** | **GPIO 2** | ADC1_CH1 (Left / Right strafe) |
| **VRy (Y-Axis)** | **P1 Pin 2** | **GPIO 4** | ADC1_CH3 (Forward / Reverse) |
| **SW (Button)** | **P1 Pin 3** | **GPIO 6** | Digital IN (Deadman switch / Turbo) |

---

#### 3. Rotary Encoder (Menu Scroll & Detent Controls)
Used for 360° flick spins, menu navigation, and parameter adjustments:

| Encoder Pin | Target Header & Pin | ESP32-S3 GPIO | Function / Notes |
| :--- | :--- | :--- | :--- |
| **CLK / A** | **P1 Pin 4** | **GPIO 16** | Quadrature Channel A (Interrupt on CHANGE) |
| **DT / B** | **P1 Pin 5** | **GPIO 17** | Quadrature Channel B |
| **SW (Button)** | **P1 Pin 6** | **GPIO 18** | Push-button (Menu Select / Enter) |
| **+ / VCC** | **P2 Pin 1** | **3.3V** | Pull-up supply |
| **GND** | **P1 Pin 13** | **GND** | Ground |

---

#### 4. External Push Buttons (To Ground)
Tactile switches configured with ESP32-S3 internal pull-ups (`INPUT_PULLUP`):

| Button | Terminal 1 | Terminal 2 | ESP32-S3 GPIO | Function |
| :--- | :--- | :--- | :--- | :--- |
| **Button 1 (Trigger)** | **P1 Pin 9** | **P1 Pin 13 (GND)** | **GPIO 7** | Deadman safety switch (Air Remote) / Mode Toggle |
| **Button 2 (E-Stop)** | **P1 Pin 8** | **P1 Pin 13 (GND)** | **GPIO 8** | Immediate Emergency Stop / Zero Heading |

---

#### 5. ERM / LRA Haptic Vibration Motor
Connect using an NPN transistor (e.g., 2N2222, S8050) or N-channel MOSFET (e.g., 2N7002, AO3400) to protect the GPIO from inductive flyback:

```
                  +3.3V
                    |
                  [ + ] Haptic Motor
                  [ - ]
                    |--------+
                    |        |
                   ===     [1N4148 Diode: Cathode to +3.3V, Anode to Drain]
              Gate | |       |
  GPIO 21 ---[ 1k ]| |       |
  (P1 Pin 7)       | |-------+
                  ===
                   | Source
                  GND (P1 Pin 13)
```

| Signal | Target Header & Pin | ESP32-S3 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **Gate / Base** | **P1 Pin 7** | **GPIO 21** | 12 kHz LEDC PWM output |
| **Power (+)** | **P2 Pin 1** | **3.3V** | Motor positive terminal |
| **Ground (-)** | **P1 Pin 13** | **GND** | Transistor emitter/source ground |

---

#### 6. Onboard Peripherals (Zero Extra Wiring Needed)
* **QMI8658C 6-Axis IMU:** Internally wired to `GPIO 47` (SCL) and `GPIO 48` (SDA) with interrupt on `GPIO 3`. I2C Address = `0x6B`.
* **Battery ADC Monitor:** Internally wired to `GPIO 5` through a 200k/100k voltage divider to `VBAT`. Formula: $V_{\text{bat}} = V_{\text{ADC}} \times 3.0$.

---

## Part 2: Rover Generic ESP32 + MPU6050 + Magnetometer

### Rover Architecture Overview
The Rover consists of:
1. **Generic ESP32 DevKit (30-pin or 38-pin):** Handles ESP-NOW reception, Wi-Fi web dashboard, kinematics calculations, S-curve profiling, and sensor fusion.
2. **MPU6050 (6-Axis Gyro + Accelerometer):** Provides high-rate (100 Hz) angular velocity ($\omega_z$) for heading integration.
3. **QMC5883L / HMC5883L (3-Axis Magnetometer):** Provides absolute magnetic heading reference to cancel out gyro drift over long runs.
4. **RP2040 Motor Controller (CircuitPython `code.py`):** Drives the 4 PWM motor H-bridges via dedicated 3.7V rail.

---

### Rover Wiring Diagram & Interconnections

```
   [ 3.3V Rail ] -----------------------------+-------------+
                                              |             |
   [ GND Rail  ] --------------------+--------+----+--------+----+
                                     |        |    |        |    |
 +--------------------------+        |        |    |        |    |
 |      Generic ESP32       |        |        |    |        |    |
 |                          |        |        |    |        |    |
 |  3V3                     |--------+        |    |        |    |
 |  GND                     |-----------------+    |        |    |
 |                          |                      |        |    |
 |  GPIO 21 (I2C SDA)       |----------------------+--------|----+ [SDA] QMC5883L / HMC5883L
 |  GPIO 22 (I2C SCL)       |-----------------+    |        +----+ [SCL] (Addr: 0x0D / 0x1E)
 |                          |                 |    |        |    | [VCC: 3.3V, GND: GND]
 |  GPIO 17 (UART2 TX2)     |---+             |    |        |    |
 |  GPIO 16 (UART2 RX2)     |---|--+          |    +--------|----+ [SDA] MPU6050
 |                          |   |  |          +-------------|----+ [SCL] (Addr: 0x68, AD0->GND)
 +--------------------------+   |  |                        |    | [VCC: 3.3V, GND: GND]
                                |  |
                                |  |       Common GND
                                |  +------------+
                                +------------+  |
                                             |  |
                                       +-----+--+------------------+
                                       |      RP2040 Board         |
                                       |   (CircuitPython code)    |
                                       |                           |
                                       |  GP1 (UART RX) <--- TX2   |
                                       |  GP0 (UART TX) ---> RX2   |
                                       |  GND           <--- GND   |
                                       |                           |
                                       |  GP8/9   -> FL Motor PWM  |
                                       |  GP10/11 -> RL Motor PWM  |
                                       |  GP12/13 -> FR Motor PWM  |
                                       |  GP14/15 -> RR Motor PWM  |
                                       +---------------------------+
```

---

### Pin-by-Pin Rover Connection Table

#### 1. I2C Sensor Bus (MPU6050 & Magnetometer)
Both sensors share the same hardware I2C bus (`Wire.begin(21, 22)`):

| Sensor Pin | ESP32 Pin | Function / Protocol | Notes |
| :--- | :--- | :--- | :--- |
| **MPU6050 VCC** | **3V3** | Power (3.3V) | Do NOT connect to 5V |
| **MPU6050 GND** | **GND** | Ground | Common system ground |
| **MPU6050 SCL** | **GPIO 22** | I2C Clock | Shared I2C line |
| **MPU6050 SDA** | **GPIO 21** | I2C Data | Shared I2C line |
| **MPU6050 AD0** | **GND** | Address Select | Sets I2C Address to `0x68` |
| **MPU6050 INT** | *Unconnected* | Interrupt | Optional (polled via timer) |
| **QMC5883L VCC** | **3V3** | Power (3.3V) | Shared 3.3V rail |
| **QMC5883L GND** | **GND** | Ground | Shared ground rail |
| **QMC5883L SCL** | **GPIO 22** | I2C Clock | Shared with MPU6050 SCL |
| **QMC5883L SDA** | **GPIO 21** | I2C Data | Shared with MPU6050 SDA |
| **QMC5883L DRDY**| *Unconnected* | Data Ready | Not required |

> [!TIP]
> **Magnetometer Placement:**
> Mount the magnetometer board as far away as possible (at least 5–8 cm) from the high-current DC motor wiring and batteries to minimize electromagnetic interference.

---

#### 2. ESP32 to RP2040 UART Bus
Operates at **115200 baud, 8N1**:

| ESP32 Pin | RP2040 Pin | Function / Direction | Packet Structure |
| :--- | :--- | :--- | :--- |
| **GPIO 17 (TX2)** | **GP1 (RX)** | ESP32 Output -> RP2040 Input | Commands (`D,x,y,z,s\n`, `S\n`, `K,...`) |
| **GPIO 16 (RX2)** | **GP0 (TX)** | RP2040 Output -> ESP32 Input | Telemetry (`T,fl,fr,rl,rr,macro\n`) |
| **GND** | **GND** | Common Ground | Essential reference plane |

---

## Part 3: Power & Noise Isolation Best Practices

1. **Separate Power Rails:**
   - Motors run from the dedicated high-current 3.7V/7.4V battery pack.
   - ESP32 and RP2040 run from a clean 5V regulated step-down buck converter or power bank.
   - Connect all GND lines together at a single star-ground point.
2. **I2C Pull-Up Resistors:**
   - Both MPU6050 and QMC5883L breakout boards feature onboard 4.7kΩ pull-up resistors to 3.3V. Parallel connection on short wires (<15 cm) works out-of-the-box.
3. **Decoupling Capacitors:**
   - Place a 100nF ceramic capacitor across VCC and GND close to the MPU6050 sensor to absorb motor brush switching transients.

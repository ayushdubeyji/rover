# Hardware Schematic & Pin Interconnect Reference
## Waveshare ESP32-S3 Remote & Mecanum Rover Architecture

---

### 1. Remote Controller Schematic Diagram

```
===================================================================================================
                     WAVESHARE ESP32-S3-TOUCH-LCD-2 REMOTE CONTROLLER
===================================================================================================

                    +------------------------------------+
                    |        ESP32-S3-WROOM-1 / R8       |
                    |   (8MB Octal PSRAM, 16MB Flash)    |
                    +------------------------------------+

     ====================== HEADER P1 (14 PINS) ======================
     Pin 1:  IO2 (ADC1_CH1) ─────────────── Analog Joystick VRx (X-Axis Potentiometer)
     Pin 2:  IO4 (ADC1_CH3) ─────────────── Analog Joystick VRy (Y-Axis Potentiometer)
     Pin 3:  IO6 (ADC1_CH5) ─────────────── Analog Joystick SW  (Push-switch to GND)
     Pin 4:  IO16           ─────────────── Rotary Encoder Phase A (CLK)
     Pin 5:  IO17           ─────────────── Rotary Encoder Phase B (DT)
     Pin 6:  IO18           ─────────────── Rotary Encoder Push Button (SW to GND)
     Pin 7:  IO21           ────[1kΩ]────── Transistor Base (2N2222) ── Haptic Vibration Motor
     Pin 8:  IO8 (ADC1_CH7) ─────────────── Momentary Tactile Switch (Button 2: E-STOP to GND)
     Pin 9:  IO7 (ADC1_CH6) ─────────────── Momentary Tactile Switch (Button 1: DEADMAN / TURBO to GND)
     Pin 10: IO10           ─────────────── (Expansion / Spare)
     Pin 11: IO20           ─────────────── (Expansion / Spare)
     Pin 12: IO19           ─────────────── (Expansion / Spare)
     Pin 13: GND            ─────────────── Common Ground (Joystick, Encoder, Buttons, Haptic)
     Pin 14: 5V             ─────────────── 5V Supply Rail (From USB / Booster to Joystick/Motor)

     ====================== HEADER P2 (14 PINS) ======================
     Pin 1:  3V3            ─────────────── ST7789 VCC & Encoder VCC
     Pin 2:  GND            ─────────────── ST7789 GND
     Pin 3:  IO43 (TXD0)    ─────────────── Hardware Serial TX0
     Pin 4:  IO44 (RXD0)    ─────────────── Hardware Serial RX0
     Pin 5:  IO47           ──────┬──────── Onboard QMI8658 SCL (I2C Clock)
     Pin 6:  IO48           ──────┼──────── Onboard QMI8658 SDA (I2C Data)
     Pin 7:  IO15           ──────┼──────── ST7789 CS   (SPI Chip Select)
     Pin 8:  IO13           ──────┼──────── ST7789 DC   (Data / Command Select)
     Pin 9:  IO11           ──────┼──────── ST7789 MOSI (SPI Data In / SDA)
     Pin 10: IO12           ──────┼──────── ST7789 SCLK (SPI Clock / SCL)
     Pin 11: IO14           ──────┼──────── ST7789 RST  (Hardware Reset / RES)
     Pin 12: IO9  (ADC1_CH8)──────┴──────── ST7789 BLK  (Backlight LED PWM)
     Pin 13: GND            ─────────────── Power Ground
     Pin 14: VBAT           ─────────────── Battery Positive (+) Terminal (3.7V LiPo)

     ====================== ONBOARD HARDWIRED TRACES ======================
     IO47: I2C SCL ──────────────────────── QMI8658 6-Axis IMU (Addr: 0x6B)
     IO48: I2C SDA ──────────────────────── QMI8658 6-Axis IMU (Addr: 0x6B)
     IO3:  INT1    ──────────────────────── QMI8658 Interrupt 1
     IO5:  ADC1_CH4──────────────────────── Battery Voltage Sense (200kΩ / 100kΩ divider to VBAT)
```

---

### 2. Rover ESP32 & Coprocessor Schematic Diagram

```
===================================================================================================
                        MECANUM ROVER CONTROLLER & SENSOR FUSION
===================================================================================================

  +-----------------------+                            +-------------------------------+
  |     ESP32 DEVKIT      |                            |       RP2040 COPROCESSOR      |
  |                       |                            |       (CircuitPython)         |
  |  GPIO 17 (TX2) ───────┼──────── UART 115k2 ───────▶│ GP1 (RX)                      |
  |  GPIO 16 (RX2) ◀──────┼────────────────────────────┤ GP0 (TX)                      |
  |  GND           ───────┼────────────────────────────┤ GND                           |
  +-----------┬-----------+                            +---------------+---------------+
              │                                                        │
              │ I2C Bus (400kHz)                                       │ 20kHz Silent PWM
              ▼                                                        ▼
  +─────────────────────────+                         +────────────────────────────────+
  | MPU6050 6-AXIS IMU      |                         | MOTOR 0: FL (GP9 Fwd, GP8 Bck) |
  |   - SDA: GPIO 21        |                         | MOTOR 1: FR (GP12 Fwd,GP13 Bck)|
  |   - SCL: GPIO 22        |                         | MOTOR 2: RL (GP11 Fwd,GP10 Bck)|
  |   - Addr: 0x68          |                         | MOTOR 3: RR (GP14 Fwd,GP15 Bck)|
  +─────────────────────────+                         +--------------------------------+
              │
              │ Shared I2C Bus
              ▼
  +─────────────────────────+
  | MAGNETOMETER MODULE     |
  |   - QMC5883L (0x0D) or  |
  |     HMC5883L (0x1E)     |
  |   - SDA: GPIO 21        |
  |   - SCL: GPIO 22        |
  +─────────────────────────+
```

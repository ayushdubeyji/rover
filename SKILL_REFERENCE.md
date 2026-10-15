---
name: waveshare-esp32-s3-remote
description: Comprehensive hardware reference, pinout architecture, and firmware guide for the Waveshare ESP32-S3-Touch-LCD-2 board converted to a custom Mecanum Rover remote controller with 1.3-inch ST7789 SPI display, analog joystick, rotary encoder, QMI8658 Air Remote gestures, and MPU6050+magnetometer sensor fusion.
---

# Waveshare ESP32-S3 Remote & Rover System Skill Reference

## Overview
This skill provides complete pinouts, hardware constraints, register definitions, and firmware architecture when working with the **Waveshare ESP32-S3-Touch-LCD-2** development board converted into a dedicated remote controller, along with the **Mecanum Rover** ESP32 host firmware.

---

## Hardware Specifications

### Waveshare ESP32-S3-Touch-LCD-2
* **SoC:** ESP32-S3R8 (Xtensa 32-bit LX7 dual-core @ 240 MHz).
* **Memory:** 512 KB SRAM, 8 MB Octal PSRAM (stacked), 16 MB SPI Flash.
* **Onboard IMU:** QMI8658C 6-axis Accelerometer + Gyroscope (`I2C 0x6B`, SCL=GPIO 47, SDA=GPIO 48, INT1=GPIO 3).
* **Battery Sensing:** ETA6098 Li-ion charger with 200k/100k voltage divider on `GPIO 5` ($3.0\times$ ADC multiplier).
* **Expansion Headers:** Two 14-pin 2.54mm headers (**P1** and **P2**).

---

## Pinout Mapping & Hardware Rules

### 1. Header P1 (14-Pin)
| Pin # | Net Name | Primary Role in Remote | Constraints / Notes |
| :--- | :--- | :--- | :--- |
| **1** | `IO2` | **Joystick VRx** | **ADC1_CH1** (Wi-Fi safe, no transmission jitter) |
| **2** | `IO4` | **Joystick VRy** | **ADC1_CH3** (Wi-Fi safe, no transmission jitter) |
| **3** | `IO6` | **Joystick SW** | Digital input with internal pull-up |
| **4** | `IO16` | **Encoder CLK (A)** | Interruptible GPIO |
| **5** | `IO17` | **Encoder DT (B)** | Quadrature input |
| **6** | `IO18` | **Encoder Switch (SW)**| Digital input with internal pull-up |
| **7** | `IO21` | **Haptic Motor PWM** | Driven via NPN/MOSFET; 12 kHz LEDC PWM |
| **8** | `IO8` | **Button 2 (E-Stop)** | Active LOW, internal pull-up |
| **9** | `IO7` | **Button 1 (Trigger)**| Active LOW, internal pull-up (Dead-man switch) |
| **10**| `IO10` | *Unused / GPIO* | ADC1_CH9 |
| **11**| `IO20` | `USB_P` (D+) | Native USB |
| **12**| `IO19` | `USB_N` (D-) | Native USB |
| **13**| `GND` | **Ground** | Common system ground |
| **14**| `5V` | **5V Input** | USB / External 5V rail |

### 2. Header P2 (14-Pin) — Display Cluster
| Pin # | Net Name | Primary Role in Remote | Constraints / Notes |
| :--- | :--- | :--- | :--- |
| **1** | `3V3` | **3.3V Power** | Clean 3.3V power output |
| **2** | `GND` | **Ground** | Common system ground |
| **3** | `IO43` | `U0TXD` | Hardware Serial TX |
| **4** | `IO44` | `U0RXD` | Hardware Serial RX |
| **5** | `IO47` | `IMU_SCL` | Internal QMI8658 I2C SCL line |
| **6** | `IO48` | `IMU_SDA` | Internal QMI8658 I2C SDA line |
| **7** | `IO15` | *Unused / Free* | Free GPIO (7-pin ST7789 has CS tied to GND internally) |
| **8** | `IO13` | **ST7789 DC** | Display Data/Command line |
| **9** | `IO11` | **ST7789 MOSI (SDA)**| SPI Data Out |
| **10**| `IO12` | **ST7789 SCLK (SCL)**| SPI Clock |
| **11**| `IO14` | **ST7789 RST** | Display Reset (active LOW) |
| **12**| `IO9` | **ST7789 BL** | Backlight PWM control |
| **13**| `GND` | **Ground** | Display GND |
| **14**| `VBAT` | **Battery Rail** | Connected to LiPo battery positive terminal |

---

## Critical Engineering Rules

1. **ADC1 vs. ADC2 with Wi-Fi:**
   * Never connect analog joystick axes to ADC2 channels (GPIO 11–18) when ESP-NOW or Wi-Fi is active. ADC2 will return noisy or invalid readings during RF activity. Use **ADC1** (`GPIO 2`, `GPIO 4`).
2. **QMI8658 Internal Bus:**
   * Do not instantiate a second `Wire` instance on `GPIO 47/48`. Initialize the default `Wire` via `Wire.begin(48, 47, 400000)` (SDA=48, SCL=47).
3. **ST7789 240x240 Display Driver:**
   * Use `SPI.begin(LCD_SCLK_PIN, -1, LCD_MOSI_PIN, LCD_CS_PIN)`.
   * Initialize with `tft.init(240, 240)`.
4. **Rover ESP32 I2C Sensor Bus:**
   * MPU6050 and QMC5883L/HMC5883L share `GPIO 21` (SDA) and `GPIO 22` (SCL).
   * MPU6050 address = `0x68` (AD0 tied to GND).
   * QMC5883L address = `0x0D`, HMC5883L address = `0x1E`.
   * Complementary sensor fusion: $\text{Heading} = \text{GyroYaw} + 0.04 \times (\text{MagHeading} - \text{GyroYaw})$.

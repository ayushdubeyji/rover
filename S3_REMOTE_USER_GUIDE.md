# Waveshare ESP32-S3 Air Remote: User & Flashing Guide

This document covers how to compile, flash, and operate the **Waveshare ESP32-S3 Air Remote** and **Rover ESP32** firmware using the Arduino IDE.

---

## 1. Arduino IDE Setup & Required Libraries

### Board Selection
* **For S3 Remote (`s3_remote/`):**
  * Tools > Board > ESP32 Arduino > **ESP32S3 Dev Module**
  * USB CDC On Boot: **Enabled**
  * Flash Size: **16MB (128Mb)**
  * Partition Scheme: **16M Flash (3MB APP/9.9MB FATFS)** or **Default 4MB with spiffs**
  * PSRAM: **OPI PSRAM** (or Enabled)
* **For Rover (`rover esp/rover/`):**
  * Tools > Board > ESP32 Arduino > **ESP32 Dev Module**
  * Upload Speed: **921600**
  * Flash Size: **4MB**

### Required Arduino Libraries
Open **Tools > Manage Libraries...** (Ctrl+Shift+I) and install:
1. **Adafruit ST7735 and ST7789 Library** (by Adafruit) — for 1.3" 240×240 display.
2. **Adafruit GFX Library** (by Adafruit) — core graphics engine.
3. **Adafruit MPU6050** (by Adafruit) — for Rover IMU.
4. **Adafruit Unified Sensor** (by Adafruit) — dependency for MPU6050.

---

## 2. Remote UI & Controls Manual

### Screen Layout (1.3" ST7789 240×240 HUD)
```
+-------------------------------------------------------+
| [ROVER LINK OK]        -64dBm      [BAT 98% 4.12V]    |  <-- Status Bar
+-------------------------------------------------------+
|                                                       |
|   MODE: [FIELD-ORIENTED]          SPD: 80% [TURBO]    |  <-- Mode Banner
|                                                       |
|                 +-----------------+                   |
|                 |    \       /    |                   |
|                 |      \ O /      |  <-- Live Reticle |
|                 |     -- + --     |      & Artificial |
|                 |      /   \      |      Horizon      |
|                 |    /       \    |                   |
|                 +-----------------+                   |
|                                                       |
|   SRC: AIR-REMOTE [DEADMAN ON]    HDG: 142°           |  <-- Sensor State
+-------------------------------------------------------+
```

1. **Top Status Bar:**
   * **Link Status Pill:** Green `[ROVER LINK OK]` when receiving bidirectional telemetry ACKs; flashes red `[NO LINK]` if packets are lost.
   * **RSSI Indicator:** Real-time signal strength in dBm.
   * **Battery Gauge:** Displays remote LiPo battery voltage and remaining percentage based on onboard `GPIO 5` voltage divider.
2. **Mode & Speed Banner:**
   * Shows active drive mode (Normal, Crawl, Cruise, Sport, Field-Oriented, Heading Lock, Drift, Orbit, Tank).
   * Highlights `[TURBO]` when high-speed mode is engaged.
3. **Center Live HUD:**
   * **In Joystick Mode:** Crosshair dot dynamically moves in 2D space reflecting exact joystick physical displacement ($X, Y$).
   * **In Air Remote Mode:** Displays an **Artificial Horizon Pitch/Roll Bar** showing remote tilt angles in real time.
4. **Bottom Telemetry Bar:**
   * Current Control Source (`JOYSTICK` or `AIR-REMOTE`).
   * `[DEADMAN ON / OFF]` safety state.
   * `HDG:` Rover compass heading ($0^\circ - 360^\circ$) fused from MPU6050 + Magnetometer.

---

## 3. Operating Instructions

### A. Driving with Analog Joystick
1. Push Joystick forward/reverse for linear motion ($Y$).
2. Push Joystick left/right for sideways mecanum strafe ($X$).
3. Twist Rotary Encoder or tilt to add yaw rotation ($Z$).
4. Press **Button 1 (Trigger)** for Turbo boost (100% speed override).
5. Press **Button 2 (E-Stop)** to instantly halt all wheels and clear motion buffers.

---

### B. Driving with Air Remote (IMU Gesture Tilt)
1. In the menu, select **Control Source -> AIR REMOTE**.
2. **Dead-Man's Safety Trigger:** The remote will **only** transmit motion commands when you hold down the **Joystick Push Button (SW)** or **Button 1**.
   * If you release the button, the rover immediately glides to a stop.
3. **Tilt Actions:**
   * **Pitch Forward:** Remote tilts down -> Rover drives forward.
   * **Pitch Backward:** Remote tilts up -> Rover drives in reverse.
   * **Roll Left:** Remote tilts left -> Rover strafes sideways left.
   * **Roll Right:** Remote tilts right -> Rover strafes sideways right.
4. **Deadband Filter:** A $\pm 5^\circ$ central deadband absorbs natural hand tremors when resting.

---

### C. Menu Navigation (Rotary Encoder)
1. **Open / Close Menu:** Press the **Rotary Encoder Push Button (SW)** or long-press **Button 1**.
2. **Scroll Items:** Rotate the encoder clockwise/counter-clockwise. You will feel a crisp haptic tick on every detent.
3. **Select / Toggle:** Press the Rotary Encoder switch:
   * **1. Drive Mode:** Cycles between all 9 mecanum driving modes.
   * **2. Control Source:** Switches between `JOYSTICK` and `AIR REMOTE`.
   * **3. Speed Presets:** Sets cruise/crawl caps (15%, 35%, 60%, 100%).
   * **4. Stunts / Macros:** Triggers 360° Tornado Spin, 180° Quick Turn, J-Turn, or Orbit.
   * **5. Calibrate Zero:** Re-zeros IMU pitch/roll to your current hand-holding angle.
   * **6. Enable OTA:** Connects to Wi-Fi for wireless over-the-air firmware uploads.

---

### D. Rotary Encoder 360° Flick Maneuver
* **Slow Turn:** Adjusts speed or turns rover incrementally.
* **Rapid Flick (3+ clicks under 300ms):** Automatically sends a **360° spin command** to the rover in that direction!

---

## 4. Over-The-Air (OTA) Wireless Flashing

1. Select **Enable OTA** in the remote menu.
2. The remote connects to Wi-Fi (`REMOVED`) and displays its IP address on the 1.3" display.
3. In Arduino IDE, select **Tools > Port > Network Ports > Mecanum-Remote at <IP>**.
4. Click **Upload** to reflash without plugging in any USB cables!

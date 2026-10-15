# Quick Start & User Guide
## Waveshare ESP32-S3 Remote & Mecanum Rover

---

### 1. Project Directory Structure

```
C:\Users\dell\Desktop\rover\
├── WIRING_GUIDE.md           <- Comprehensive physical wiring tables & diagrams
├── HARDWARE_SCHEMATIC.md     <- Circuit interconnect schematics
├── QUICK_START.md            <- Flashing instructions, calibration & controls (this file)
│
├── s3 remote\                <- Remote firmware for Arduino IDE (contains "s3 remote.ino")
│   ├── s3 remote.ino
│   ├── pin_defs.h
│   ├── rover_proto.h
│   ├── qmi8658.h / .cpp
│   ├── display_ui.h / .cpp
│   └── haptics.h
│
├── s3_remote\                <- Mirror folder for CLI / build scripts (contains "s3_remote.ino")
│   ├── s3_remote.ino
│   └── (all supporting files identical to above)
│
├── rover esp\rover\          <- Rover ESP32 main controller firmware
│   ├── rover.ino             <- Sensor fusion, heading lock, ESP-NOW & web server
│   ├── rover_proto.h         <- Protocol packet definition
│   ├── compass.h             <- QMC5883L/HMC5883L dual driver with tilt compensation
│   └── dashboard.h           <- Embedded HTML5 responsive dashboard
│
└── rover rp2040\             <- RP2040 CircuitPython motor coprocessor
    └── code.py               <- 20kHz motor PWM & kinematics
```

---

### 2. Flashing Instructions

#### A. Flashing the Rover ESP32
1. Open Arduino IDE or run via CLI:
   ```powershell
   & "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32 --upload -p COM<PORT> "C:\Users\dell\Desktop\rover\rover esp\rover"
   ```
2. Verify in Serial Monitor (115200 baud):
   - `[IMU] MPU6050 ready.`
   - `[MAG] Magnetometer ready: QMC5883L` (or `HMC5883L`)
   - `MAC <MAC_ADDRESS> channel <CH>`

#### B. Flashing the S3 Remote Controller
1. Connect Waveshare ESP32-S3 via USB-C.
2. In Arduino IDE:
   - Board: **ESP32S3 Dev Module** (or `Waveshare ESP32S3 Touch LCD 128` / `esp32s3`)
   - Flash Size: **16MB (128Mb)**
   - Partition Scheme: **16M Flash (3MB APP/9.9MB FATFS)** or **Default 4MB with spiffs**
   - PSRAM: **OPI PSRAM**
3. Or compile via CLI:
   ```powershell
   & "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32s3 --upload -p COM<PORT> "C:\Users\dell\Desktop\rover\s3_remote"
   ```

---

### 3. Controls & Operational Guide

#### Live HUD Screen
- **Top Status Bar**: Shows Remote Battery Voltage (`4.1V`), Charge Icon, and Link State (`LINK OK` in green / `OFFLINE` in red).
- **Center Reticle**:
  * Crosshair circle displays the real-time drive vector dot.
  * Dot glows **green** during normal drive, **orange** during Turbo, and **cyan** when stationary.
  * Top mode pill displays active Drive Mode.
  * Bottom indicators show Rover Heading (`HD: 142 deg`) and Control Source (`SRC: JOYSTICK` vs `AIR: ACTIVE`).
- **Bottom Bar**: Shows current Speed Limit (`SPEED LIMIT: 60%`) and graphic slider.

#### Control Inputs
| Input Device | Action | Function |
|:---|:---|:---|
| **Joystick Thumbstick** | Push X / Y | Forward / Reverse / Strafe translation |
| **Rotary Knob (Rotate)** | Turn CW / CCW | Adjust Speed Limit up/down in 5% increments |
| **Rotary Knob (Click)** | Press Switch | Open / Close Interactive Settings Menu |
| **Button 1 (GPIO 7)** | Hold in Joy Mode | Engage **Turbo Mode** (100% max duty) |
| **Button 1 (GPIO 7)** | Hold in Air Mode | **Safety Deadman Trigger** (Allows tilt steering) |
| **Button 2 (GPIO 8)** | Press | **Emergency Stop** (Instant motor brake & reset) |

#### Air Remote Mode (Tilt Gestures)
1. Click the Rotary Knob to open the Menu.
2. Select `Control Input` and toggle to `AirTilt`.
3. Click `Exit Menu`.
4. Hold **Button 1** (Deadman Trigger):
   - Tilt remote forward -> Rover drives forward.
   - Tilt remote backward -> Rover reverses.
   - Tilt remote left/right -> Rover strafes sideways.
5. Release Button 1 -> Rover halts immediately for safety.

#### Drive Modes
- **Normal**: Smooth cubic expo drive response.
- **Crawl**: Low-speed precision maneuvering (20% cap).
- **Cruise**: Constant directional heading speed.
- **Sport**: Maximum acceleration with crisp response.
- **Field-Oriented**: Heading decoupled from rover nose! Joystick forward drives toward room North regardless of rotation.
- **Heading Lock**: Closed-loop gyroscope/compass PID maintains exact directional line even over carpet bumps or wheel slip!
- **Drift**: Dynamic rear-slip for aggressive sliding turns.
- **Orbit**: Circular rotation around a fixed focal radius.
- **Tank**: Skid-steer rotation mode.

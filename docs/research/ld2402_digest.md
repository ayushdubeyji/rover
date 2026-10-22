# HLK-LD2402 24GHz Millimeter-Wave Radar Sensor Digest

## 1. Electrical & Physical Specifications
| Parameter | Specification | Notes |
| :--- | :--- | :--- |
| **Frequency** | 24.00 GHz – 24.25 GHz (ISM Band) | FMCW sweep bandwidth 0.25 GHz |
| **Supply Voltage** | 3.0 V – 3.6 V (Typ. 3.3 V) | 4.5 V – 5.5 V if onboard LDO fitted |
| **Operating Current**| 50 mA average | Low power consumption |
| **Logic Levels** | 3.3 V TTL | Direct ESP32 / RP2040 / Pi compatibility |
| **Dimensions** | 20 mm x 20 mm x 1.2 mm PCB | 2.54 mm & 2.0 mm compatible pin pitch |
| **Refresh Period** | ~165 ms | Real-time reporting rate |
| **EIRP** | 11 dBm | Complies with FCC/CE regulations |

## 2. Pinout & Interfacing (J2 Connector)
| Pin # | Label | Type | Function | Electrical Limits |
| :--- | :--- | :--- | :--- | :--- |
| **J2-1**| **V** | Power | DC Power Input (3.3V typ, 3.0–3.6V / 5V LDO) | 3.0V – 3.6V (or 5V) |
| **J2-2**| **IO**| Digital OUT | Presence Output: HIGH (3.3V) = Present, LOW (0V) = Empty | 0V / 3.3V CMOS |
| **J2-3**| **G** | Power | Ground / Reference Plane | 0V |
| **J2-4**| **T** | UART TX | Radar Data Output -> MCU RX | 3.3V TTL |
| **J2-5**| **R** | UART RX | Radar Command Input <- MCU TX | 3.3V TTL |
*(J1 SWD header: Pin 1=GND, Pin 2=DIO, Pin 3=CLK, Pin 4=3V3 for factory burning/debugging)*

## 3. UART Serial Interface & Framing
- **Baud Rate**: 115200 bps | **Data Bits**: 8 | **Stop Bits**: 1 | **Parity**: None (8N1)
- **Byte Order**: Little-Endian (LSB first) for all multi-byte values

### A. Command & ACK Frame Structure
- **Command Frame**: `[FD FC FB FA]` + `[Len (2B)]` + `[Cmd (2B)]` + `[Payload (N B)]` + `[04 03 02 01]`
- **ACK Frame**: `[FD FC FB FA]` + `[Len (2B)]` + `[Cmd|0x0100 (2B)]` + `[Status (2B: 0=OK, 1=ERR)]` + `[Data]` + `[04 03 02 01]`

### B. Periodic Data Reporting Frame (Engineering Mode 0x0004)
- **Header**: `F4 F3 F2 F1` (4 bytes)
- **Length**: `83 00` (131 bytes little-endian)
- **Target State**: `00` = Nobody, `01` = Moving target present, `02` = Stationary / micro-motion target present (1 byte)
- **Target Distance**: 2 bytes little-endian uint16 in **cm** (e.g. `66 00` = 102 cm)
- **Gate Energy Spectrum**: 128 bytes (32 range gates x 4 bytes uint32 LE per gate; gate resolution ~0.33 m)
- **Tail**: `F8 F7 F6 F5` (4 bytes)

### C. Normal Working Mode Data
- Text mode output: string `"OFF"` (no target) or `"target distance : XX"` (target present, distance in cm)

## 4. Core Command Set
| Command | Cmd Word | Payload | Description |
| :--- | :--- | :--- | :--- |
| **Enable Config** | `0x00FF` | `0x0001` | Unlock config mode (mandatory before parameter writes) |
| **End Config** | `0x00FE` | None | Exit config mode; resume normal radar operation |
| **Read FW Version**| `0x0000` | None | Returns version string (e.g., `v3.0.3`) |
| **Read Parameters**| `0x0008` | `[ParamID (2B)]` | Reads target parameter ID |
| **Write Parameter**| `0x0007` | `[ParamID (2B)] [Val (4B)]` | Sets specified parameter register |
| **Set Output Mode**| `0x0012` | `0x0000 [Mode (4B)]` | Mode: `0x00000004` (Engineering), `0x00000064` (Normal) |
| **Save Parameters**| `0x00FD` | None | Persists configuration into internal flash (v3.3.2+) |
| **Auto-Gain Adjust**| `0x00EE`| None | Calibrates receiver gain against radome/chassis saturation |

## 5. Configuration Registers (Parameter IDs)
| Param ID | Register Name | Range | Description |
| :--- | :--- | :--- | :--- |
| `0x0001` | Maximum Distance | `7` – `100` | Max detection distance gate (0.7 m to 10.0 m, step 0.1 m) |
| `0x0004` | Disappearance Delay| `0` – `65535` | Hold time in seconds after target leaves before reporting empty |
| `0x0010`–`0x001F` | Motion Thresholds | `0` – `95` | Motion trigger sensitivity for range gates 0 to 15 ($N = 10 \log_{10} M$) |
| `0x0030`–`0x003F` | Micro-motion Thresh | `0` – `95` | Stationary/breathing trigger sensitivity for gates 0 to 15 |
| `0x0005` | Power Interference | Read-only | 0: Untested, 1: Clean, 2: Power rail noise detected |

## 6. Detection Field of View (FOV) & Rover Forward Mounting
| Specification | Value | Context |
| :--- | :--- | :--- |
| **Horizontal FOV (Azimuth)** | ±60° (120° total) | Symmetrical along rover forward X-axis |
| **Vertical FOV (Elevation)** | ±60° (120° total) | Conical cone centered on radar Y normal |
| **Max Motion Range** | 10.0 m | Moving human body |
| **Max Micro-Motion Range** | 6.0 m | Slight movements / gestures |
| **Max Stationary Range** | 5.0 m | Seated / breathing static human presence |
| **Ranging Accuracy** | ±0.15 m | Within 6 m direct line of sight |
| **Rover Mounting Attitude** | Vertical normal forward | Face radar patch antenna towards rover front (+x) |
| **Radome Clearance** | > 10 mm | Non-metallic plastic enclosure (ABS/PLA, no carbon-fiber) |

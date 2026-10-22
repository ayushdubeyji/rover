#pragma once
#include <Arduino.h>

// ==============================================================================
// Waveshare RP2040-Zero Ground Truth Pin Map
// ==============================================================================

// --- UART0: ESP32-C3 ESP-NOW Telemetry Bridge ---
#define PIN_UART0_TX            0   // GP0 -> C3 RX
#define PIN_UART0_RX            1   // GP1 <- C3 TX
#define ROVER_UART0_BAUD        921600
#define ROVER_UART0_BAUD_ALT    460800

// --- I2C1: Shared Sensor Bus (400 kHz) ---
// Connected: MPU6050 (0x68/0x69), Compass (0x30/0x0D/0x1E), Front ToF (0x32), Rear ToF (0x33)
#define PIN_I2C1_SDA            2   // GP2 (I2C1 SDA)
#define PIN_I2C1_SCL            3   // GP3 (I2C1 SCL)
#define ROVER_I2C_FREQ          400000

// --- UART1: Raspberry Pi 3B+ Host Link ---
#define PIN_UART1_TX            4   // GP4 -> Pi 3B+ RX (/dev/serial0)
#define PIN_UART1_RX            5   // GP5 <- Pi 3B+ TX (/dev/serial0)
#define ROVER_UART1_BAUD        115200

// --- Dual VL53L0X XSHUT Power Enable Pins ---
#define PIN_TOF_FRONT_XSHUT     6   // GP6 -> Front VL53L0X (0 deg @ 11cm fwd, addr 0x32)
#define PIN_TOF_REAR_XSHUT      7   // GP7 -> Rear VL53L0X (180 deg @ 10cm back, addr 0x33)
#define TOF_ADDR_FRONT          0x32
#define TOF_ADDR_REAR           0x33

// --- Dual PWM Motor Drive Pins (20 kHz) ---
// Identical mapping to CircuitPython code.py
// FL (Motor 0): Forward GP9, Backward GP8
#define PIN_MOTOR_FL_BCK        8   // GP8  (PWM4 A)
#define PIN_MOTOR_FL_FWD        9   // GP9  (PWM4 B)

// RL (Motor 2): Forward GP11, Backward GP10
#define PIN_MOTOR_RL_BCK        10  // GP10 (PWM5 A)
#define PIN_MOTOR_RL_FWD        11  // GP11 (PWM5 B)

// FR (Motor 1): Forward GP12, Backward GP13
#define PIN_MOTOR_FR_FWD        12  // GP12 (PWM6 A)
#define PIN_MOTOR_FR_BCK        13  // GP13 (PWM6 B)

// RR (Motor 3): Forward GP14, Backward GP15
#define PIN_MOTOR_RR_FWD        14  // GP14 (PWM7 A)
#define PIN_MOTOR_RR_BCK        15  // GP15 (PWM7 B)

// PWM Specifications
#define MOTOR_PWM_FREQ          20000
#define MOTOR_PWM_MAX_DUTY      65535

// --- Analog, Radar, and Pulse Counter Pins ---
#define PIN_BATTERY_ADC         26  // GP26 / ADC0 (External divider 200k/100k, Vbat = Vadc * 3.0)
#define BATTERY_DIVIDER_RATIO   3.0f

#define PIN_RADAR_OUT           27  // GP27 (HLK-LD2402 Radar presence OUT, HIGH = motion)
#define PIN_ENCODER_OPTIONAL    28  // GP28 (Optional single-encoder pulse counter, disabled by default)
#define PIN_SPARE               29  // GP29 (Spare ADC3 / GPIO)

// Onboard WS2812 RGB LED on Waveshare RP2040-Zero
#define PIN_WS2812_ONBOARD      16

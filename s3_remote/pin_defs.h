#pragma once
#include <Arduino.h>

// =============================================================================
// PIN DEFINITIONS: Waveshare ESP32-S3-Touch-LCD-2 Remote Controller
// =============================================================================

// --- 1.3-inch ST7789 SPI Display (Clustered on Expansion Header P2) ---
#define LCD_MOSI_PIN    11   // P2 Pin 9  (SDA / MOSI)
#define LCD_SCLK_PIN    12   // P2 Pin 10 (SCL / SCLK)
#define LCD_DC_PIN      13   // P2 Pin 8  (DC / Data-Command)
#define LCD_RST_PIN     14   // P2 Pin 11 (RES / Reset)
#define LCD_CS_PIN      -1   // No CS pin on display (tied to GND internally on 7-pin module)
#define LCD_BL_PIN      9    // P2 Pin 12 (BLK / Backlight PWM, ADC1_CH8)
#define LCD_WIDTH       240
#define LCD_HEIGHT      240

// --- Analog Joystick (Header P1: strictly ADC1 channels for Wi-Fi immunity) ---
#define JOY_X_PIN       2    // P1 Pin 1  (ADC1_CH1)
#define JOY_Y_PIN       4    // P1 Pin 2  (ADC1_CH3)
#define JOY_SW_PIN      6    // P1 Pin 3  (Active LOW, internal pull-up)

// --- Rotary Encoder (Header P1) ---
#define ENC_CLK_PIN     16   // P1 Pin 4  (Encoder Phase A - onboard 4.7k pullup)
#define ENC_DT_PIN      17   // P1 Pin 5  (Encoder Phase B - onboard 10k pulldown R6; needs ext 1k-2.2k pullup or IO10)
#define ENC_DT_ALT_PIN  10   // P1 Pin 10 (Encoder Phase B Alternate - Clean GPIO without onboard pulldown)
#define ENC_SW_PIN      18   // P1 Pin 6  (Encoder Push Switch, Active LOW)

// --- Push Buttons to GND (Header P1) ---
#define BTN_BRAKE_PIN    7   // P1 Pin 9  (Button 1: Active Brake, Active LOW)
#define BTN_THROTTLE_PIN 8   // P1 Pin 8  (Button 2: Full Throttle / Boost, Active LOW)
#define BTN1_TRIGGER_PIN BTN_BRAKE_PIN
#define BTN2_ESTOP_PIN   BTN_THROTTLE_PIN

// --- Haptic Feedback (Header P1) ---
#define HAPTIC_PWM_PIN  21   // P1 Pin 7  (LEDC PWM via Transistor/FET driver)

// --- Onboard Hardware (Hardwired on PCB) ---
#define QMI_SCL_PIN     47   // Onboard I2C SCL (also accessible on P2 Pin 5)
#define QMI_SDA_PIN     48   // Onboard I2C SDA (also accessible on P2 Pin 6)
#define QMI_INT1_PIN    3    // Onboard QMI8658 Interrupt 1
#define QMI_I2C_ADDR    0x6B // Onboard QMI8658 6-axis IMU I2C Address

#define BATTERY_ADC_PIN 5    // Onboard VBAT Divider (ADC1_CH4, 200k/100k = 3.0x multiplier)

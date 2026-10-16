#pragma once
#include <Arduino.h>
#include <Wire.h>

class QMI8658Driver {
public:
  bool isConnected = false;
  float ax = 0, ay = 0, az = 0; // Accelerations in g
  float gx = 0, gy = 0, gz = 0; // Angular velocities in dps
  float pitch = 0;              // Degrees (-90 to +90)
  float roll = 0;               // Degrees (-90 to +90)
  
  // Calibration offsets
  float pitchZero = 0;
  float rollZero = 0;
  float gxZero = 0;
  float gyZero = 0;
  float gzZero = 0;

  bool begin(uint8_t sdaPin, uint8_t sclPin, uint8_t addr = 0x6B);
  bool readSensor();
  void calibrateZero();
  
  // Get normalized tilt commands (-100 to +100) for Mecanum rover control
  // pitch -> Forward/Backward (Y)
  // roll  -> Strafe (X)
  void getDriveTilt(int8_t &xOut, int8_t &yOut, float deadbandDeg = 6.0f, float maxAngleDeg = 35.0f);

private:
  uint8_t _addr = 0x6B;
  void writeReg(uint8_t reg, uint8_t val);
  uint8_t readReg(uint8_t reg);
};

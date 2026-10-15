#include "qmi8658.h"
#include <math.h>

#define REG_WHO_AM_I   0x00
#define REG_CTRL1      0x02
#define REG_CTRL2      0x03
#define REG_CTRL3      0x04
#define REG_CTRL5      0x06
#define REG_CTRL7      0x08
#define REG_STATUS0    0x2D
#define REG_DATA_START 0x35

void QMI8658Driver::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t QMI8658Driver::readReg(uint8_t reg) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(_addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

bool QMI8658Driver::begin(uint8_t sdaPin, uint8_t sclPin, uint8_t addr) {
  _addr = addr;
  Wire.begin(sdaPin, sclPin, 400000);
  delay(20);

  uint8_t who = readReg(REG_WHO_AM_I);
  if (who != 0x05) {
    // If not found at 0x6B, check 0x6A
    if (_addr == 0x6B) {
      _addr = 0x6A;
      who = readReg(REG_WHO_AM_I);
    }
  }

  if (who != 0x05) {
    isConnected = false;
    return false;
  }

  // Address auto-increment enable
  writeReg(REG_CTRL1, 0x60);
  // Accel: ±4g range (0x01 << 4), 100Hz ODR (0x04) -> 0x14
  writeReg(REG_CTRL2, 0x14);
  // Gyro: ±512 dps (0x05 << 4), 100Hz ODR (0x04) -> 0x54
  writeReg(REG_CTRL3, 0x54);
  // LPF filter on accel & gyro
  writeReg(REG_CTRL5, 0x11);
  // Enable Accel and Gyro continuous mode
  writeReg(REG_CTRL7, 0x03);

  isConnected = true;
  delay(20);
  calibrateZero();
  return true;
}

bool QMI8658Driver::readSensor() {
  if (!isConnected) return false;

  Wire.beginTransmission(_addr);
  Wire.write(REG_DATA_START);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(_addr, (uint8_t)12) != 12) return false;

  int16_t rawAx = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawAy = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawAz = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawGx = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawGy = (int16_t)(Wire.read() | (Wire.read() << 8));
  int16_t rawGz = (int16_t)(Wire.read() | (Wire.read() << 8));

  // Scale: ±4g = 8192 LSB/g; ±512 dps = 64 LSB/dps
  ax = (float)rawAx / 8192.0f;
  ay = (float)rawAy / 8192.0f;
  az = (float)rawAz / 8192.0f;

  gx = (float)rawGx / 64.0f;
  gy = (float)rawGy / 64.0f;
  gz = (float)rawGz / 64.0f;

  // Angles in degrees (Rotation 2: 180 deg inverted handheld screen)
  // Screen facing user -> resting gravity pulls along -az (~ -1.0g).
  // Using -az as positive vertical axis prevents +/-180 deg branch cut jump at rest.
  // Pitch (forward tilt, top down) = +ax; Roll (right bank tilt) = -ay
  float p = atan2f(ax, -az) * (180.0f / (float)M_PI);
  float r = atan2f(-ay, -az) * (180.0f / (float)M_PI);

  pitch = p - pitchZero;
  roll = r - rollZero;
  return true;
}

void QMI8658Driver::calibrateZero() {
  if (!isConnected) return;
  float sumP = 0, sumR = 0, sumGz = 0;
  for (int i = 0; i < 40; i++) {
    readSensor();
    float p = atan2f(ax, -az) * (180.0f / (float)M_PI);
    float r = atan2f(-ay, -az) * (180.0f / (float)M_PI);
    sumP += p;
    sumR += r;
    sumGz += gz;
    delay(4);
  }
  pitchZero = sumP / 40.0f;
  rollZero = sumR / 40.0f;
  gzZero = sumGz / 40.0f;
}

void QMI8658Driver::getDriveTilt(int8_t &xOut, int8_t &yOut, float deadbandDeg, float maxAngleDeg) {
  readSensor();

  // Pitch controls Forward/Backward (Y axis)
  // Tilting remote forward tilts nose down: pitch decreases or increases depending on orientation.
  // Standard handheld orientation:
  // Forward tilt -> Positive Y (Forward)
  // Backward tilt -> Negative Y (Reverse)
  // Right tilt -> Positive X (Right strafe)
  // Left tilt -> Negative X (Left strafe)
  float p = pitch;
  float r = roll;

  // Apply deadband
  if (fabsf(p) < deadbandDeg) p = 0;
  else p = (p > 0 ? 1 : -1) * (fabsf(p) - deadbandDeg) / (maxAngleDeg - deadbandDeg);

  if (fabsf(r) < deadbandDeg) r = 0;
  else r = (r > 0 ? 1 : -1) * (fabsf(r) - deadbandDeg) / (maxAngleDeg - deadbandDeg);

  p = constrain(p, -1.0f, 1.0f);
  r = constrain(r, -1.0f, 1.0f);

  // Map to [-100, 100]
  yOut = (int8_t)lroundf(p * 100.0f);
  xOut = (int8_t)lroundf(r * 100.0f);
}

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

  // 3D Euclidean tilt angles for handheld screen (works whether held flat OR vertically at any resting angle):
  // Roll (left/right steering tilt): atan2(ax, sqrt(ay^2 + az^2))
  // Pitch (forward/backward throttle tilt): atan2(-ay, -az) when flat, or full 3D pitch relative to calibrated resting gravity vector
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm < 0.1f) norm = 1.0f;
  float nAx = ax / norm, nAy = ay / norm, nAz = az / norm;

  // Pitch: forward/back tilt in the Y-Z plane
  float p = atan2f(-nAy, -nAz) * (180.0f / (float)M_PI);
  // Roll: left/right bank tilt across the X axis
  float r = asinf(constrain(nAx, -1.0f, 1.0f)) * (180.0f / (float)M_PI);

  float dp = p - pitchZero;
  while (dp > 180.0f) dp -= 360.0f;
  while (dp < -180.0f) dp += 360.0f;
  pitch = dp;
  roll = r - rollZero;
  return true;
}

void QMI8658Driver::calibrateZero() {
  if (!isConnected) return;
  float sumAx = 0, sumAy = 0, sumAz = 0, sumGx = 0, sumGy = 0, sumGz = 0;
  for (int i = 0; i < 40; i++) {
    readSensor();
    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;
    delay(4);
  }
  float mAx = sumAx / 40.0f, mAy = sumAy / 40.0f, mAz = sumAz / 40.0f;
  float norm = sqrtf(mAx * mAx + mAy * mAy + mAz * mAz);
  if (norm < 0.1f) norm = 1.0f;
  pitchZero = atan2f(-mAy / norm, -mAz / norm) * (180.0f / (float)M_PI);
  rollZero = asinf(constrain(mAx / norm, -1.0f, 1.0f)) * (180.0f / (float)M_PI);
  gxZero = sumGx / 40.0f;
  gyZero = sumGy / 40.0f;
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
  float r = -roll; // Negate roll so tilting right produces positive X (Right strafe/turn)

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

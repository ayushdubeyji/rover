#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

enum MagChipType {
  MAG_NONE = 0,
  MAG_QMC5883L,
  MAG_HMC5883L,
  MAG_AK8963,
  MAG_LIS3MDL,
  MAG_IST8310,
  MAG_MMC5603
};

class RoverCompass {
public:
  MagChipType chip = MAG_NONE;
  float magX = 0, magY = 0, magZ = 0;
  
  // Hard-iron calibration offsets & scale
  float xMin = -500, xMax = 500;
  float yMin = -500, yMax = 500;
  float zMin = -500, zMax = 500;
  bool calMode = false;

  bool begin() {
    // 1. Try QMC5883L (0x0D)
    Wire.beginTransmission(0x0D);
    if (Wire.endTransmission() == 0) {
      // Soft reset
      writeReg(0x0D, 0x0A, 0x80);
      delay(15);
      // Set/Reset Period
      writeReg(0x0D, 0x0B, 0x01);
      // Mode: Continuous, ODR: 200Hz, RNG: 8G, OSR: 512
      writeReg(0x0D, 0x09, 0x1D);
      chip = MAG_QMC5883L;
      return true;
    }

    // 2. Try HMC5883L (0x1E)
    Wire.beginTransmission(0x1E);
    if (Wire.endTransmission() == 0) {
      writeReg(0x1E, 0x00, 0x70); // 8-average, 15 Hz
      writeReg(0x1E, 0x01, 0x20); // Gain 1.3 Ga
      writeReg(0x1E, 0x02, 0x00); // Continuous measurement
      chip = MAG_HMC5883L;
      return true;
    }

    // 3. Try AK8963 (0x0C) - MPU-9250 internal magnetometer via I2C bypass
    Wire.beginTransmission(0x0C);
    if (Wire.endTransmission() == 0) {
      // Power down mode before mode change
      writeReg(0x0C, 0x0A, 0x00);
      delay(10);
      // Mode 2: 16-bit output, 100Hz continuous measurement
      writeReg(0x0C, 0x0A, 0x16);
      delay(10);
      chip = MAG_AK8963;
      return true;
    }

    // 4. Try LIS3MDL (0x1C)
    Wire.beginTransmission(0x1C);
    if (Wire.endTransmission() == 0) {
      writeReg(0x1C, 0x20, 0x70); // UHP mode, 10 Hz
      writeReg(0x1C, 0x21, 0x00); // +/- 4 gauss
      writeReg(0x1C, 0x22, 0x00); // Continuous-conversion mode
      chip = MAG_LIS3MDL;
      return true;
    }

    // 5. Try IST8310 (0x0E)
    Wire.beginTransmission(0x0E);
    if (Wire.endTransmission() == 0) {
      writeReg(0x0E, 0x0A, 0x0B); // Continuous mode, 100Hz
      chip = MAG_IST8310;
      return true;
    }

    // 6. Try MMC5603 (0x30)
    Wire.beginTransmission(0x30);
    if (Wire.endTransmission() == 0) {
      writeReg(0x30, 0x1B, 0x01); // Take measurement / continuous mode
      chip = MAG_MMC5603;
      return true;
    }

    chip = MAG_NONE;
    return false;
  }

  bool readRaw(int16_t &rx, int16_t &ry, int16_t &rz) {
    if (chip == MAG_QMC5883L) {
      Wire.beginTransmission(0x0D);
      Wire.write(0x00);
      if (Wire.endTransmission() != 0) return false;
      if (Wire.requestFrom(0x0D, 6) != 6) return false;
      rx = (int16_t)(Wire.read() | (Wire.read() << 8));
      ry = (int16_t)(Wire.read() | (Wire.read() << 8));
      rz = (int16_t)(Wire.read() | (Wire.read() << 8));
      return true;
    } else if (chip == MAG_HMC5883L) {
      Wire.beginTransmission(0x1E);
      Wire.write(0x03);
      if (Wire.endTransmission() != 0) return false;
      if (Wire.requestFrom(0x1E, 6) != 6) return false;
      rx = (int16_t)((Wire.read() << 8) | Wire.read());
      rz = (int16_t)((Wire.read() << 8) | Wire.read()); // HMC has Z before Y
      ry = (int16_t)((Wire.read() << 8) | Wire.read());
      return true;
    } else if (chip == MAG_AK8963) {
      Wire.beginTransmission(0x0C);
      Wire.write(0x02); // ST1 status register
      if (Wire.endTransmission() != 0) return false;
      if (Wire.requestFrom(0x0C, 1) != 1) return false;
      uint8_t st1 = Wire.read();
      if (!(st1 & 0x01)) return false; // Data ready check

      Wire.beginTransmission(0x0C);
      Wire.write(0x03); // HXL
      if (Wire.endTransmission() != 0) return false;
      if (Wire.requestFrom(0x0C, 7) != 7) return false; // 6 data bytes + 1 ST2 byte
      rx = (int16_t)(Wire.read() | (Wire.read() << 8));
      ry = (int16_t)(Wire.read() | (Wire.read() << 8));
      rz = (int16_t)(Wire.read() | (Wire.read() << 8));
      Wire.read(); // ST2 read releases data register for next measurement
      return true;
    } else if (chip == MAG_LIS3MDL) {
      Wire.beginTransmission(0x1C);
      Wire.write(0x28 | 0x80); // OUT_X_L with auto-increment
      if (Wire.endTransmission() != 0) return false;
      if (Wire.requestFrom(0x1C, 6) != 6) return false;
      rx = (int16_t)(Wire.read() | (Wire.read() << 8));
      ry = (int16_t)(Wire.read() | (Wire.read() << 8));
      rz = (int16_t)(Wire.read() | (Wire.read() << 8));
      return true;
    } else if (chip == MAG_IST8310) {
      Wire.beginTransmission(0x0E);
      Wire.write(0x03);
      if (Wire.endTransmission() != 0) return false;
      if (Wire.requestFrom(0x0E, 6) != 6) return false;
      rx = (int16_t)(Wire.read() | (Wire.read() << 8));
      ry = (int16_t)(Wire.read() | (Wire.read() << 8));
      rz = (int16_t)(Wire.read() | (Wire.read() << 8));
      return true;
    } else if (chip == MAG_MMC5603) {
      Wire.beginTransmission(0x30);
      Wire.write(0x00);
      if (Wire.endTransmission(false) != 0) return false;
      if (Wire.requestFrom((uint8_t)0x30, (uint8_t)6) != 6) return false;
      uint8_t mx_h = Wire.read(), mx_l = Wire.read();
      uint8_t my_h = Wire.read(), my_l = Wire.read();
      uint8_t mz_h = Wire.read(), mz_l = Wire.read();
      rx = (int16_t)(((uint16_t)(mx_h << 8) | mx_l) - 32768);
      ry = (int16_t)(((uint16_t)(my_h << 8) | my_l) - 32768);
      rz = (int16_t)(((uint16_t)(mz_h << 8) | mz_l) - 32768);
      // Trigger next measurement
      writeReg(0x30, 0x1B, 0x01);
      return true;
    }
    return false;
  }

  bool update() {
    int16_t rx, ry, rz;
    if (!readRaw(rx, ry, rz)) return false;

    // Track min/max for auto-calibration
    if (calMode) {
      if (rx < xMin) xMin = rx;
      if (rx > xMax) xMax = rx;
      if (ry < yMin) yMin = ry;
      if (ry > yMax) yMax = ry;
      if (rz < zMin) zMin = rz;
      if (rz > zMax) zMax = rz;
    }

    // Hard-iron correction (center)
    float xOffset = (xMax + xMin) / 2.0f;
    float yOffset = (yMax + yMin) / 2.0f;
    float zOffset = (zMax + zMin) / 2.0f;

    // Soft-iron scale factor
    float xDelta = (xMax - xMin) / 2.0f;
    float yDelta = (yMax - yMin) / 2.0f;
    float zDelta = (zMax - zMin) / 2.0f;
    float avgDelta = (xDelta + yDelta + zDelta) / 3.0f;
    if (avgDelta < 1.0f) avgDelta = 1.0f;

    magX = (rx - xOffset) * (avgDelta / (xDelta > 0 ? xDelta : 1.0f));
    magY = (ry - yOffset) * (avgDelta / (yDelta > 0 ? yDelta : 1.0f));
    magZ = (rz - zOffset) * (avgDelta / (zDelta > 0 ? zDelta : 1.0f));
    return true;
  }

  // Calculate tilt-compensated heading in degrees (-180 to +180)
  // ax, ay, az are accelerometer values in m/s^2 or g
  float getHeading(float ax, float ay, float az) {
    // Roll (phi) and Pitch (theta)
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    float sinR = sinf(roll);
    float cosR = cosf(roll);
    float sinP = sinf(pitch);
    float cosP = cosf(pitch);

    // Tilt-compensated horizontal magnetic components
    float Xh = magX * cosP + magY * sinR * sinP + magZ * cosR * sinP;
    float Yh = magY * cosR - magZ * sinR;

    float headingRad = atan2f(-Yh, Xh); // Standard compass convention
    float headingDeg = headingRad * 180.0f / (float)M_PI;
    if (headingDeg > 180.0f) headingDeg -= 360.0f;
    if (headingDeg < -180.0f) headingDeg += 360.0f;
    return headingDeg;
  }

private:
  void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
};

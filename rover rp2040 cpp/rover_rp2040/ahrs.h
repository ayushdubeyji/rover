#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "pins.h"
#include "rover_link.h"

// ==============================================================================
// 1. Multi-Chip Auto-Probing Magnetometer Driver
// Probes MMC5603 (0x30), QMC5883L (0x0D), HMC5883L (0x1E)
// ==============================================================================

enum MagChipType {
  MAG_NONE = 0,
  MAG_QMC5883L,
  MAG_HMC5883L,
  MAG_MMC5603
};

class RoverCompass {
public:
  MagChipType chip = MAG_NONE;
  float magX = 0, magY = 0, magZ = 0;
  float xMin = -500, xMax = 500;
  float yMin = -500, yMax = 500;
  float zMin = -500, zMax = 500;
  int16_t lastRx = 0, lastRy = 0, lastRz = 0;
  uint32_t lastReadMs = 0;

  bool begin(TwoWire &wire) {
    // 1. Try QMC5883L (0x0D)
    wire.beginTransmission(0x0D);
    if (wire.endTransmission() == 0) {
      writeReg(wire, 0x0D, 0x0A, 0x80); // soft reset
      delay(15);
      writeReg(wire, 0x0D, 0x0B, 0x01); // set/reset period
      writeReg(wire, 0x0D, 0x09, 0x1D); // Continuous, 200Hz, 8G, OSR 512
      chip = MAG_QMC5883L;
      return true;
    }

    // 2. Try HMC5883L (0x1E)
    wire.beginTransmission(0x1E);
    if (wire.endTransmission() == 0) {
      writeReg(wire, 0x1E, 0x00, 0x70); // 8-average, 15 Hz
      writeReg(wire, 0x1E, 0x01, 0x20); // Gain 1.3 Ga
      writeReg(wire, 0x1E, 0x02, 0x00); // Continuous measurement
      chip = MAG_HMC5883L;
      return true;
    }

    // 3. Try MMC5603 (0x30)
    // Verify it is not VL53L0X (reg 0xC0 != 0xEE) and Product ID reg 0x39 == 0x10
    wire.beginTransmission(0x30);
    wire.write(0xC0);
    uint8_t c0 = 0;
    if (wire.endTransmission(false) == 0 && wire.requestFrom((uint8_t)0x30, (uint8_t)1) == 1) {
      c0 = wire.read();
    }
    if (c0 != 0xEE) {
      wire.beginTransmission(0x30);
      wire.write(0x39);
      if (wire.endTransmission(false) == 0 && wire.requestFrom((uint8_t)0x30, (uint8_t)1) == 1) {
        uint8_t pid = wire.read();
        if (pid == 0x10) {
          writeReg(wire, 0x30, 0x1C, 0x80); // Software reset
          delay(20);
          writeReg(wire, 0x30, 0x1B, 0x08); // Do_Set (degauss sensor bridge)
          delay(5);
          writeReg(wire, 0x30, 0x1A, 100);  // ODR = 100Hz
          writeReg(wire, 0x30, 0x1B, 0xA0); // Auto_SR_en + Cmm_freq_en
          writeReg(wire, 0x30, 0x1D, 0x10); // Cmm_en continuous mode
          delay(10);
          chip = MAG_MMC5603;
          return true;
        }
      }
    }

    chip = MAG_NONE;
    return false;
  }

  bool readRaw(TwoWire &wire, int16_t &rx, int16_t &ry, int16_t &rz) {
    if (chip == MAG_QMC5883L) {
      wire.beginTransmission(0x0D);
      wire.write(0x00);
      if (wire.endTransmission() != 0) return false;
      if (wire.requestFrom((uint8_t)0x0D, (uint8_t)6) != 6) return false;
      rx = (int16_t)(wire.read() | (wire.read() << 8));
      ry = (int16_t)(wire.read() | (wire.read() << 8));
      rz = (int16_t)(wire.read() | (wire.read() << 8));
      return true;
    } else if (chip == MAG_HMC5883L) {
      wire.beginTransmission(0x1E);
      wire.write(0x03);
      if (wire.endTransmission() != 0) return false;
      if (wire.requestFrom((uint8_t)0x1E, (uint8_t)6) != 6) return false;
      rx = (int16_t)((wire.read() << 8) | wire.read());
      rz = (int16_t)((wire.read() << 8) | wire.read()); // HMC has Z before Y
      ry = (int16_t)((wire.read() << 8) | wire.read());
      return true;
    } else if (chip == MAG_MMC5603) {
      uint32_t now = millis();
      wire.beginTransmission(0x30);
      wire.write(0x18);
      if (wire.endTransmission(false) == 0 && wire.requestFrom((uint8_t)0x30, (uint8_t)1) == 1) {
        uint8_t st = wire.read();
        if (!(st & 0x40)) {
          if (now - lastReadMs > 150) {
            writeReg(wire, 0x30, 0x1B, 0xA0);
            writeReg(wire, 0x30, 0x1D, 0x10);
          }
          if (lastRx != 0 || lastRy != 0) {
            rx = lastRx; ry = lastRy; rz = lastRz;
            return true;
          }
          return false;
        }
      }
      wire.beginTransmission(0x30);
      wire.write(0x00);
      if (wire.endTransmission(false) != 0) return false;
      if (wire.requestFrom((uint8_t)0x30, (uint8_t)6) != 6) return false;
      uint16_t mx_u = ((uint16_t)wire.read() << 8) | wire.read();
      uint16_t my_u = ((uint16_t)wire.read() << 8) | wire.read();
      uint16_t mz_u = ((uint16_t)wire.read() << 8) | wire.read();
      int16_t curX = (int16_t)((int32_t)mx_u - 32768);
      int16_t curY = (int16_t)((int32_t)my_u - 32768);
      int16_t curZ = (int16_t)((int32_t)mz_u - 32768);
      if (lastRx == 0 && lastRy == 0 && lastRz == 0) {
        lastRx = curX; lastRy = curY; lastRz = curZ;
      } else {
        lastRx = (int16_t)(lastRx * 0.35f + curX * 0.65f);
        lastRy = (int16_t)(lastRy * 0.35f + curY * 0.65f);
        lastRz = (int16_t)(lastRz * 0.35f + curZ * 0.65f);
      }
      rx = lastRx; ry = lastRy; rz = lastRz;
      lastReadMs = now;
      return true;
    }
    return false;
  }

private:
  void writeReg(TwoWire &wire, uint8_t dev, uint8_t reg, uint8_t val) {
    wire.beginTransmission(dev);
    wire.write(reg);
    wire.write(val);
    wire.endTransmission();
  }
};

// ==============================================================================
// 2. Madgwick AHRS 9-DoF Sensor Fusion
// Ported from fusion.h with stationary gyro bias tracking
// ==============================================================================

class RoverFusion {
public:
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float beta = 0.08f;
  float gyroBias[3] = {0, 0, 0};
  float magMin[3] = {1e9f, 1e9f, 1e9f}, magMax[3] = {-1e9f, -1e9f, -1e9f};
  bool gyroCal = false, magCal = false, levelCal = false, stationary = false;
  float magCoverage = 0.0f;
  float gyroRaw[3] = {0, 0, 0};
  float gz = 0.0f; // Bias-compensated yaw rate (rad/s)

  void initOrientation(const float a[3], const float *mraw = nullptr) {
    float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (an < 0.1f) return;
    float ax = a[0] / an, ay = a[1] / an, az = a[2] / an;
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    float yaw = 0.0f;
    if (mraw) {
      float sinR = sinf(roll), cosR = cosf(roll);
      float sinP = sinf(pitch), cosP = cosf(pitch);
      float Xh = mraw[0] * cosP + mraw[1] * sinR * sinP + mraw[2] * cosR * sinP;
      float Yh = mraw[1] * cosR - mraw[2] * sinR;
      yaw = atan2f(-Yh, Xh);
    }
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
    q0 = cr * cp * cy + sr * sp * sy;
    q1 = sr * cp * cy - cr * sp * sy;
    q2 = cr * sp * cy + sr * cp * sy;
    q3 = cr * cp * sy - sr * sp * cy;
    normalise();
  }

  void update(const float g[3], const float a[3], const float *mraw, float dt) {
    for (int i = 0; i < 3; i++) gyroRaw[i] = g[i];
    detectStationary(g, a, dt);

    float gx = g[0] - gyroBias[0];
    float gy = g[1] - gyroBias[1];
    float gz_val = g[2] - gyroBias[2];

    if (stationary && fabsf(gx) < 0.01f && fabsf(gy) < 0.01f && fabsf(gz_val) < 0.01f) {
      gx = gy = gz_val = 0;
    }
    this->gz = gz_val;

    float m[3] = {0, 0, 0};
    bool useMag = false;
    if (mraw) {
      trackMag(mraw);
      if (magCal) {
        float avg = ((magMax[0] - magMin[0]) + (magMax[1] - magMin[1]) + (magMax[2] - magMin[2])) / 6.0f;
        for (int i = 0; i < 3; i++) {
          float c = (magMax[i] + magMin[i]) * 0.5f;
          float r = (magMax[i] - magMin[i]) * 0.5f;
          m[i] = (mraw[i] - c) * (r > 1e-3f ? avg / r : 1.0f);
        }
        useMag = true;
      }
    }

    float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    bool accOk = an > 0.4f && an < 2.5f;
    float b = accOk ? beta : 0.0f;
    if (!accOk) {
      integrate(gx, gy, gz_val, dt);
      return;
    }
    madgwick(gx, gy, gz_val, a[0]/an, a[1]/an, a[2]/an, m, useMag, b, dt);
  }

  void euler(float &roll, float &pitch, float &yaw) const {
    roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    float s = 2*(q0*q2 - q3*q1);
    if (s > 1.0f) s = 1.0f; else if (s < -1.0f) s = -1.0f;
    pitch = asinf(s) * RAD_TO_DEG;
    yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * RAD_TO_DEG;
  }

  void zeroYaw() {
    float r, p, y;
    euler(r, p, y);
    float h = -y * DEG_TO_RAD * 0.5f;
    float c = cosf(h), s = sinf(h);
    float n0 = c * q0 - s * q3;
    float n1 = c * q1 - s * q2;
    float n2 = c * q2 + s * q1;
    float n3 = c * q3 + s * q0;
    q0 = n0; q1 = n1; q2 = n2; q3 = n3;
    normalise();
  }

private:
  float stillTimer = 0;
  float gN = 0;

  void detectStationary(const float g[3], const float a[3], float dt) {
    float gm = fabsf(g[0] - gyroBias[0]) + fabsf(g[1] - gyroBias[1]) + fabsf(g[2] - gyroBias[2]);
    float am = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    bool still = gm < 0.06f && fabsf(am - 1.0f) < 0.04f;
    if (still) {
      stillTimer += dt;
      if (stillTimer > 0.5f) {
        stationary = true;
        float k = 0.01f;
        for (int i = 0; i < 3; i++) gyroBias[i] += k * (g[i] - gyroBias[i]);
        gN += dt;
        if (gN > 3.0f) gyroCal = true;
      }
    } else {
      stillTimer = 0;
      stationary = false;
      gN = 0;
    }
  }

  void trackMag(const float m[3]) {
    float mag2 = m[0]*m[0] + m[1]*m[1] + m[2]*m[2];
    if (mag2 < 1.0f) return;
    for (int i = 0; i < 3; i++) {
      if (m[i] < magMin[i]) magMin[i] = m[i];
      if (m[i] > magMax[i]) magMax[i] = m[i];
    }
    float maxSpan = 0;
    for (int i = 0; i < 3; i++) {
      float span = magMax[i] - magMin[i];
      if (span > maxSpan) maxSpan = span;
    }
    float xy = fminf(magMax[0] - magMin[0], magMax[1] - magMin[1]);
    magCoverage = maxSpan > 1.0f ? (xy / maxSpan) : 0.0f;
    if (magCoverage > 1.0f) magCoverage = 1.0f;
    magCal = (magMax[0] - magMin[0] > 40) && (magMax[1] - magMin[1] > 40) && magCoverage > 0.5f;
  }

  void integrate(float gx, float gy, float gz, float dt) {
    float qd0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qd1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qd2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qd3 = 0.5f * ( q0*gz + q1*gy - q2*gx);
    q0 += qd0*dt; q1 += qd1*dt; q2 += qd2*dt; q3 += qd3*dt;
    normalise();
  }

  void normalise() {
    float n = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= n; q1 *= n; q2 *= n; q3 *= n;
  }

  void madgwick(float gx, float gy, float gz, float ax, float ay, float az, const float m[3], bool useMag, float b, float dt) {
    float qd0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qd1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qd2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qd3 = 0.5f * ( q0*gz + q1*gy - q2*gx);
    float s0, s1, s2, s3;

    float mn = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (useMag && mn > 1e-6f) {
      float mx = m[0]/mn, my = m[1]/mn, mz = m[2]/mn;
      float _2q0mx = 2*q0*mx, _2q0my = 2*q0*my, _2q0mz = 2*q0*mz, _2q1mx = 2*q1*mx;
      float _2q0 = 2*q0, _2q1 = 2*q1, _2q2 = 2*q2, _2q3 = 2*q3;
      float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3, q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
      float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;
      float hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
      float hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
      float _2bx = sqrtf(hx*hx + hy*hy);
      float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;
      float _4bx = 2*_2bx, _4bz = 2*_2bz;
      s0 = -_2q2*(2*q1q3 - 2*q0q2 - ax) + _2q1*(2*q0q1 + 2*q2q3 - ay) - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3*(2*q1q3 - 2*q0q2 - ax) + _2q0*(2*q0q1 + 2*q2q3 - ay) - 4*q1*(1 - 2*q1q1 - 2*q2q2 - az) + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q3 - _4bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0*(2*q1q3 - 2*q0q2 - ax) + _2q3*(2*q0q1 + 2*q2q3 - ay) - 4*q2*(1 - 2*q1q1 - 2*q2q2 - az) + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q0 - _4bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1*(2*q1q3 - 2*q0q2 - ax) + _2q2*(2*q0q1 + 2*q2q3 - ay) + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    } else {
      float _2q0 = 2*q0, _2q1 = 2*q1, _2q2 = 2*q2, _2q3 = 2*q3, _4q0 = 4*q0, _4q1 = 4*q1, _4q2 = 4*q2, _8q1 = 8*q1, _8q2 = 8*q2;
      float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
      s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
      s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
      s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
      s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay;
    }
    float n = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (n > 1e-9f) {
      n = 1.0f / n;
      s0 *= n; s1 *= n; s2 *= n; s3 *= n;
      qd0 -= b*s0; qd1 -= b*s1; qd2 -= b*s2; qd3 -= b*s3;
    }
    q0 += qd0*dt; q1 += qd1*dt; q2 += qd2*dt; q3 += qd3*dt;
    normalise();
  }
};

// ==============================================================================
// 3. RoverAHRS System: MPU6050 + Compass + Impact + Gated Dead Reckoning
// ==============================================================================

class RoverAHRS {
public:
  uint8_t mpuAddr = 0x68;
  bool imuReady = false;
  bool magReady = false;
  RoverCompass compass;
  RoverFusion fusion;

  float lastAcc[3] = {0, 0, 1};
  float lastGyro[3] = {0, 0, 0};
  float lastMag[3] = {0, 0, 0};
  float roll = 0, pitch = 0, heading = 0;

  // Impact detection state
  float emaAc[3] = {0.0f, 0.0f, 1.0f};
  float prevAc[3] = {0.0f, 0.0f, 1.0f};
  float prevGy[3] = {0.0f, 0.0f, 0.0f};
  bool impactInited = false;
  float dynJerk = 0.0f;
  uint8_t impactDir = IMPACT_NONE;
  uint32_t lastImpactTime = 0;
  uint32_t impactUntil = 0;
  uint32_t motionStartTime = 0;

  // IMU-gated dead reckoning
  float odomX = 0.0f; // cm
  float odomY = 0.0f; // cm

  void zeroYaw() {
    fusion.zeroYaw();
    heading = 0.0f;
    odomX = 0.0f;
    odomY = 0.0f;
  }

  bool begin(TwoWire &wire) {
    uint8_t addrs[2] = {0x68, 0x69};
    for (int i = 0; i < 2; i++) {
      uint8_t addr = addrs[i];
      wire.beginTransmission(addr);
      if (wire.endTransmission() != 0) continue;

      wire.beginTransmission(addr);
      wire.write(0x75); // WHO_AM_I
      if (wire.endTransmission(false) == 0 && wire.requestFrom((int)addr, 1) == 1) {
        uint8_t who = wire.read();
        if (who == 0x00 || who == 0xFF) continue;
        mpuAddr = addr;

        // Wake up MPU6050 (0x6B = 0x01)
        wire.beginTransmission(addr); wire.write(0x6B); wire.write(0x01); wire.endTransmission();
        delay(20);
        // Sample rate divider = 0
        wire.beginTransmission(addr); wire.write(0x19); wire.write(0x00); wire.endTransmission();
        // DLPF = 42Hz
        wire.beginTransmission(addr); wire.write(0x1A); wire.write(0x03); wire.endTransmission();
        // Gyro +/- 500 deg/s
        wire.beginTransmission(addr); wire.write(0x1B); wire.write(0x08); wire.endTransmission();
        // Accel +/- 4g
        wire.beginTransmission(addr); wire.write(0x1C); wire.write(0x08); wire.endTransmission();

        // Enable I2C Bypass mode to expose compass on shared bus
        wire.beginTransmission(addr); wire.write(0x6A); wire.write(0x00); wire.endTransmission();
        wire.beginTransmission(addr); wire.write(0x37); wire.write(0x02); wire.endTransmission();
        delay(10);

        imuReady = true;

        // Gyro initial bias sampling
        float sumG[3] = {0, 0, 0};
        float testG[3], testA[3];
        int samples = 0;
        for (int k = 0; k < 40; k++) {
          if (readRawMPU(wire, testG, testA)) {
            sumG[0] += testG[0];
            sumG[1] += testG[1];
            sumG[2] += testG[2];
            samples++;
          }
          delay(3);
        }
        if (samples > 0) {
          fusion.gyroBias[0] = sumG[0] / samples;
          fusion.gyroBias[1] = sumG[1] / samples;
          fusion.gyroBias[2] = sumG[2] / samples;
        }
        break;
      }
    }

    magReady = compass.begin(wire);
    return imuReady;
  }

  bool readRawMPU(TwoWire &wire, float g[3], float a[3]) {
    if (!imuReady) return false;
    wire.beginTransmission(mpuAddr);
    wire.write(0x3B);
    if (wire.endTransmission(false) != 0) return false;
    if (wire.requestFrom((int)mpuAddr, 14) != 14) return false;

    int16_t ax = (int16_t)((wire.read() << 8) | wire.read());
    int16_t ay = (int16_t)((wire.read() << 8) | wire.read());
    int16_t az = (int16_t)((wire.read() << 8) | wire.read());
    wire.read(); wire.read(); // Skip temperature
    int16_t gx = (int16_t)((wire.read() << 8) | wire.read());
    int16_t gy = (int16_t)((wire.read() << 8) | wire.read());
    int16_t gz = (int16_t)((wire.read() << 8) | wire.read());

    if ((ax == 0 && ay == 0 && az == 0) || (ax == -1 && ay == -1 && az == -1)) return false;

    // Accel +/- 4g (8192 LSB/g) -> g
    a[0] = (float)ax / 8192.0f;
    a[1] = (float)ay / 8192.0f;
    a[2] = (float)az / 8192.0f;

    // Gyro +/- 500 deg/s (65.5 LSB/(deg/s)) -> rad/s
    g[0] = ((float)gx / 65.5f) * DEG_TO_RAD;
    g[1] = ((float)gy / 65.5f) * DEG_TO_RAD;
    g[2] = ((float)gz / 65.5f) * DEG_TO_RAD;
    return true;
  }

  // 100 Hz update step (called with dt = 0.01s)
  void step(TwoWire &wire, float dt, uint32_t now, bool motion_active, float fwd_speed, float strafe_speed) {
    if (!imuReady) return;

    float gy[3] = {0, 0, 0};
    float ac[3] = {0, 0, 1};
    if (readRawMPU(wire, gy, ac)) {
      lastAcc[0] = ac[0]; lastAcc[1] = ac[1]; lastAcc[2] = ac[2];
      lastGyro[0] = gy[0]; lastGyro[1] = gy[1]; lastGyro[2] = gy[2];

      if (!impactInited) {
        for (int i = 0; i < 3; i++) {
          prevAc[i] = ac[i];
          prevGy[i] = gy[i];
          emaAc[i] = ac[i];
        }
        impactInited = true;
      }

      // Fast EMA gravity tracker
      for (int i = 0; i < 3; i++) {
        emaAc[i] = emaAc[i] * 0.88f + ac[i] * 0.12f;
      }

      // High-pass linear jerk (g)
      float jerkX = ac[0] - emaAc[0];
      float jerkY = ac[1] - emaAc[1];
      float hJerk = sqrtf(jerkX * jerkX + jerkY * jerkY);
      dynJerk = hJerk;

      float djX = fabsf(ac[0] - prevAc[0]);
      float djY = fabsf(ac[1] - prevAc[1]);
      float dGyroZ = fabsf(gy[2] - prevGy[2]) * RAD_TO_DEG;

      prevAc[0] = ac[0]; prevAc[1] = ac[1]; prevAc[2] = ac[2];
      prevGy[0] = gy[0]; prevGy[1] = gy[1]; prevGy[2] = gy[2];

      // Motion start suppression: suppress impact alert within ~600ms of motion start
      bool suppressed = false;
      if (motion_active) {
        if (motionStartTime == 0) motionStartTime = now;
        if (now - motionStartTime < 600) suppressed = true;
      } else {
        motionStartTime = 0;
      }

      // Physical impact threshold (distinguishes abrupt collision from motor ramp S-curves)
      bool isImpact = ((hJerk > 0.85f && (djX > 0.65f || djY > 0.65f)) || (dGyroZ > 160.0f && hJerk > 0.55f));
      if (isImpact && !suppressed && (now - lastImpactTime >= 1500)) {
        lastImpactTime = now;
        impactUntil = now + 420; // 420ms defensive recovery period

        if (fabsf(jerkY) >= fabsf(jerkX)) {
          impactDir = (jerkY < 0.0f) ? IMPACT_FRONT : IMPACT_REAR;
        } else {
          impactDir = (jerkX > 0.0f) ? IMPACT_LEFT : IMPACT_RIGHT;
        }
      }

      if (now >= impactUntil && impactDir != IMPACT_NONE) {
        impactDir = IMPACT_NONE;
      }

      // Compass read
      float mg[3] = {0, 0, 0};
      bool haveMag = false;
      if (magReady) {
        int16_t rx, ry, rz;
        if (compass.readRaw(wire, rx, ry, rz)) {
          mg[0] = (float)rx; mg[1] = (float)ry; mg[2] = (float)rz;
          lastMag[0] = mg[0]; lastMag[1] = mg[1]; lastMag[2] = mg[2];
          haveMag = true;
        }
      }

      fusion.update(gy, ac, haveMag ? mg : nullptr, dt);
      fusion.euler(roll, pitch, heading);

      // --- IMU-Gated Dead Reckoning ---
      // Gated on physical motion / accelerometer jerk and gyro rate:
      // If rover is stationary or stuck in static friction, do not integrate phantom motion.
      bool isPhysicallyMoving = (!fusion.stationary) && (dynJerk > 0.045f || fabsf(fusion.gz) > 0.06f);

      float vx = 0.0f, vy = 0.0f;
      float effort = fmaxf(fabsf(fwd_speed), fabsf(strafe_speed));
      if (isPhysicallyMoving && effort >= 12.0f) {
        float scale = (effort - 12.0f) / effort;
        vx = fwd_speed * 8.5f * scale;    // mm/s forward along heading
        vy = strafe_speed * 8.5f * scale; // mm/s lateral strafe
      }

      float th = heading * DEG_TO_RAD;
      odomX += (vx * cosf(th) - vy * sinf(th)) * dt / 10.0f; // cm
      odomY += (vx * sinf(th) + vy * cosf(th)) * dt / 10.0f; // cm
    }
  }
};

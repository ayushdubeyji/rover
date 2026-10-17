#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

// 9-DoF Madgwick AHRS + online calibration (gyro bias, hard/soft-iron mag, level trim)
// Frame: X forward, Y left, Z up. Units: gyro rad/s, accel g, mag arbitrary (normalised internally).
class RoverFusion {
public:
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  float beta = 0.08f;
  float gyroBias[3] = {0, 0, 0};
  float magMin[3] = {1e9f, 1e9f, 1e9f}, magMax[3] = {-1e9f, -1e9f, -1e9f};
  float rollTrim = 0, pitchTrim = 0, yawZero = 0;
  bool gyroCal = false, magCal = false, levelCal = false, stationary = false;
  float magCoverage = 0;
  float gyroRaw[3] = {0, 0, 0};
  float gz = 0; // bias-compensated yaw rate (rad/s)

  void load(Preferences &p) {
    p.begin("fusion", true);
    gyroCal = p.getBool("gc", false);
    for (int i = 0; i < 3; i++) {
      char k[4] = {'b', (char)('x' + i), 0, 0};
      gyroBias[i] = p.getFloat(k, 0);
      k[0] = 'n'; magMin[i] = p.getFloat(k, 1e9f);
      k[0] = 'm'; magMax[i] = p.getFloat(k, -1e9f);
    }
    rollTrim = p.getFloat("rt", 0);
    pitchTrim = p.getFloat("pt", 0);
    levelCal = p.getBool("lc", false);
    p.end();
    updateMagCoverage();
  }

  void save(Preferences &p) {
    p.begin("fusion", false);
    p.putBool("gc", gyroCal);
    for (int i = 0; i < 3; i++) {
      char k[4] = {'b', (char)('x' + i), 0, 0};
      p.putFloat(k, gyroBias[i]);
      k[0] = 'n'; p.putFloat(k, magMin[i]);
      k[0] = 'm'; p.putFloat(k, magMax[i]);
    }
    p.putFloat("rt", rollTrim);
    p.putFloat("pt", pitchTrim);
    p.putBool("lc", levelCal);
    p.end();
  }

  void resetMagCal() {
    for (int i = 0; i < 3; i++) { magMin[i] = 1e9f; magMax[i] = -1e9f; }
    magCal = false; magCoverage = 0;
  }

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

  // Call at a fixed rate with dt seconds. mag may be NULL.
  void update(const float g[3], const float a[3], const float *mraw, float dt) {
    for (int i = 0; i < 3; i++) gyroRaw[i] = g[i];
    detectStationary(g, a, dt);
    float gx = g[0] - gyroBias[0], gy = g[1] - gyroBias[1], gz = g[2] - gyroBias[2];
    if (stationary && fabsf(gx) < 0.01f && fabsf(gy) < 0.01f && fabsf(gz) < 0.01f) gx = gy = gz = 0;
    this->gz = gz;

    float m[3] = {0, 0, 0};
    bool useMag = false;
    if (mraw) {
      trackMag(mraw);
      if (magCal) {
        for (int i = 0; i < 3; i++) {
          float c = (magMax[i] + magMin[i]) * 0.5f;
          float r = (magMax[i] - magMin[i]) * 0.5f;
          float avg = ((magMax[0] - magMin[0]) + (magMax[1] - magMin[1]) + (magMax[2] - magMin[2])) / 6.0f;
          m[i] = (mraw[i] - c) * (r > 1e-3f ? avg / r : 1.0f);
        }
        useMag = true;
      } else {
        float magMag = sqrtf(mraw[0]*mraw[0] + mraw[1]*mraw[1] + mraw[2]*mraw[2]);
        if (magMag > 10.0f && magMag < 50000.0f) {
          for (int i = 0; i < 3; i++) {
            float c = (magMax[i] < 1e8f && magMin[i] > -1e8f) ? (magMax[i] + magMin[i]) * 0.5f : 0.0f;
            m[i] = mraw[i] - c;
          }
          useMag = true;
        }
      }
    }

    float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    bool accOk = an > 0.4f && an < 2.5f;
    // Adaptive gain: trust accel less under linear acceleration
    float b = accOk ? beta : 0.0f;
    if (!accOk) { integrate(gx, gy, gz, dt); return; }
    madgwick(gx, gy, gz, a[0]/an, a[1]/an, a[2]/an, m, useMag, b, dt);
  }

  void euler(float &roll, float &pitch, float &yaw) const {
    roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RAD_TO_DEG - rollTrim;
    float s = 2*(q0*q2 - q3*q1); s = constrain(s, -1.0f, 1.0f);
    pitch = asinf(s) * RAD_TO_DEG - pitchTrim;
    yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * RAD_TO_DEG;
  }

  void zeroYaw() {
    float r, p, y; euler(r, p, y);
    yawZero += y;
    while (yawZero > 180.0f) yawZero -= 360.0f;
    while (yawZero < -180.0f) yawZero += 360.0f;
    float h = -y * DEG_TO_RAD * 0.5f;
    float c = cosf(h), s = sinf(h);
    float n0 = c*q0 - s*q3, n1 = c*q1 - s*q2, n2 = c*q2 + s*q1, n3 = c*q3 + s*q0;
    q0 = n0; q1 = n1; q2 = n2; q3 = n3;
  }

  // Level trim: call while rover is stationary on a flat surface
  void calibrateLevel(Preferences &p) {
    float r, pi, y; rollTrim = pitchTrim = 0; euler(r, pi, y);
    rollTrim = r; pitchTrim = pi; levelCal = true; save(p);
  }

  void updateMagCoverage() {
    float minSpan = 1e9f;
    for (int i = 0; i < 3; i++) minSpan = fminf(minSpan, magMax[i] - magMin[i]);
    float maxSpan = 0;
    for (int i = 0; i < 3; i++) maxSpan = fmaxf(maxSpan, magMax[i] - magMin[i]);
    // Heading only needs X/Y rotation; Z spans widen only when tilted
    float xy = fminf(magMax[0] - magMin[0], magMax[1] - magMin[1]);
    magCoverage = maxSpan > 1 ? constrain(xy / maxSpan, 0.0f, 1.0f) : 0;
    magCal = (magMax[0] - magMin[0] > 40) && (magMax[1] - magMin[1] > 40) && magCoverage > 0.5f;
    (void)minSpan;
  }

private:
  float stillTimer = 0;
  float gAcc[3] = {0, 0, 0};
  float gN = 0;
  unsigned long lastSave = 0;

  void detectStationary(const float g[3], const float a[3], float dt) {
    float gm = fabsf(g[0] - gyroBias[0]) + fabsf(g[1] - gyroBias[1]) + fabsf(g[2] - gyroBias[2]);
    float am = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    bool still = gm < 0.06f && fabsf(am - 1.0f) < 0.04f;
    if (still) {
      stillTimer += dt;
      if (stillTimer > 0.5f) {
        stationary = true;
        // slow bias tracking, converges ~ 3 s
        float k = 0.01f;
        for (int i = 0; i < 3; i++) gyroBias[i] += k * (g[i] - gyroBias[i]);
        gN += dt;
        if (gN > 3.0f) gyroCal = true;
      }
    } else { stillTimer = 0; stationary = false; gN = 0; }
  }

  void trackMag(const float m[3]) {
    float mag2 = m[0]*m[0] + m[1]*m[1] + m[2]*m[2];
    if (mag2 < 1.0f) return;
    for (int i = 0; i < 3; i++) {
      if (m[i] < magMin[i]) magMin[i] = m[i];
      if (m[i] > magMax[i]) magMax[i] = m[i];
    }
    updateMagCoverage();
    if (magCal && millis() - lastSave > 60000) { lastSave = millis(); }
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
      float _2q0 = 2*q0, _2q1 = 2*q1, _2q2 = 2*q2, _2q3 = 2*q3, _2q0q2 = 2*q0*q2, _2q2q3 = 2*q2*q3;
      float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3, q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
      float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;
      float hx = mx*q0q0 - _2q0my*q3 + _2q0mz*q2 + mx*q1q1 + _2q1*my*q2 + _2q1*mz*q3 - mx*q2q2 - mx*q3q3;
      float hy = _2q0mx*q3 + my*q0q0 - _2q0mz*q1 + _2q1mx*q2 - my*q1q1 + my*q2q2 + _2q2*mz*q3 - my*q3q3;
      float _2bx = sqrtf(hx*hx + hy*hy);
      float _2bz = -_2q0mx*q2 + _2q0my*q1 + mz*q0q0 + _2q1mx*q3 - mz*q1q1 + _2q2*my*q3 - mz*q2q2 + mz*q3q3;
      float _4bx = 2*_2bx, _4bz = 2*_2bz;
      s0 = -_2q2*(2*q1q3 - _2q0q2 - ax) + _2q1*(2*q0q1 + _2q2q3 - ay) - _2bz*q2*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q3 + _2bz*q1)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q2*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
      s1 = _2q3*(2*q1q3 - _2q0q2 - ax) + _2q0*(2*q0q1 + _2q2q3 - ay) - 4*q1*(1 - 2*q1q1 - 2*q2q2 - az) + _2bz*q3*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q2 + _2bz*q0)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q3 - _4bz*q1)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
      s2 = -_2q0*(2*q1q3 - _2q0q2 - ax) + _2q3*(2*q0q1 + _2q2q3 - ay) - 4*q2*(1 - 2*q1q1 - 2*q2q2 - az) + (-_4bx*q2 - _2bz*q0)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (_2bx*q1 + _2bz*q3)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + (_2bx*q0 - _4bz*q2)*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
      s3 = _2q1*(2*q1q3 - _2q0q2 - ax) + _2q2*(2*q0q1 + _2q2q3 - ay) + (-_4bx*q3 + _2bz*q1)*(_2bx*(0.5f - q2q2 - q3q3) + _2bz*(q1q3 - q0q2) - mx) + (-_2bx*q0 + _2bz*q2)*(_2bx*(q1q2 - q0q3) + _2bz*(q0q1 + q2q3) - my) + _2bx*q1*(_2bx*(q0q2 + q1q3) + _2bz*(0.5f - q1q1 - q2q2) - mz);
    } else {
      float _2q0 = 2*q0, _2q1 = 2*q1, _2q2 = 2*q2, _2q3 = 2*q3, _4q0 = 4*q0, _4q1 = 4*q1, _4q2 = 4*q2, _8q1 = 8*q1, _8q2 = 8*q2;
      float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
      s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
      s1 = _4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
      s2 = 4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
      s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay;
    }
    float n = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (n > 1e-9f) { n = 1.0f / n; s0 *= n; s1 *= n; s2 *= n; s3 *= n; qd0 -= b*s0; qd1 -= b*s1; qd2 -= b*s2; qd3 -= b*s3; }
    q0 += qd0*dt; q1 += qd1*dt; q2 += qd2*dt; q3 += qd3*dt;
    normalise();
  }
};

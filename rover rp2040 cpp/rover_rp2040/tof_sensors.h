#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <pico/mutex.h>
#include "pins.h"

class DualToFDriver {
public:
  Adafruit_VL53L0X tof_front;
  Adafruit_VL53L0X tof_rear;

  bool front_ready = false;
  bool rear_ready = false;

  uint16_t front_mm = 0;
  uint16_t rear_mm = 0;

  uint32_t last_read_ms[2] = {0, 0};
  uint32_t last_retry_ms = 0;

  // 5-sample circular buffers for median filtering
  uint16_t buf_front[5] = {0, 0, 0, 0, 0};
  uint8_t idx_front = 0;
  uint8_t count_front = 0;
  uint8_t spike_count_front = 0;
  uint8_t misses_front = 0;

  uint16_t buf_rear[5] = {0, 0, 0, 0, 0};
  uint8_t idx_rear = 0;
  uint8_t count_rear = 0;
  uint8_t spike_count_rear = 0;
  uint8_t misses_rear = 0;

  void begin(TwoWire &wire, mutex_t *i2c_lock = nullptr) {
    pinMode(PIN_TOF_FRONT_XSHUT, OUTPUT);
    pinMode(PIN_TOF_REAR_XSHUT, OUTPUT);

    initSensors(wire, i2c_lock);
  }

  void initSensors(TwoWire &wire, mutex_t *i2c_lock = nullptr) {
    front_ready = false;
    rear_ready = false;

    // 1. Reset both sensors
    digitalWrite(PIN_TOF_FRONT_XSHUT, LOW);
    digitalWrite(PIN_TOF_REAR_XSHUT, LOW);
    delay(25);

    // 2. Power up Front ToF and set address to 0x32
    digitalWrite(PIN_TOF_FRONT_XSHUT, HIGH);
    delay(25);
    if (i2c_lock) mutex_enter_blocking(i2c_lock);
    front_ready = tof_front.begin(TOF_ADDR_FRONT, false, &wire);
    if (front_ready) {
      tof_front.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
      tof_front.startRangeContinuous(30);
      last_read_ms[0] = millis();
    }
    if (i2c_lock) mutex_exit(i2c_lock);

    if (!front_ready) {
      digitalWrite(PIN_TOF_FRONT_XSHUT, LOW);
    }

    // 3. Power up Rear ToF and set address to 0x33
    digitalWrite(PIN_TOF_REAR_XSHUT, HIGH);
    delay(25);
    if (i2c_lock) mutex_enter_blocking(i2c_lock);
    rear_ready = tof_rear.begin(TOF_ADDR_REAR, false, &wire);
    if (rear_ready) {
      tof_rear.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
      tof_rear.startRangeContinuous(30);
      last_read_ms[1] = millis();
    }
    if (i2c_lock) mutex_exit(i2c_lock);

    if (!rear_ready) {
      digitalWrite(PIN_TOF_REAR_XSHUT, LOW);
    }

    last_retry_ms = millis();
  }

  // Non-blocking poll routine (called ~35 Hz from Core 1)
  void update(TwoWire &wire, uint32_t now, mutex_t *i2c_lock = nullptr) {
    // 1. Stall Detection (> 2500 ms without completed range)
    if (front_ready && (now - last_read_ms[0] > 2500)) {
      front_ready = false;
      front_mm = 0;
      count_front = 0;
      idx_front = 0;
    }
    if (rear_ready && (now - last_read_ms[1] > 2500)) {
      rear_ready = false;
      rear_mm = 0;
      count_rear = 0;
      idx_rear = 0;
    }

    // 2. Self-Healing background re-init every 5s if any sensor is offline
    if ((!front_ready || !rear_ready) && (now - last_retry_ms > 5000)) {
      initSensors(wire, i2c_lock);
      return;
    }

    // 3. Front ToF Read & Filter
    if (front_ready) {
      bool complete = false;
      uint16_t raw = 0;
      uint8_t status = 0;

      if (i2c_lock) mutex_enter_blocking(i2c_lock);
      complete = tof_front.isRangeComplete();
      if (complete) {
        raw = tof_front.readRangeResult();
        status = tof_front.readRangeStatus();
      }
      if (i2c_lock) mutex_exit(i2c_lock);

      if (complete) {
        // Status 4 is Phase Fail / Out of Bounds
        if (status != 4 && raw >= 30 && raw <= 1800) {
          misses_front = 0;
          last_read_ms[0] = now;

          // Spike filter (> 250 mm difference from previous valid reading)
          bool is_spike = false;
          if (front_mm > 0 && abs((int)raw - (int)front_mm) > 250) {
            if (++spike_count_front < 2) {
              is_spike = true;
            } else {
              // Confirmed step change: re-seed buffer with new baseline
              for (int k = 0; k < 5; k++) buf_front[k] = raw;
              count_front = 5;
              idx_front = 0;
              spike_count_front = 0;
              front_mm = raw;
            }
          } else {
            spike_count_front = 0;
          }

          if (!is_spike) {
            buf_front[idx_front] = raw;
            idx_front = (idx_front + 1) % 5;
            if (count_front < 5) count_front++;
            front_mm = computeMedian(buf_front, count_front);
          }
        } else {
          // Range out of bounds or phase fail: clear after 4 consecutive misses
          if (++misses_front >= 4) {
            front_mm = 0;
            count_front = 0;
            idx_front = 0;
          }
        }
      }
    }

    // 4. Rear ToF Read & Filter
    if (rear_ready) {
      bool complete = false;
      uint16_t raw = 0;
      uint8_t status = 0;

      if (i2c_lock) mutex_enter_blocking(i2c_lock);
      complete = tof_rear.isRangeComplete();
      if (complete) {
        raw = tof_rear.readRangeResult();
        status = tof_rear.readRangeStatus();
      }
      if (i2c_lock) mutex_exit(i2c_lock);

      if (complete) {
        if (status != 4 && raw >= 30 && raw <= 1800) {
          misses_rear = 0;
          last_read_ms[1] = now;

          bool is_spike = false;
          if (rear_mm > 0 && abs((int)raw - (int)rear_mm) > 250) {
            if (++spike_count_rear < 2) {
              is_spike = true;
            } else {
              for (int k = 0; k < 5; k++) buf_rear[k] = raw;
              count_rear = 5;
              idx_rear = 0;
              spike_count_rear = 0;
              rear_mm = raw;
            }
          } else {
            spike_count_rear = 0;
          }

          if (!is_spike) {
            buf_rear[idx_rear] = raw;
            idx_rear = (idx_rear + 1) % 5;
            if (count_rear < 5) count_rear++;
            rear_mm = computeMedian(buf_rear, count_rear);
          }
        } else {
          if (++misses_rear >= 4) {
            rear_mm = 0;
            count_rear = 0;
            idx_rear = 0;
          }
        }
      }
    }
  }

private:
  uint16_t computeMedian(const uint16_t *buf, uint8_t n) {
    if (n == 0) return 0;
    if (n == 1) return buf[0];
    uint16_t s[5];
    for (uint8_t i = 0; i < n; i++) s[i] = buf[i];
    for (uint8_t i = 0; i < n - 1; i++) {
      for (uint8_t j = i + 1; j < n; j++) {
        if (s[i] > s[j]) {
          uint16_t tmp = s[i];
          s[i] = s[j];
          s[j] = tmp;
        }
      }
    }
    return s[n / 2];
  }
};

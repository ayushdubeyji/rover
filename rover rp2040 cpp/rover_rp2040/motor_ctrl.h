#pragma once
#include <Arduino.h>
#include "pins.h"
#include <math.h>

struct MotorPair {
  uint8_t fwd_pin;
  uint8_t bck_pin;
};

class RoverMotors {
public:
  // Motor definitions: FL (0), FR (1), RL (2), RR (3)
  MotorPair pairs[4] = {
    { PIN_MOTOR_FL_FWD, PIN_MOTOR_FL_BCK }, // FL: GP9 fwd, GP8 bck
    { PIN_MOTOR_FR_FWD, PIN_MOTOR_FR_BCK }, // FR: GP12 fwd, GP13 bck
    { PIN_MOTOR_RL_FWD, PIN_MOTOR_RL_BCK }, // RL: GP11 fwd, GP10 bck
    { PIN_MOTOR_RR_FWD, PIN_MOTOR_RR_BCK }  // RR: GP14 fwd, GP15 bck
  };

  int8_t invert[4] = { 1, 1, 1, 1 };
  int8_t strafe_sign = 1;
  int8_t rot_sign = 1;
  uint16_t min_duty = 38000;
  float accel = 25.0f;
  uint16_t spin_ms = 1400;

  float target[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  float current[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
  bool was_stopped[4] = { true, true, true, true };
  uint8_t stiction_kick[4] = { 0, 0, 0, 0 };

  uint32_t last_cmd_ms = 0;
  uint32_t tick_count = 0;

  void setConfig(const int8_t inv[4], int8_t st, int8_t rt, uint32_t md, float ac, uint32_t sp) {
    for (int i = 0; i < 4; i++) invert[i] = (inv[i] == -1) ? -1 : 1;
    strafe_sign = (st == -1) ? -1 : 1;
    rot_sign = (rt == -1) ? -1 : 1;
    min_duty = (uint16_t)(md > 60000 ? 60000 : md);
    accel = (ac < 1.0f) ? 1.0f : (ac > 100.0f ? 100.0f : ac);
    spin_ms = (uint16_t)(sp < 200 ? 200 : (sp > 10000 ? 10000 : sp));
  }

  // Macro state machine
  struct MacroStep {
    float x, y, z, spd;
    uint32_t duration_ms;
  };
  static const uint8_t MAX_MACRO_STEPS = 6;
  MacroStep macro_queue[MAX_MACRO_STEPS];
  uint8_t macro_step_count = 0;
  uint8_t macro_step_idx = 0;
  uint32_t macro_step_end_ms = 0;
  bool macro_active = false;

  void begin() {
    analogWriteFreq(MOTOR_PWM_FREQ);
    analogWriteRange(MOTOR_PWM_MAX_DUTY);

    for (int i = 0; i < 4; i++) {
      pinMode(pairs[i].fwd_pin, OUTPUT);
      pinMode(pairs[i].bck_pin, OUTPUT);
      analogWrite(pairs[i].fwd_pin, 0);
      analogWrite(pairs[i].bck_pin, 0);
    }
    stopAll();
    last_cmd_ms = millis();
  }

  void stopAll() {
    macro_active = false;
    macro_step_count = 0;
    macro_step_idx = 0;
    for (int i = 0; i < 4; i++) {
      target[i] = 0.0f;
      current[i] = 0.0f;
      was_stopped[i] = true;
      stiction_kick[i] = 0;
      driveRaw(i, 0.0f);
    }
  }

  void mix(float x, float y, float z, float spd, float out[4]) {
    x *= (float)strafe_sign;
    z *= (float)rot_sign;

    float w[4];
    w[0] = y + x + z;  // FL
    w[1] = y - x - z;  // FR
    w[2] = y - x + z;  // RL
    w[3] = y + x - z;  // RR

    float m = 0.0f;
    for (int i = 0; i < 4; i++) {
      float a = fabsf(w[i]);
      if (a > m) m = a;
    }
    if (m > 100.0f) {
      for (int i = 0; i < 4; i++) {
        w[i] = (w[i] * 100.0f) / m;
      }
    }
    float k = spd / 100.0f;
    for (int i = 0; i < 4; i++) {
      out[i] = w[i] * k;
    }
  }

  void setDrive(float x, float y, float z, float spd) {
    if (macro_active) return; // Do not interrupt running macro unless stop requested
    last_cmd_ms = millis();
    float t[4];
    mix(x, y, z, spd, t);
    for (int i = 0; i < 4; i++) {
      target[i] = t[i];
    }
  }

  void setSingleWheel(uint8_t wheel_idx, float speed) {
    if (macro_active) return;
    last_cmd_ms = millis();
    for (int i = 0; i < 4; i++) {
      target[i] = 0.0f;
    }
    if (wheel_idx < 4) {
      target[wheel_idx] = speed;
    }
  }

  void startMacro(uint8_t mid, int8_t direction) {
    float d = (direction >= 0) ? 1.0f : -1.0f;
    uint32_t s_ms = spin_ms;
    macro_step_count = 0;
    macro_step_idx = 0;

    if (mid == 1) { // 360 spin
      macro_queue[0] = { 0, 0, 100.0f * d, 100, s_ms };
      macro_queue[1] = { 0, 0, 0, 0, 150 };
      macro_step_count = 2;
    } else if (mid == 2) { // 180 spin
      macro_queue[0] = { 0, 0, 100.0f * d, 100, s_ms / 2 };
      macro_queue[1] = { 0, 0, 0, 0, 150 };
      macro_step_count = 2;
    } else if (mid == 3) { // J-turn: reverse, 180 spin, forward, stop
      macro_queue[0] = { 0, -100.0f, 0, 100, 450 };
      macro_queue[1] = { 0, 0, 100.0f * d, 100, s_ms / 2 };
      macro_queue[2] = { 0, 100.0f, 0, 100, 450 };
      macro_queue[3] = { 0, 0, 0, 0, 150 };
      macro_step_count = 4;
    } else if (mid == 4) { // Strafing circle
      macro_queue[0] = { 100.0f * d, 0, -60.0f * d, 100, (uint32_t)(s_ms * 1.5f) };
      macro_queue[1] = { 0, 0, 0, 0, 150 };
      macro_step_count = 2;
    } else {
      return;
    }

    macro_step_end_ms = 0;
    macro_active = true;
    last_cmd_ms = millis();
  }

  void updateMacro(uint32_t now) {
    if (!macro_active) return;

    if (macro_step_end_ms == 0 || now >= macro_step_end_ms) {
      if (macro_step_end_ms != 0) {
        macro_step_idx++;
        if (macro_step_idx >= macro_step_count) {
          stopAll();
          return;
        }
      }
      MacroStep &step = macro_queue[macro_step_idx];
      float t[4];
      mix(step.x, step.y, step.z, step.spd, t);
      for (int i = 0; i < 4; i++) {
        target[i] = t[i];
      }
      macro_step_end_ms = now + step.duration_ms;
    }
  }

  // 100 Hz step (called every 10 ms from Core 0 loop)
  void step(uint32_t now) {
    tick_count++;
    updateMacro(now);

    // Failsafe 0.4s: if no active macro and command timed out, zero targets
    if (!macro_active && (now - last_cmd_ms > 400)) {
      for (int i = 0; i < 4; i++) {
        target[i] = 0.0f;
      }
    }

    // Slew rate ramp current speeds toward target
    for (int i = 0; i < 4; i++) {
      float d = target[i] - current[i];
      if (fabsf(d) <= accel) {
        current[i] = target[i];
      } else {
        current[i] += (d > 0.0f) ? accel : -accel;
      }
      driveRaw(i, current[i]);
    }
  }

  bool isMoving() const {
    for (int i = 0; i < 4; i++) {
      if (fabsf(current[i]) >= 2.0f || fabsf(target[i]) >= 2.0f) return true;
    }
    return false;
  }

private:
  void driveRaw(uint8_t i, float speed) {
    if (i >= 4) return;
    speed *= (float)invert[i];
    float a = fabsf(speed);

    if (a < 2.0f) {
      analogWrite(pairs[i].fwd_pin, 0);
      analogWrite(pairs[i].bck_pin, 0);
      was_stopped[i] = true;
      stiction_kick[i] = 0;
      return;
    }

    if (was_stopped[i]) {
      was_stopped[i] = false;
      stiction_kick[i] = 18; // 180 ms breakaway kick
    }

    uint32_t duty = min_duty + (uint32_t)((a - 2.0f) / 98.0f * (float)(MOTOR_PWM_MAX_DUTY - min_duty));

    if (stiction_kick[i] > 0) {
      stiction_kick[i]--;
      if (duty < 52000) duty = 52000; // Break static friction with +40% rover mass
    } else if (a < 20.0f) {
      // Dither at very low speeds
      int32_t dither = ((tick_count % 4) < 2) ? 1800 : -1800;
      int32_t d = (int32_t)duty + dither;
      if (d < (int32_t)min_duty + 2000) d = min_duty + 2000;
      if (d > MOTOR_PWM_MAX_DUTY) d = MOTOR_PWM_MAX_DUTY;
      duty = (uint32_t)d;
    }

    if (duty > MOTOR_PWM_MAX_DUTY) duty = MOTOR_PWM_MAX_DUTY;

    if (speed > 0.0f) {
      analogWrite(pairs[i].bck_pin, 0);
      analogWrite(pairs[i].fwd_pin, duty);
    } else {
      analogWrite(pairs[i].fwd_pin, 0);
      analogWrite(pairs[i].bck_pin, duty);
    }
  }
};

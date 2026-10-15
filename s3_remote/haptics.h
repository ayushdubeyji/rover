#pragma once
#include <Arduino.h>

enum HapticEffect {
  FX_NONE = 0,
  FX_TICK,
  FX_CLICK,
  FX_DOUBLE_CLICK,
  FX_MODE_SWITCH,
  FX_THROTTLE_BOOST,
  FX_BRAKE,
  FX_ALERT,
  FX_ESTOP,
  FX_STARTUP,
  FX_CLUTCH_ENGAGE,
  FX_CLUTCH_RELEASE,
  FX_SUCCESS
};

// Modern Easing Functions for Natural Tactile Envelopes
static inline float haptSmoothStep(float t) {
  t = constrain(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

static inline float haptSmootherStep(float t) {
  t = constrain(t, 0.0f, 1.0f);
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float haptSineBell(float t) {
  t = constrain(t, 0.0f, 1.0f);
  return sinf(t * 3.14159265f);
}

class HapticEngine {
public:
  bool enabled = true;
  uint8_t pin = 21;
  uint8_t strength = 60; // 0 to 100% master intensity

  void begin(uint8_t hapticPin = 21) {
    pin = hapticPin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pin, 1000, 8);
    ledcWrite(pin, 0);
#else
    ledcSetup(0, 1000, 8);
    ledcAttachPin(pin, 0);
    ledcWrite(0, 0);
#endif
    _lastFreq = 1000;
  }

  void play(HapticEffect effect) {
    if (!enabled || strength == 0) return;
    _currentFx = effect;
    _fxStart = millis();
  }

  void update() {
    if (_currentFx == FX_NONE) return;
    unsigned long elapsed = millis() - _fxStart;
    uint8_t duty = 0;
    uint32_t freq = 220; // Default gentle resonant vibration frequency

    switch (_currentFx) {
      case FX_TICK: {
        // Soft organic rotary notch (35ms): Crisp 1600 Hz blip, toned-down duty
        const unsigned long dur = 35;
        if (elapsed < dur) {
          freq = 1600;
          float u = (float)elapsed / (float)dur;
          duty = (uint8_t)(95.0f * haptSineBell(u));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_CLICK: {
        // Two-tone pleasant chime (70ms): C6 (1046Hz) -> E6 (1318Hz)
        if (elapsed < 30) {
          freq = 1046;
          duty = (uint8_t)(115.0f * haptSineBell((float)elapsed / 30.0f));
        } else if (elapsed < 70) {
          freq = 1318;
          duty = (uint8_t)(125.0f * haptSineBell((float)(elapsed - 30) / 40.0f));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_DOUBLE_CLICK: {
        // 3-note ascending arpeggio (110ms): C5 (523Hz) -> E5 (659Hz) -> G5 (784Hz)
        if (elapsed < 35) {
          freq = 523;
          duty = (uint8_t)(105.0f * haptSineBell((float)elapsed / 35.0f));
        } else if (elapsed < 70) {
          freq = 659;
          duty = (uint8_t)(115.0f * haptSineBell((float)(elapsed - 35) / 35.0f));
        } else if (elapsed < 110) {
          freq = 784;
          duty = (uint8_t)(125.0f * haptSineBell((float)(elapsed - 70) / 40.0f));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_MODE_SWITCH: {
        // Melodic drive mode fanfare (130ms): D5 (587Hz) -> A5 (880Hz) -> D6 (1175Hz)
        if (elapsed < 40) {
          freq = 587;
          duty = (uint8_t)(110.0f * haptSineBell((float)elapsed / 40.0f));
        } else if (elapsed < 80) {
          freq = 880;
          duty = (uint8_t)(125.0f * haptSineBell((float)(elapsed - 40) / 40.0f));
        } else if (elapsed < 130) {
          freq = 1175;
          duty = (uint8_t)(135.0f * haptSineBell((float)(elapsed - 80) / 50.0f));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_STARTUP: {
        // 4-note boot chime (220ms): C5 -> E5 -> G5 -> C6
        if (elapsed < 50) {
          freq = 523;
          duty = 95;
        } else if (elapsed < 100) {
          freq = 659;
          duty = 110;
        } else if (elapsed < 150) {
          freq = 784;
          duty = 120;
        } else if (elapsed < 220) {
          freq = 1046;
          duty = (uint8_t)(130.0f * (1.0f - (float)(elapsed - 150) / 70.0f));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_SUCCESS: {
        // Joyful bright confirmation chime (100ms): G5 (784Hz) -> C6 (1046Hz)
        if (elapsed < 40) {
          freq = 784;
          duty = (uint8_t)(110.0f * haptSineBell((float)elapsed / 40.0f));
        } else if (elapsed < 100) {
          freq = 1046;
          duty = (uint8_t)(125.0f * haptSineBell((float)(elapsed - 40) / 60.0f));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_CLUTCH_ENGAGE: {
        // Push-to-gyrate clutch engage: high confidence chirp (30ms)
        const unsigned long dur = 30;
        if (elapsed < dur) {
          freq = 1400;
          duty = (uint8_t)(110.0f * haptSineBell((float)elapsed / (float)dur));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_CLUTCH_RELEASE: {
        // Push-to-gyrate clutch release: soft disengage pip (25ms)
        const unsigned long dur = 25;
        if (elapsed < dur) {
          freq = 700;
          duty = (uint8_t)(85.0f * haptSineBell((float)elapsed / (float)dur));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_THROTTLE_BOOST: {
        // Smooth spool-up rising frequency chirp (200ms)
        const unsigned long dur = 200;
        if (elapsed < dur) {
          float u = (float)elapsed / (float)dur;
          freq = (uint32_t)(380.0f + 620.0f * u); // 380 Hz -> 1000 Hz sweep
          duty = (uint8_t)(135.0f * haptSmootherStep(u));
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_BRAKE: {
        // Dynamic braking bite (180ms): gentle ABS vibration flutter
        const unsigned long dur = 180;
        if (elapsed < dur) {
          freq = 200; // gentle resonance
          float u = (float)elapsed / (float)dur;
          float flutter = 0.75f + 0.25f * cosf((float)elapsed * 0.12f);
          duty = (uint8_t)(130.0f * (1.0f - u) * flutter);
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_ALERT: {
        // Warning siren (260ms): alternating 880Hz / 587Hz
        if (elapsed < 260) {
          unsigned long t = elapsed % 130;
          freq = (t < 65) ? 880 : 587;
          duty = 115;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_ESTOP: {
        // Rapid alarm beeps (280ms)
        freq = 1760;
        unsigned long t = elapsed % 100;
        if (elapsed < 280) {
          duty = (t < 55) ? 135 : 0;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      default:
        duty = 0;
        _currentFx = FX_NONE;
        break;
    }

    // Apply master strength scaling and safety clamp
    uint8_t scaledDuty = 0;
    if (enabled && strength > 0 && duty > 0) {
      scaledDuty = (uint8_t)(((uint32_t)duty * strength) / 100);
      if (scaledDuty < 4 && duty > 0 && strength > 0) scaledDuty = 4;
    }

    if (scaledDuty > 0) {
      _setFreq(freq);
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, scaledDuty);
#else
    ledcWrite(0, scaledDuty);
#endif
  }

  void stop() {
    _currentFx = FX_NONE;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, 0);
#else
    ledcWrite(0, 0);
#endif
  }

private:
  HapticEffect _currentFx = FX_NONE;
  unsigned long _fxStart = 0;
  uint32_t _lastFreq = 1000;

  void _setFreq(uint32_t freq) {
    if (freq == _lastFreq || freq == 0) return;
    _lastFreq = freq;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcChangeFrequency(pin, freq, 8);
#else
    ledcSetup(0, freq, 8);
#endif
  }
};


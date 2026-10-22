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

class HapticEngine {
public:
  bool enabled = true;
  uint8_t pin = 21;
  uint8_t strength = 100; // 0 to 100% master intensity

  void begin(uint8_t hapticPin = 21) {
    pin = hapticPin;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(pin, 2000, 8, 4);
    ledcWrite(pin, 0);
#else
    ledcSetup(4, 2000, 8);
    ledcAttachPin(pin, 4);
    ledcWrite(4, 0);
#endif
    _currentFx = FX_NONE;
  }

  void play(HapticEffect effect) {
    if (!enabled || strength == 0 || effect == FX_NONE) return;
    // Debounce duplicate triggers while effect is actively playing
    if (_currentFx == effect && (millis() - _fxStart < 220)) return;
    _currentFx = effect;
    _fxStart = millis();
    update();
  }

  void update() {
    HapticEffect fx = _currentFx;
    if (fx == FX_NONE) return;

    unsigned long elapsed = millis() - _fxStart;
    uint8_t duty = 0;

    switch (fx) {
      case FX_TICK: {
        // Crisp 25ms tactile tick
        if (elapsed < 25) {
          duty = 180;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }
      case FX_CLICK: {
        // Sharp 45ms tactile click
        if (elapsed < 45) {
          duty = 220;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }
      case FX_DOUBLE_CLICK: {
        // Double pulse click (35ms on, 35ms off, 35ms on)
        if (elapsed < 35) {
          duty = 220;
        } else if (elapsed < 70) {
          duty = 0;
        } else if (elapsed < 105) {
          duty = 220;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }
      case FX_ALERT: {
        // Crisp attention alert (70ms buzz, 50ms pause, 70ms buzz)
        if (elapsed < 70) {
          duty = 255;
        } else if (elapsed < 120) {
          duty = 0;
        } else if (elapsed < 190) {
          duty = 255;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }
      case FX_SUCCESS: {
        // Pleasant ascending confirmation (40ms mid, 60ms high)
        if (elapsed < 40) {
          duty = 160;
        } else if (elapsed < 100) {
          duty = 240;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }
      default: {
        if (elapsed < 50) {
          duty = 180;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }
    }

    uint32_t scaledDuty = 0;
    if (enabled && strength > 0 && duty > 0) {
      scaledDuty = ((uint32_t)duty * strength) / 100;
      if (scaledDuty > 255) scaledDuty = 255;
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, scaledDuty);
#else
    ledcWrite(4, scaledDuty);
#endif
    if (scaledDuty == 0) {
      digitalWrite(pin, LOW);
    }
  }

  void stop() {
    _currentFx = FX_NONE;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, 0);
#else
    ledcWrite(4, 0);
#endif
    digitalWrite(pin, LOW);
  }

private:
  volatile HapticEffect _currentFx = FX_NONE;
  volatile unsigned long _fxStart = 0;
};

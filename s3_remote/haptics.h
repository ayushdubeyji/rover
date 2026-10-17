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

#include <esp_timer.h>

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
    // Use dedicated Channel 4 (Timer 2) so Haptics NEVER shares Timer 0 with LCD_BL_PIN analogWrite!
    ledcAttachChannel(pin, 2000, 8, 4);
    ledcWrite(pin, 0);
#else
    ledcSetup(4, 2000, 8);
    ledcAttachPin(pin, 4);
    ledcWrite(4, 0);
#endif
    _lastFreq = 2000;

    // 500 Hz (2ms) hardware timer on Core 0 so haptics never lag or stall during 3D SPI rendering
    if (!_timer) {
      esp_timer_create_args_t args = {};
      args.callback = &_timerCb;
      args.arg = this;
      args.dispatch_method = ESP_TIMER_TASK;
      args.name = "hapt_tmr";
      if (esp_timer_create(&args, &_timer) == ESP_OK) {
        esp_timer_start_periodic(_timer, 2000); // 2000 us = 2 ms precision
      }
    }
  }

  void play(HapticEffect effect) {
    if (!enabled || strength == 0) return;
    _currentFx = effect;
    _fxStart = millis();
    update(); // Immediate t=0ms hardware kick (zero loop latency)
  }

  void update() {
    HapticEffect fx = _currentFx;
    if (fx == FX_NONE) return;
    unsigned long elapsed = millis() - _fxStart;
    uint8_t duty = 0;     // 255 = 100% pure DC overdrive spin-up, 160..185 = max-dB square wave tone
    uint32_t freq = 250;

    switch (fx) {
      case FX_TICK: {
        // Instant 2500Hz acoustic click (14ms) + 50ms 100% DC overdrive physical kick
        if (elapsed < 14) {
          freq = 2500;
          duty = 170;
        } else if (elapsed < 64) {
          freq = 250;
          duty = 255; // 100% DC overdrive spin-up
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_CLICK: {
        // 55ms 100% DC physical punch + crisp 2-tone high-dB chime (1760Hz -> 2400Hz)
        if (elapsed < 55) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 80) {
          freq = 1760;
          duty = 170;
        } else if (elapsed < 110) {
          freq = 2400;
          duty = 175;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_DOUBLE_CLICK: {
        // Double 100% DC physical punch interleaved with ascending chime
        if (elapsed < 50) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 75) {
          freq = 1568;
          duty = 170;
        } else if (elapsed < 120) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 155) {
          freq = 2349;
          duty = 175;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_MODE_SWITCH: {
        // Strong 65ms 100% DC punch + 3-note bright fanfare (D6 -> A6 -> D7)
        if (elapsed < 65) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 95) {
          freq = 1318;
          duty = 170;
        } else if (elapsed < 125) {
          freq = 1760;
          duty = 170;
        } else if (elapsed < 165) {
          freq = 2489;
          duty = 175;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_STARTUP: {
        // 80ms 100% DC wakeup thump + 4-note loud boot fanfare
        if (elapsed < 80) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 120) {
          freq = 1318;
          duty = 170;
        } else if (elapsed < 160) {
          freq = 1568;
          duty = 170;
        } else if (elapsed < 200) {
          freq = 1976;
          duty = 175;
        } else if (elapsed < 260) {
          freq = 2637;
          duty = 175;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_SUCCESS: {
        // 60ms 100% DC punch + bright confirmation chime
        if (elapsed < 60) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 95) {
          freq = 1760;
          duty = 170;
        } else if (elapsed < 140) {
          freq = 2637;
          duty = 175;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_CLUTCH_ENGAGE: {
        // 55ms 100% DC kick + 2200Hz chirp
        if (elapsed < 55) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 80) {
          freq = 2200;
          duty = 175;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_CLUTCH_RELEASE: {
        // 40ms 100% DC kick + 1400Hz pip
        if (elapsed < 40) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 65) {
          freq = 1400;
          duty = 165;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_THROTTLE_BOOST: {
        // 90ms 100% DC full-power kick + 4 discrete stepped turbo notes (no continuous freq thrashing)
        if (elapsed < 90) {
          freq = 250;
          duty = 255;
        } else if (elapsed < 115) {
          freq = 1400;
          duty = 170;
        } else if (elapsed < 140) {
          freq = 1800;
          duty = 170;
        } else if (elapsed < 165) {
          freq = 2200;
          duty = 175;
        } else if (elapsed < 195) {
          freq = 2637;
          duty = 175;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_BRAKE: {
        // Heavy pulsed 100% DC ABS mechanical rumble (3 hard kicks)
        if (elapsed < 190) {
          unsigned long t = elapsed % 65;
          freq = 250;
          duty = (t < 50) ? 255 : 0;
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_ALERT: {
        // Alternating 100% DC physical shake + 2400Hz siren
        if (elapsed < 260) {
          unsigned long t = elapsed % 130;
          if (t < 65) {
            freq = 250;
            duty = 255;
          } else {
            freq = 2400;
            duty = 175;
          }
        } else {
          _currentFx = FX_NONE;
        }
        break;
      }

      case FX_ESTOP: {
        // Urgent alternating 100% DC slam + 2637Hz alarm
        if (elapsed < 280) {
          unsigned long t = elapsed % 95;
          if (t < 50) {
            freq = 250;
            duty = 255;
          } else if (t < 85) {
            freq = 2637;
            duty = 175;
          } else {
            duty = 0;
          }
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

    uint32_t scaledDuty = 0;
    if (enabled && strength > 0 && duty > 0) {
      if (duty == 255) {
        // Physical overdrive phase: saturate transistor at 256 (100% solid DC) when strength >= 90%
        scaledDuty = (strength >= 90) ? 256 : (155 + strength);
      } else {
        // Acoustic square-wave phase: keep duty near 55-68% (140-175) for maximum AC decibel output
        scaledDuty = ((uint32_t)duty * (40 + (strength * 60) / 100)) / 100;
      }
    }

    if (scaledDuty > 0 && scaledDuty < 256) {
      _setFreq(freq);
    }

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, scaledDuty);
#else
    ledcWrite(4, scaledDuty);
#endif
  }

  void stop() {
    _currentFx = FX_NONE;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, 0);
#else
    ledcWrite(4, 0);
#endif
  }

private:
  volatile HapticEffect _currentFx = FX_NONE;
  volatile unsigned long _fxStart = 0;
  uint32_t _lastFreq = 2000;
  esp_timer_handle_t _timer = nullptr;

  static void _timerCb(void* arg) {
    ((HapticEngine*)arg)->update();
  }

  void _setFreq(uint32_t freq) {
    if (freq == _lastFreq || freq == 0) return;
    _lastFreq = freq;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcChangeFrequency(pin, freq, 8);
#else
    ledcSetup(4, freq, 8);
#endif
  }
};


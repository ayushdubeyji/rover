#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>
#include "rover_link.h"

// ==============================================================================
// Hardware Configuration for LOLIN C3 Mini (ESP32-C3)
// ==============================================================================

// UART0 to RP2040-Zero (GP0 TX -> C3 RX 20, GP1 RX <- C3 TX 21)
#define UART_RP_RX                  20
#define UART_RP_TX                  21
#define UART_RP_BAUD                ROVER_UART0_BAUD_DEFAULT  // 921600

// WS2812 RGB LED Strip
#define STATUS_LED_PIN              5
#define STATUS_LED_COUNT            40
#define STATUS_LED_MAX_BRIGHTNESS   70   // Hardware brightness capped at 70 (~27%)

// LED Layout Configuration:
// 0 = Continuous perimeter ring (LED 0..39 around chassis)
// 1 = Dual lateral runners (Left runner 0..19, Right runner 20..39)
#ifndef LED_LAYOUT_LATERAL_RUNNERS
#define LED_LAYOUT_LATERAL_RUNNERS  0
#endif

// Optional wheel pulse counter interrupts on GPIO 0, 1, 3, 4 (disabled by default)
#define ENABLE_C3_ENCODERS          0
#define PIN_ENC_FL                  0
#define PIN_ENC_FR                  1
#define PIN_ENC_RL                  3
#define PIN_ENC_RR                  4

// ==============================================================================
// State & Peripherals
// ==============================================================================
HardwareSerial SerialRP(0);
Adafruit_NeoPixel strip(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

RoverFrameParser rp_parser;
uint8_t tx_frame_buf[ROVER_LINK_MAX_FRAME_SIZE];

uint8_t remoteMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool remoteKnown = false;
uint32_t lastRemoteRecvMs = 0;

DriveMode cached_mode = MODE_NORMAL;
AutoNavState current_auto_state = AUTO_CRUISE_CLEAR;
float cached_heading = 0.0f;
bool cached_crash_prot = true;
bool cached_impact = false;

// Optional encoder tick counters
#if ENABLE_C3_ENCODERS
volatile int32_t enc_counts[4] = {0, 0, 0, 0};
void IRAM_ATTR isr_enc_fl() { enc_counts[0]++; }
void IRAM_ATTR isr_enc_fr() { enc_counts[1]++; }
void IRAM_ATTR isr_enc_rl() { enc_counts[2]++; }
void IRAM_ATTR isr_enc_rr() { enc_counts[3]++; }
#endif

// Forward declarations
void handle_rp2040_frame(uint8_t msg_type, const uint8_t *payload, uint8_t len);
void update_led_state_machine(uint32_t now_ms);

// ==============================================================================
// ESP-NOW Receive Callback (S3 Remote -> C3)
// ==============================================================================
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *src_mac = info->src_addr;
#else
void onEspNowRecv(const uint8_t *src_mac, const uint8_t *data, int len) {
#endif
  if (len != sizeof(RoverPkt) || data[0] != PKT_MAGIC) return;

  RoverPkt pkt;
  memcpy(&pkt, data, sizeof(RoverPkt));

  // Auto-pair with sender
  if (!remoteKnown || memcmp(remoteMac, src_mac, 6) != 0) {
    if (esp_now_is_peer_exist(src_mac)) {
      esp_now_del_peer(src_mac);
    }
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, src_mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    memcpy(remoteMac, src_mac, 6);
    remoteKnown = true;
  }
  lastRemoteRecvMs = millis();
  cached_mode = (DriveMode)(pkt.mode % MODE_COUNT);

  // 1. Pack into framed protocol and forward to RP2040-Zero over UART0
  size_t frame_len = rover_pack_frame(MSG_ROVER_CMD, &pkt, sizeof(RoverPkt),
                                      tx_frame_buf, sizeof(tx_frame_buf));
  if (frame_len > 0) {
    SerialRP.write(tx_frame_buf, frame_len);
  }

  // 2. Return immediate low-latency ACK to S3 Remote
  RoverPkt ack = {};
  ack.magic = PKT_MAGIC;
  ack.type = PKT_ACK;
  ack.mode = cached_mode;
  ack.speed = pkt.speed;
  ack.flags = (cached_mode == MODE_AUTO ? FLAG_AUTO : 0) |
              (cached_crash_prot ? 0 : FLAG_CRASH_OFF) |
              (cached_impact ? FLAG_IMPACT : 0);
  ack.seq = pkt.seq;
  ack.arg = (int16_t)lroundf(fmodf(cached_heading + 3600.0f, 360.0f));
  ack.x = (int8_t)current_auto_state;
  esp_now_send(remoteMac, (const uint8_t *)&ack, sizeof(ack));
}

// ==============================================================================
// UART0 Binary Frame Processor (RP2040-Zero -> C3)
// ==============================================================================
void handle_rp2040_frame(uint8_t msg_type, const uint8_t *payload, uint8_t len) {
  if (msg_type == MSG_TELEM && len >= sizeof(TelemPkt)) {
    TelemPkt telem;
    memcpy(&telem, payload, sizeof(TelemPkt));

    // Extract telemetry attributes
    current_auto_state = (AutoNavState)(telem.batMv & 0xFF);
    cached_crash_prot = !(telem.calFlags & FLAG_CRASH_OFF);
    cached_impact = (telem.calFlags & FLAG_IMPACT);

    // Approximate heading from quaternion if available: hdg = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))
    float q0 = (float)telem.q[0] / 16384.0f;
    float q1 = (float)telem.q[1] / 16384.0f;
    float q2 = (float)telem.q[2] / 16384.0f;
    float q3 = (float)telem.q[3] / 16384.0f;
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    cached_heading = atan2f(siny_cosp, cosy_cosp) * 57.2957795f;

    // Forward telemetry to S3 Remote if paired and active
    if (remoteKnown && (millis() - lastRemoteRecvMs < 2000)) {
      esp_now_send(remoteMac, (const uint8_t *)&telem, sizeof(TelemPkt));
    }
  } else if (msg_type == MSG_ACK && len >= sizeof(RoverPkt)) {
    if (remoteKnown) {
      esp_now_send(remoteMac, payload, sizeof(RoverPkt));
    }
  }
}

// ==============================================================================
// WS2812 Helper Utilities
// ==============================================================================
static inline uint32_t color_rgb(uint8_t r, uint8_t g, uint8_t b) {
  return strip.Color(r, g, b);
}

// Blend between two RGB colors (t: 0..255)
static inline uint32_t color_blend(uint32_t c1, uint32_t c2, uint8_t t) {
  uint8_t r1 = (c1 >> 16) & 0xFF, g1 = (c1 >> 8) & 0xFF, b1 = c1 & 0xFF;
  uint8_t r2 = (c2 >> 16) & 0xFF, g2 = (c2 >> 8) & 0xFF, b2 = c2 & 0xFF;
  uint8_t r = ((uint16_t)r1 * (255 - t) + (uint16_t)r2 * t) / 255;
  uint8_t g = ((uint16_t)g1 * (255 - t) + (uint16_t)g2 * t) / 255;
  uint8_t b = ((uint16_t)b1 * (255 - t) + (uint16_t)b2 * t) / 255;
  return color_rgb(r, g, b);
}

// Set all LEDs to a uniform color
static void set_all_leds(uint32_t color) {
  for (int i = 0; i < STATUS_LED_COUNT; i++) {
    strip.setPixelColor(i, color);
  }
}

// ==============================================================================
// WS2812 12-State Visual Animation Machine
// ==============================================================================
void update_led_state_machine(uint32_t now_ms) {
  bool link_lost = (!remoteKnown || (now_ms - lastRemoteRecvMs > 1000));
  bool is_auto = (cached_mode == MODE_AUTO);

  // --- Link Lost: Deep Purple Heartbeat Pulse (2 Hz) ---
  if (link_lost) {
    uint32_t cycle = now_ms % 500; // 2 Hz cycle
    uint8_t brightness = 0;
    if (cycle < 100) {
      brightness = (uint8_t)((cycle * 255) / 100);
    } else if (cycle < 200) {
      brightness = (uint8_t)(((200 - cycle) * 255) / 100);
    } else if (cycle < 300) {
      brightness = (uint8_t)(((cycle - 200) * 180) / 100);
    } else if (cycle < 400) {
      brightness = (uint8_t)(((400 - cycle) * 180) / 100);
    }
    // #7E22CE = (126, 34, 206)
    uint8_t r = (126 * brightness) / 255;
    uint8_t g = (34  * brightness) / 255;
    uint8_t b = (206 * brightness) / 255;
    set_all_leds(color_rgb(r, g, b));
    strip.show();
    return;
  }

  // --- Manual Mode: Dim Ice Blue (#38BDF8) Constant Gentle Glow ---
  if (!is_auto) {
    // #38BDF8 = (56, 189, 248), hardware brightness capped at 70 (~27%)
    set_all_leds(color_rgb(56, 189, 248));
    strip.show();
    return;
  }

  // --- 12-State Autonomy Engine Visuals ---
  switch (current_auto_state) {

    // 0: Cruise Clear - Emerald Breathing (#00FF44) at 1 Hz
    case AUTO_CRUISE_CLEAR: {
      uint32_t cycle = now_ms % 1000;
      float phase = (float)cycle / 1000.0f * 6.283185f;
      float breath = 0.25f + 0.75f * (0.5f + 0.5f * sinf(phase));
      uint8_t g = (uint8_t)(255.0f * breath);
      uint8_t r = 0;
      uint8_t b = (uint8_t)(68.0f * breath);
      set_all_leds(color_rgb(r, g, b));
      break;
    }

    // 1: Thinking Scan - Warm Amber Chase (#FFB700)
    case AUTO_THINKING_SCAN: {
      strip.clear();
#if LED_LAYOUT_LATERAL_RUNNERS
      uint8_t head = (now_ms / 35) % 20;
      for (int i = 0; i < 6; i++) {
        int idx = (head - i + 20) % 20;
        uint8_t fade = 255 - (i * 40);
        uint32_t c = color_rgb((255 * fade) / 255, (183 * fade) / 255, 0);
        strip.setPixelColor(idx, c);
        strip.setPixelColor(20 + idx, c);
      }
#else
      uint8_t head = (now_ms / 35) % STATUS_LED_COUNT;
      for (int i = 0; i < 6; i++) {
        int idx = (head - i + STATUS_LED_COUNT) % STATUS_LED_COUNT;
        uint8_t fade = 255 - (i * 40);
        uint8_t r = (255 * fade) / 255;
        uint8_t g = (183 * fade) / 255;
        strip.setPixelColor(idx, color_rgb(r, g, 0));
      }
#endif
      break;
    }

    // 2: Planning Path - Electric Blue Split Wave (#0088FF)
    case AUTO_PLANNING_PATH: {
      strip.clear();
#if LED_LAYOUT_LATERAL_RUNNERS
      uint8_t wave_pos = (now_ms / 35) % 20;
      for (int i = 0; i < 20; i++) {
        int dist = abs(i - (int)wave_pos);
        if (dist < 4) {
          uint8_t bright = 255 - (dist * 60);
          uint32_t c = color_rgb(0, (136 * bright) / 255, (255 * bright) / 255);
          strip.setPixelColor(i, c);
          strip.setPixelColor(20 + i, c);
        }
      }
#else
      int half = STATUS_LED_COUNT / 2;
      uint8_t wave_pos = (now_ms / 40) % half;
      for (int i = 0; i < half; i++) {
        int dist = abs(i - (int)wave_pos);
        if (dist < 4) {
          uint8_t bright = 255 - (dist * 60);
          uint32_t c = color_rgb(0, (136 * bright) / 255, (255 * bright) / 255);
          strip.setPixelColor(i, c);
          strip.setPixelColor(STATUS_LED_COUNT - 1 - i, c);
        }
      }
#endif
      break;
    }

    // 3: Veer Left - Tangerine Orange (#FF5500) Left-directed Wipe
    case AUTO_VEER_LEFT: {
      strip.clear();
#if LED_LAYOUT_LATERAL_RUNNERS
      // Highlight left lateral runner (LED 0..19) with traveling wipe
      uint8_t pos = (now_ms / 30) % 20;
      for (int i = 0; i < 20; i++) {
        int dist = abs(i - (int)pos);
        if (dist < 4) {
          uint8_t b = 255 - dist * 60;
          strip.setPixelColor(i, color_rgb((255 * b) / 255, (85 * b) / 255, 0));
        }
      }
#else
      // Perimeter wipe concentrated on left half (0..19)
      uint8_t pos = (now_ms / 35) % (STATUS_LED_COUNT / 2);
      for (int i = 0; i < (STATUS_LED_COUNT / 2); i++) {
        int dist = abs(i - (int)pos);
        if (dist < 4) {
          uint8_t b = 255 - dist * 60;
          strip.setPixelColor(i, color_rgb((255 * b) / 255, (85 * b) / 255, 0));
        }
      }
#endif
      break;
    }

    // 4: Veer Right - Tangerine Orange (#FF5500) Right-directed Wipe
    case AUTO_VEER_RIGHT: {
      strip.clear();
#if LED_LAYOUT_LATERAL_RUNNERS
      // Highlight right lateral runner (LED 20..39) with traveling wipe
      uint8_t pos = (now_ms / 30) % 20;
      for (int i = 0; i < 20; i++) {
        int dist = abs(i - (int)pos);
        if (dist < 4) {
          uint8_t b = 255 - dist * 60;
          strip.setPixelColor(20 + i, color_rgb((255 * b) / 255, (85 * b) / 255, 0));
        }
      }
#else
      // Perimeter wipe concentrated on right half (20..39)
      int half = STATUS_LED_COUNT / 2;
      uint8_t pos = (now_ms / 35) % half;
      for (int i = 0; i < half; i++) {
        int dist = abs(i - (int)pos);
        if (dist < 4) {
          uint8_t b = 255 - dist * 60;
          strip.setPixelColor(half + i, color_rgb((255 * b) / 255, (85 * b) / 255, 0));
        }
      }
#endif
      break;
    }

    // 5: Cautious Creep - Cyan (#00FFFF) Slow Gentle Pulse
    case AUTO_CAUTIOUS_CREEP: {
      uint32_t cycle = now_ms % 2000;
      float phase = (float)cycle / 2000.0f * 6.283185f;
      float p = 0.15f + 0.35f * (0.5f + 0.5f * sinf(phase));
      uint8_t b = (uint8_t)(255.0f * p);
      set_all_leds(color_rgb(0, b, b));
      break;
    }

    // 6: Stuck Detected - Crimson Red (#FF0000) 5 Hz Strobe
    case AUTO_STUCK_DETECTED: {
      uint32_t cycle = now_ms % 200; // 5 Hz
      if (cycle < 100) {
        set_all_leds(color_rgb(255, 0, 0));
      } else {
        set_all_leds(color_rgb(0, 0, 0));
      }
      break;
    }

    // 7: Reverse Backtrack - Magenta (#FF0088) Reverse Flow
    case AUTO_REVERSE_BACKTRACK: {
      strip.clear();
#if LED_LAYOUT_LATERAL_RUNNERS
      uint8_t head = 19 - ((now_ms / 35) % 20);
      for (int i = 0; i < 6; i++) {
        int idx = (head + i) % 20;
        uint8_t fade = 255 - (i * 40);
        uint32_t c = color_rgb((255 * fade) / 255, 0, (136 * fade) / 255);
        strip.setPixelColor(idx, c);
        strip.setPixelColor(20 + idx, c);
      }
#else
      uint8_t head = (STATUS_LED_COUNT - 1) - ((now_ms / 40) % STATUS_LED_COUNT);
      for (int i = 0; i < 7; i++) {
        int idx = (head + i) % STATUS_LED_COUNT;
        uint8_t fade = 255 - (i * 35);
        strip.setPixelColor(idx, color_rgb((255 * fade) / 255, 0, (136 * fade) / 255));
      }
#endif
      break;
    }

    // 8: S-Curve Escape - Amber/Orange (#FF8800) Alternating Flow
    case AUTO_SCURVE_ESCAPE: {
      strip.clear();
      uint8_t offset = (now_ms / 60) % 6;
      for (int i = 0; i < STATUS_LED_COUNT; i++) {
        if (((i + offset) % 6) < 3) {
          strip.setPixelColor(i, color_rgb(255, 136, 0));
        }
      }
      break;
    }

    // 9: Compass Pivot - Amber Orbit Spin (#FFB700)
    case AUTO_COMPASS_PIVOT: {
      strip.clear();
#if LED_LAYOUT_LATERAL_RUNNERS
      uint8_t head = (now_ms / 25) % 20;
      for (int i = 0; i < 5; i++) {
        int idx_l = (head - i + 20) % 20;
        int idx_r = (19 - ((head - i + 20) % 20));
        strip.setPixelColor(idx_l, color_rgb(255, 183, 0));
        strip.setPixelColor(20 + idx_r, color_rgb(255, 183, 0));
      }
#else
      uint8_t head = (now_ms / 25) % STATUS_LED_COUNT;
      for (int i = 0; i < 5; i++) {
        int idx = (head - i + STATUS_LED_COUNT) % STATUS_LED_COUNT;
        strip.setPixelColor(idx, color_rgb(255, 183, 0));
      }
#endif
      break;
    }

    // 10: Wall Follow - Cyan/Blue Lateral Contour (#00D0FF)
    case AUTO_WALL_FOLLOW: {
      uint32_t cycle = now_ms % 600;
      float phase = (float)cycle / 600.0f * 6.283185f;
      float val = 0.3f + 0.5f * (0.5f + 0.5f * sinf(phase));
      uint8_t g = (uint8_t)(208.0f * val);
      uint8_t b = (uint8_t)(255.0f * val);
      set_all_leds(color_rgb(0, g, b));
      break;
    }

    // 11: Impact Reflex - Immediate 50 ms White Flash (#FFFFFF)
    case AUTO_IMPACT_REFLEX: {
      uint32_t cycle = now_ms % 100;
      if (cycle < 50) {
        set_all_leds(color_rgb(255, 255, 255));
      } else {
        set_all_leds(color_rgb(0, 0, 0));
      }
      break;
    }

    default:
      set_all_leds(color_rgb(0, 40, 20));
      break;
  }

  strip.show();
}

// ==============================================================================
// Arduino Setup & Loop
// ==============================================================================
void setup() {
  // 1. Initialize Hardware UART0 to RP2040-Zero
  SerialRP.begin(UART_RP_BAUD, SERIAL_8N1, UART_RP_RX, UART_RP_TX);
  rover_parser_init(&rp_parser);

  // 2. Initialize Optional Encoder Pins (disabled by default)
#if ENABLE_C3_ENCODERS
  pinMode(PIN_ENC_FL, INPUT_PULLUP);
  pinMode(PIN_ENC_FR, INPUT_PULLUP);
  pinMode(PIN_ENC_RL, INPUT_PULLUP);
  pinMode(PIN_ENC_RR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_FL), isr_enc_fl, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_FR), isr_enc_fr, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RL), isr_enc_rl, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RR), isr_enc_rr, RISING);
#else
  pinMode(PIN_ENC_FL, INPUT_PULLUP);
  pinMode(PIN_ENC_FR, INPUT_PULLUP);
  pinMode(PIN_ENC_RL, INPUT_PULLUP);
  pinMode(PIN_ENC_RR, INPUT_PULLUP);
#endif

  // 3. Initialize WS2812 LED Strip
  strip.begin();
  strip.setBrightness(STATUS_LED_MAX_BRIGHTNESS);
  set_all_leds(color_rgb(20, 0, 40)); // Initial boot purple
  strip.show();

  // 4. Initialize ESP-NOW in STA mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);

    // Register broadcast peer for discovery
    esp_now_peer_info_t bcast = {};
    memset(bcast.peer_addr, 0xFF, 6);
    bcast.channel = 0;
    bcast.ifidx = WIFI_IF_STA;
    bcast.encrypt = false;
    esp_now_add_peer(&bcast);
  }
}

void loop() {
  uint32_t now = millis();

  // 1. Read UART0 bytes from RP2040-Zero
  while (SerialRP.available()) {
    uint8_t b = SerialRP.read();
    if (rover_parser_feed(&rp_parser, b)) {
      handle_rp2040_frame(rp_parser.msg_type, rp_parser.payload, rp_parser.len);
    }
  }

  // 2. Optional Encoder Telemetry Dispatch (20 Hz)
  static uint32_t last_enc_ms = 0;
  if (now - last_enc_ms >= 50) {
    last_enc_ms = now;
    EncoderPkt epkt = {};
#if ENABLE_C3_ENCODERS
    epkt.valid_mask = 0x0F;
    epkt.enc_count[0] = enc_counts[0];
    epkt.enc_count[1] = enc_counts[1];
    epkt.enc_count[2] = enc_counts[2];
    epkt.enc_count[3] = enc_counts[3];
#else
    epkt.valid_mask = 0x00; // Disabled by default
#endif
    size_t f_len = rover_pack_frame(MSG_ENCODER, &epkt, sizeof(EncoderPkt),
                                    tx_frame_buf, sizeof(tx_frame_buf));
    if (f_len > 0) SerialRP.write(tx_frame_buf, f_len);
  }

  // 3. WS2812 LED State Machine Ticker (30 Hz)
  static uint32_t last_led_ms = 0;
  if (now - last_led_ms >= 33) {
    last_led_ms = now;
    update_led_state_machine(now);
  }
}

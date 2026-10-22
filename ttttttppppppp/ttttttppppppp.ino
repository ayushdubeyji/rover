/*
  =====================================================================
  MASTER SKETCH v2.0 — ESP32-C3 Supermini HID Keypad / Encoder Controller
  =====================================================================
  v2.0 changes (on top of the previous v1.4.0-ota base):

   - SWIPE GESTURE DETECTION
       Horizontal swipe  : pads 13-16 (indices 12-15, bottom row)
         touch 13->14->15->16 = Swipe Right   (fires swipeRight action)
         touch 16->15->14->13 = Swipe Left    (fires swipeLeft action)
       Vertical swipe    : pads 1,5,9,13 (indices 0,4,8,12, left column)
         touch 1->5->9->13  = Swipe Down    (fires swipeDown action)
         touch 13->9->5->1  = Swipe Up      (fires swipeUp action)
       At least 2 pads in the correct sequential order within the
       configured time window triggers the swipe. A standalone tap on
       a swipe pad (no swipe detected) still fires the pad's own action.
       All four swipe actions are configurable from the Gestures tab.

   - MULTI-SLOT ENCODER COMBOS (4 slots)
       Each slot has:  shiftPadIndex (255 = no shift / bare encoder)
                       cw action  (clockwise turn)
                       ccw action (counter-clockwise turn)
       Default combos:
         [0] Bare (no pad)   CW=Down    CCW=Up      (scroll TV menus)
         [1] Pad 13 held     CW=Right   CCW=Left    (horizontal nav)
         [2] Pad 2  held     CW=VolUp   CCW=VolDown (volume)
         [3] Disabled
       All four combos are configurable from the Encoder tab.

   - SMART SHIFT PAD
       If the encoder turns while the shift pad is held -> encoder combo
       fires, pad's own action is suppressed.
       If the shift pad is tapped (released without encoder turning) ->
       the pad's own action fires as a normal tap on release.
       This fixes the "simultaneously increasing and decreasing volume"
       bug caused by the shift pad's media key firing alongside the
       encoder combo.

   - PC MODE
       operatingMode 0 = TV/Phone  (default)
       operatingMode 1 = PC
       Each mode has its own independent 16-pad action table.
       PC defaults: Ctrl+C/V/X/Z/Y/A/S, F5, Alt+Tab, Alt+Left, Alt+F4.
       Toggle with the badge in the web UI header; configure in "PC Mode" tab.

   - EXPANDED KEY TABLES (Google TV / Android / PC)
       Media: +FastFwd, Rewind, Record, Eject, Channel Up/Down, Menu,
              Select/OK (TV), Subtitles/CC, Brightness Up/Down, MediaSelect,
              AC Search, AC Forward, AC Stop, AC Refresh, AC Bookmarks.
       All HID Consumer Control codes in correct little-endian byte order.

   - FIX: JSON document size increased to 16384 bytes to prevent truncation
     of config API responses with the expanded key tables.

   - FIX: Default encoderDetentSteps lowered from 4 to 2 (most cheap
     mechanical encoders have 2 quadrature steps per physical detent, so
     4 effectively required two full detents per output tick — making the
     encoder feel laggy/unresponsive on Google TV menus).

   - FIX: dispatchTap() delay between key press and release increased from
     15ms to 30ms for better Google TV / Android TV HID compatibility.

   - FIX: Escape / SK_ESC always sends Consumer Control AC Back (0x0224)
     alongside the keyboard Escape, so a single pad acts as Back correctly
     on both PC (Escape) and Android/Google TV (AC Back) without needing
     per-device config.

   - CONFIG_VERSION bumped to 4: forces a clean defaults load from any
     firmware older than v2.0 (struct layout changed).

  Everything else (OTA, BLE crash-loop guard, haptics engine, TTP229 scan,
  combo chords, macros, DIP switches, web server, factory reset) is kept.
*/

#define USE_NIMBLE

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_idf_version.h>
#include <BleKeyboard.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

// ============================= IR TRANSMITTER ================================
#define IR_TX_PIN         21
// Haier TV power code — confirmed captured from the original remote as SAMSUNG protocol, 32-bit
#define HAIER_POWER_CODE 0x1818D02FUL
IRsend irSend(IR_TX_PIN);

#ifndef OTA_PASSWORD
#define OTA_PASSWORD     "YOUR_OTA_PASSWORD"
#endif
#define FIRMWARE_VERSION "2.1.0"

// ============================= LIMITS / SIZES ===============================
#define NUM_PADS        16
#define MAX_COMBOS      6
#define MAX_MACROS      6
#define MACRO_LEN       24
#define MAX_ENC_COMBOS  4
#define NUM_SLOTS       (NUM_PADS + MAX_COMBOS)
#define CONFIG_MAGIC    0xC1
#define CONFIG_VERSION  5

// ============================= STRUCTS & ENUMS ==============================
struct DebouncedPin {
  uint8_t pin; bool activeLow; bool stableState; bool lastRead; unsigned long lastChangeMs;
};

enum ActionType : uint8_t {
  ACT_NONE=0, ACT_ASCII=1, ACT_SPECIAL=2, ACT_MEDIA=3, ACT_MACRO=4
};

#define MOD_CTRL  0x01
#define MOD_SHIFT 0x02
#define MOD_ALT   0x04
#define MOD_GUI   0x08

struct KeyAction    { uint8_t type; uint8_t code; uint8_t modifiers; };
struct ComboConfig  { uint16_t mask; KeyAction action; };
struct EncoderCombo { uint8_t shiftPadIndex; KeyAction cw; KeyAction ccw; };

enum OutputMode : uint8_t { MODE_ESPNOW=0, MODE_BLE_KEYBOARD=1 };

struct DeviceConfig {
  uint8_t   magic, version, outputMode;
  char      staSsid[32], staPass[64], apSsid[24], apPass[24], adminPass[24];
  uint8_t   padDebounceMs;
  uint16_t  encoderDebounceUs;
  uint8_t   encoderDetentSteps, hapticStrength;
  KeyAction    padAction[NUM_PADS];
  ComboConfig  combos[MAX_COMBOS];
  EncoderCombo encoderCombos[MAX_ENC_COMBOS];
  char         macros[MAX_MACROS][MACRO_LEN];
  uint8_t      operatingMode;          // 0=TV/Phone  1=PC
  KeyAction    padActionPC[NUM_PADS];
  uint16_t     swipeTimeWindowMs;
  KeyAction    swipeRight, swipeLeft, swipeDown, swipeUp;
};

struct NamedKey      { const char* name; uint8_t code; };
struct NamedMediaKey { const char* name; uint8_t code[2]; };

// ============================= PIN DEFINITIONS ==============================
#define SCL_PIN           0
#define SDO_PIN           1
#define ENCODER_PIN_A     3
#define ENCODER_PIN_B     4
#define MOTOR_PIN         5
#define DIP_MODE_PIN      6
#define DIP_WEBSERVER_PIN 7
#define DIP_VIBE_PIN      10

uint8_t maTouchMac[] = {0xE0,0x72,0xA1,0xAB,0x5A,0x1C};
#define ESPNOW_WIFI_CHANNEL    1
#define ESPNOW_BIT_ENCODER_CW  22
#define ESPNOW_BIT_ENCODER_CCW 23

// ============================= GLOBALS ======================================
bool configMode=false, configModeForcedByDip=false;
unsigned long configStartTime=0;
const unsigned long CONFIG_TIMEOUT=600000UL;
unsigned long holdStartConfig=0;
const unsigned long HOLD_DURATION=3500UL;
const uint16_t CONFIG_TRIGGER_MASK=0xF000;  // Pads 13,14,15,16 held 3.5s -> Config Mode

// IR hold state — Pad 13 + Pad 16 held 1.2s fires Haier power code
unsigned long holdStartIR=0;
bool irFiredThisHold=false;
const unsigned long IR_HOLD_MS=1200UL;
const uint16_t IR_TRIGGER_MASK=0x9000;      // Pad 13 (bit 12) + Pad 16 (bit 15)

DeviceConfig cfg;
Preferences  preferences;
volatile uint32_t encoderDebounceUsCache=600;
bool wifiRadioActive=false, bleActuallyActive=false, otaActive=false;
unsigned long otaProgressLastHapticPct=255;
bool bleSafeModeThisBoot=false, bleCrashCounterCleared=false;

// =============================================================================
//                    SPECIAL KEY TABLE
// =============================================================================
const NamedKey specialKeyTable[] = {
  { "Enter",         KEY_RETURN      },
  { "Escape/Back",   KEY_ESC         },
  { "Backspace",     KEY_BACKSPACE   },
  { "Tab",           KEY_TAB         },
  { "Space",         ' '             },
  { "Up Arrow",      KEY_UP_ARROW    },
  { "Down Arrow",    KEY_DOWN_ARROW  },
  { "Left Arrow",    KEY_LEFT_ARROW  },
  { "Right Arrow",   KEY_RIGHT_ARROW },
  { "Home",          KEY_HOME        },
  { "End",           KEY_END         },
  { "Page Up",       KEY_PAGE_UP     },
  { "Page Down",     KEY_PAGE_DOWN   },
  { "Delete",        KEY_DELETE      },
  { "Insert",        KEY_INSERT      },
  { "Caps Lock",     KEY_CAPS_LOCK   },
  { "F1",  KEY_F1  },{ "F2",  KEY_F2  },{ "F3",  KEY_F3  },{ "F4",  KEY_F4  },
  { "F5",  KEY_F5  },{ "F6",  KEY_F6  },{ "F7",  KEY_F7  },{ "F8",  KEY_F8  },
  { "F9",  KEY_F9  },{ "F10", KEY_F10 },{ "F11", KEY_F11 },{ "F12", KEY_F12 },
};
const uint8_t NUM_SPECIAL_KEYS = sizeof(specialKeyTable)/sizeof(specialKeyTable[0]);

enum {
  SK_ENTER=0, SK_ESC, SK_BACKSPACE, SK_TAB, SK_SPACE,
  SK_UP, SK_DOWN, SK_LEFT, SK_RIGHT,
  SK_HOME, SK_END, SK_PGUP, SK_PGDN, SK_DEL, SK_INS, SK_CAPS,
  SK_F1,SK_F2,SK_F3,SK_F4,SK_F5,SK_F6,SK_F7,SK_F8,SK_F9,SK_F10,SK_F11,SK_F12
};

// Consumer Control codes — little-endian {lo, hi}
const uint8_t KEY_AC_BACK[2] = {0x24,0x02};   // HID CC 0x0224
const uint8_t KEY_AC_HOME[2] = {0x23,0x02};   // HID CC 0x0223

// =============================================================================
//                    MEDIA KEY TABLE (EXPANDED for Google TV / Android / PC)
// =============================================================================
const NamedMediaKey mediaKeyTable[] = {
  // --- Media Playback ---
  { "Play/Pause",         {0xCD,0x00} },   // 0x00CD
  { "Stop",               {0xB7,0x00} },   // 0x00B7
  { "Next Track",         {0xB5,0x00} },   // 0x00B5
  { "Prev Track",         {0xB6,0x00} },   // 0x00B6
  { "Fast Forward",       {0xB3,0x00} },   // 0x00B3
  { "Rewind",             {0xB4,0x00} },   // 0x00B4
  { "Record",             {0xB2,0x00} },   // 0x00B2
  { "Eject",              {0xB8,0x00} },   // 0x00B8
  // --- Volume ---
  { "Volume Up",          {0xE9,0x00} },   // 0x00E9
  { "Volume Down",        {0xEA,0x00} },   // 0x00EA
  { "Mute",               {0xE2,0x00} },   // 0x00E2
  // --- Android / Google TV ---
  { "Back (Android/TV)",  {0x24,0x02} },   // 0x0224 AC Back
  { "Home (Android/TV)",  {0x23,0x02} },   // 0x0223 AC Home
  { "Channel Up",         {0x9C,0x00} },   // 0x009C
  { "Channel Down",       {0x9D,0x00} },   // 0x009D
  { "Menu (TV)",          {0x40,0x00} },   // 0x0040
  { "Select/OK (TV)",     {0x41,0x00} },   // 0x0041
  { "Subtitles / CC",     {0x61,0x00} },   // 0x0061
  { "Brightness Up",      {0x6F,0x00} },   // 0x006F
  { "Brightness Down",    {0x70,0x00} },   // 0x0070
  { "Media Select",       {0x83,0x01} },   // 0x0183
  // --- Browser / System ---
  { "AC Search",          {0x21,0x02} },   // 0x0221
  { "AC Forward",         {0x25,0x02} },   // 0x0225
  { "AC Stop (Browser)",  {0x26,0x02} },   // 0x0226
  { "AC Refresh",         {0x27,0x02} },   // 0x0227
  { "AC Bookmarks",       {0x2A,0x02} },   // 0x022A
};
const uint8_t NUM_MEDIA_KEYS = sizeof(mediaKeyTable)/sizeof(mediaKeyTable[0]);

enum {
  MK_PLAYPAUSE=0, MK_STOP, MK_NEXT, MK_PREV,
  MK_FFWD, MK_RWD, MK_RECORD, MK_EJECT,
  MK_VOLUP, MK_VOLDOWN, MK_MUTE,
  MK_AC_BACK, MK_AC_HOME,
  MK_CHANUP, MK_CHANDOWN,
  MK_MENU, MK_SELECT_OK,
  MK_SUBTITLES, MK_BRIGHT_UP, MK_BRIGHT_DOWN,
  MK_MEDIASEL,
  MK_AC_SEARCH, MK_AC_FORWARD, MK_AC_STOP, MK_AC_REFRESH, MK_AC_BOOKMARKS
};

// =============================================================================
//                    SWIPE GESTURE DETECTION
// =============================================================================
// Horizontal group  : indices 12,13,14,15  (pads 13-16, bottom row)
// Vertical group    : indices  0, 4, 8,12  (pads 1,5,9,13, left column)
// Pad 13 (index 12) is the shared corner — member of both groups.

const uint8_t H_SWIPE_PADS[] = {12,13,14,15};
const uint8_t V_SWIPE_PADS[] = {0,4,8,12};
#define H_SWIPE_LEN 4
#define V_SWIPE_LEN 4

struct SwipeDetector {
  uint8_t seq[8];
  uint8_t seqLen;
  unsigned long lastMs;
  bool fired;
};

SwipeDetector hSwipe = {{},0,0,false};
SwipeDetector vSwipe = {{},0,0,false};
uint16_t lastRawForSwipe = 0;

bool isPadInGroup(uint8_t idx, const uint8_t* g, uint8_t len) {
  for (uint8_t i=0;i<len;i++) if (g[i]==idx) return true;
  return false;
}
int8_t groupPos(uint8_t idx, const uint8_t* g, uint8_t len) {
  for (uint8_t i=0;i<len;i++) if (g[i]==idx) return (int8_t)i;
  return -1;
}
bool isSwipePad(uint8_t idx) {
  return isPadInGroup(idx,H_SWIPE_PADS,H_SWIPE_LEN) ||
         isPadInGroup(idx,V_SWIPE_PADS,V_SWIPE_LEN);
}

// Returns +1 ascending (forward swipe), -1 descending (backward), 0 unclear
int8_t detectSwipeDir(const uint8_t* seq, uint8_t seqLen,
                      const uint8_t* group, uint8_t groupLen) {
  if (seqLen<2) return 0;
  int8_t pos[8]; uint8_t posLen=0;
  for (uint8_t i=0;i<seqLen&&posLen<8;i++) {
    int8_t p=groupPos(seq[i],group,groupLen);
    if (p>=0) pos[posLen++]=p;
  }
  if (posLen<2) return 0;
  bool asc=true, desc=true;
  for (uint8_t i=1;i<posLen;i++) {
    if (pos[i]<=pos[i-1]) asc=false;
    if (pos[i]>=pos[i-1]) desc=false;
  }
  if (asc)  return  1;
  if (desc) return -1;
  return 0;
}

// =============================================================================
//                    ENCODER — HARDWARE INTERRUPT
// =============================================================================
volatile int32_t encoderAccum=0;
volatile unsigned long lastEdgeMicros=0;
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoderISR() {
  static uint8_t old_AB=3;
  static const int8_t enc_states[]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  unsigned long now=micros();
  if ((now-lastEdgeMicros)<encoderDebounceUsCache) return;

  uint8_t A=digitalRead(ENCODER_PIN_A);
  uint8_t B=digitalRead(ENCODER_PIN_B);
  uint8_t new_AB=((old_AB<<2)|((A<<1)|B))&0x0F;
  int8_t ch=enc_states[new_AB];
  if (ch!=0) {
    lastEdgeMicros=now;
    old_AB=new_AB&0x03;
    portENTER_CRITICAL_ISR(&mux);
    encoderAccum+=ch;
    portEXIT_CRITICAL_ISR(&mux);
  }
}

// =============================================================================
//                    HAPTIC ENGINE
// =============================================================================
class HapticEngine {
  int motorPin=0; unsigned long effectStart=0; int currentEffect=0;
  uint8_t maxDuty=255; bool enabled=true; const int pwmChannel=0;
  static const unsigned long KICKSTART_MS=15;

  void writeMotor(int d) {
    if (!enabled) d=0;
    if (d>(int)maxDuty) d=maxDuty; if (d<0) d=0;
#if ESP_IDF_VERSION>=ESP_IDF_VERSION_VAL(5,0,0)
    ledcWrite(motorPin,d);
#else
    ledcWrite(pwmChannel,d);
#endif
  }
  void writeMotorRaw(int d) {
    if (!enabled) d=0; if (d>255) d=255; if (d<0) d=0;
#if ESP_IDF_VERSION>=ESP_IDF_VERSION_VAL(5,0,0)
    ledcWrite(motorPin,d);
#else
    ledcWrite(pwmChannel,d);
#endif
  }
  void driveOn(int d,unsigned long el) {
    if (el<KICKSTART_MS) writeMotorRaw(255); else writeMotor(d);
  }

public:
  void begin(int pin) {
    motorPin=pin; pinMode(motorPin,OUTPUT); digitalWrite(motorPin,LOW);
#if ESP_IDF_VERSION>=ESP_IDF_VERSION_VAL(5,0,0)
    bool ok=ledcAttach(motorPin,12000,8);
    Serial.printf("[HAPTIC] ledcAttach(pin=%d)->%s\n",motorPin,ok?"OK":"FAILED");
    ledcWrite(motorPin,0);
#else
    ledcSetup(pwmChannel,12000,8); ledcAttachPin(motorPin,pwmChannel); ledcWrite(pwmChannel,0);
#endif
  }
  void setStrength(uint8_t s) { maxDuty=s; }
  void setEnabled(bool en) {
    if (enabled!=en) Serial.printf("[HAPTIC] %s\n",en?"ENABLED":"DISABLED");
    enabled=en; if (!enabled) writeMotor(0);
  }
  bool isEnabled() { return enabled; }
  void play(int e) { if (!enabled) return; currentEffect=e; effectStart=millis(); }
  void process() {
    if (!currentEffect) return;
    unsigned long el=millis()-effectStart;
    if (currentEffect==1){
      if (el<35) driveOn(255,el);
      else if (el<90){ float inv=1.0f-((float)(el-35)/55.0f); writeMotor((int)(255.0f*inv*inv)); }
      else { writeMotor(0); currentEffect=0; }
    } else if (currentEffect==2){
      if (el<150) driveOn(255,el); else { writeMotor(0); currentEffect=0; }
    } else if (currentEffect==3){
      if (el<50) driveOn(255,el); else if (el<100) writeMotor(0);
      else if (el<150) writeMotor(255); else { writeMotor(0); currentEffect=0; }
    } else if (currentEffect==4){
      if (el<35) driveOn(200,el); else { writeMotor(0); currentEffect=0; }
    } else if (currentEffect==5){
      if (el<50) driveOn(255,el); else if (el<100) writeMotor(0);
      else if (el<150) writeMotor(255); else if (el<200) writeMotor(0);
      else if (el<250) writeMotor(255); else { writeMotor(0); currentEffect=0; }
    } else if (currentEffect==6){
      if (el<60) writeMotor(255); else if (el<140) writeMotor(0);
      else if (el<200) writeMotor(255); else if (el<280) writeMotor(0);
      else if (el<340) writeMotor(255); else if (el<500) writeMotor(0);
      else if (el<750) writeMotor(255); else { writeMotor(0); currentEffect=0; }
    }
  }
  bool isBusy() { return currentEffect!=0; }
};

HapticEngine haptics;

void pumpHaptics(unsigned long totalMs) {
  unsigned long s=millis();
  while (millis()-s<totalMs){ haptics.process(); if (!haptics.isBusy()) break; delay(5); }
  haptics.process();
}

// =============================================================================
//                    DIP SWITCH DEBOUNCING
// =============================================================================
bool readLogical(const DebouncedPin& d) { return d.activeLow?!digitalRead(d.pin):digitalRead(d.pin); }
bool readRaw(const DebouncedPin& d)     { return digitalRead(d.pin); }

void initDebouncedPin(DebouncedPin& d, uint8_t pin, bool activeLow) {
  d.pin=pin; d.activeLow=activeLow; pinMode(pin,INPUT_PULLUP);
  d.stableState=readLogical(d); d.lastRead=d.stableState; d.lastChangeMs=millis();
}
bool pollDebouncedPin(DebouncedPin& d, unsigned long dMs=50) {
  bool l=readLogical(d);
  if (l!=d.lastRead){ d.lastRead=l; d.lastChangeMs=millis(); return false; }
  if (l!=d.stableState&&(millis()-d.lastChangeMs)>dMs){ d.stableState=l; return true; }
  return false;
}

DebouncedPin dipMode, dipWebserver, dipVibe;

void printDipDiagnostics(const char* r) {
  Serial.printf("\n[DIP DIAG] (%s)\n",r);
  Serial.printf("  DIP1 raw=%d logical=%d -> (BLE always on)\n",readRaw(dipMode),readLogical(dipMode));
  Serial.printf("  DIP2 raw=%d logical=%d -> %s\n",readRaw(dipWebserver),readLogical(dipWebserver),readLogical(dipWebserver)?"force WebUI":"normal");
  Serial.printf("  DIP3 raw=%d logical=%d -> %s\n",readRaw(dipVibe),readLogical(dipVibe),readLogical(dipVibe)?"haptics ON":"haptics OFF");
}

// =============================================================================
//                              TRANSPORTS
// =============================================================================
BleKeyboard* bleKeyboard=nullptr;
typedef struct { uint32_t logicalState; } struct_message;
struct_message espNowOutData; uint32_t espNowLastBitmap=0;
WebServer server(80); DNSServer dnsServer;
IPAddress apIP(192,168,4,1); const byte DNS_PORT=53;

void releaseWifiEspNowRadio() {
  if (!wifiRadioActive) return;
  Serial.println("[RADIO] Releasing WiFi/ESP-NOW...");
  esp_now_deinit(); esp_wifi_stop(); esp_wifi_deinit(); WiFi.mode(WIFI_OFF);
  wifiRadioActive=false; delay(300);
}
void ensureBtControllerIdle() {
  esp_bt_controller_status_t s=esp_bt_controller_get_status();
  if (s==ESP_BT_CONTROLLER_STATUS_IDLE) return;
  Serial.printf("[BLE] controller status=%d, forcing reset\n",(int)s);
  if (s==ESP_BT_CONTROLLER_STATUS_ENABLED) esp_bt_controller_disable();
  esp_bt_controller_deinit(); delay(50);
}

// =============================================================================
//                            DEFAULT CONFIG
// =============================================================================
void setDefaultConfig() {
  memset(&cfg,0,sizeof(cfg));
  cfg.magic=CONFIG_MAGIC; cfg.version=CONFIG_VERSION;
  cfg.outputMode=MODE_BLE_KEYBOARD;
  strlcpy(cfg.staSsid,STA_SSID,sizeof(cfg.staSsid));
  strlcpy(cfg.staPass,STA_PASS,sizeof(cfg.staPass));
  strlcpy(cfg.apSsid,AP_SSID,sizeof(cfg.apSsid));
  strlcpy(cfg.apPass,AP_PASS,sizeof(cfg.apPass));
  strlcpy(cfg.adminPass,"",sizeof(cfg.adminPass));

  cfg.padDebounceMs=15;
  cfg.encoderDebounceUs=200;   // 200us optimal for mechanical EC11 encoders
  cfg.encoderDetentSteps=2;   // 2 steps per detent
  cfg.hapticStrength=255;

  // TV/Phone pad layout — swipe pads have fallback actions for standalone taps
  cfg.padAction[0]  = {ACT_SPECIAL, SK_UP,       0};  // Pad 1  V-swipe pad, fallback=Up
  cfg.padAction[1]  = {ACT_MEDIA,   MK_AC_BACK,  0};  // Pad 2  Back (also enc combo shift)
  cfg.padAction[2]  = {ACT_SPECIAL, SK_UP,        0};  // Pad 3  Up
  cfg.padAction[3]  = {ACT_SPECIAL, SK_ENTER,     0};  // Pad 4  Enter/OK
  cfg.padAction[4]  = {ACT_MEDIA,   MK_AC_HOME,  0};  // Pad 5  V-swipe pad, fallback=Home
  cfg.padAction[5]  = {ACT_MEDIA,   MK_PREV,     0};  // Pad 6  Prev Track
  cfg.padAction[6]  = {ACT_MEDIA,   MK_PLAYPAUSE,0};  // Pad 7  Play/Pause
  cfg.padAction[7]  = {ACT_MEDIA,   MK_NEXT,     0};  // Pad 8  Next Track
  cfg.padAction[8]  = {ACT_SPECIAL, SK_DOWN,     0};  // Pad 9  V-swipe pad, fallback=Down
  cfg.padAction[9]  = {ACT_MEDIA,   MK_AC_HOME,  0};  // Pad 10 Home
  cfg.padAction[10] = {ACT_MEDIA,   MK_MUTE,     0};  // Pad 11 Mute
  cfg.padAction[11] = {ACT_MEDIA,   MK_VOLDOWN,  0};  // Pad 12 Vol Down
  cfg.padAction[12] = {ACT_SPECIAL, SK_LEFT,     0};  // Pad 13 V-swipe corner + H-swipe + enc shift
  cfg.padAction[13] = {ACT_SPECIAL, SK_RIGHT,    0};  // Pad 14 H-swipe pad, fallback=Right
  cfg.padAction[14] = {ACT_SPECIAL, SK_UP,        0};  // Pad 15 H-swipe pad
  cfg.padAction[15] = {ACT_MEDIA,   MK_VOLUP,    0};  // Pad 16 H-swipe pad, fallback=Vol Up

  for (int i=0;i<MAX_COMBOS;i++) {
    cfg.combos[i].mask=0; cfg.combos[i].action={ACT_NONE,0,0};
  }

  // Encoder combos
  cfg.encoderCombos[0]={255,{ACT_SPECIAL,SK_DOWN, 0},{ACT_SPECIAL,SK_UP,  0}};  // bare: scroll
  cfg.encoderCombos[1]={12, {ACT_SPECIAL,SK_RIGHT,0},{ACT_SPECIAL,SK_LEFT,0}};  // Pad13: L/R nav
  cfg.encoderCombos[2]={1,  {ACT_MEDIA,MK_VOLUP,  0},{ACT_MEDIA,MK_VOLDOWN,0}}; // Pad2:  volume
  cfg.encoderCombos[3]={255,{ACT_NONE,0,0},{ACT_NONE,0,0}};                      // disabled

  for (int i=0;i<MAX_MACROS;i++) cfg.macros[i][0]='\0';

  // PC mode
  cfg.operatingMode=0;
  for (int i=0;i<NUM_PADS;i++) cfg.padActionPC[i]={ACT_NONE,0,0};
  cfg.padActionPC[0]  = {ACT_SPECIAL,SK_UP,    0};
  cfg.padActionPC[1]  = {ACT_SPECIAL,SK_ESC,   0};
  cfg.padActionPC[2]  = {ACT_SPECIAL,SK_UP,    0};
  cfg.padActionPC[3]  = {ACT_SPECIAL,SK_ENTER, 0};
  cfg.padActionPC[4]  = {ACT_SPECIAL,SK_DOWN,  0};
  cfg.padActionPC[5]  = {ACT_ASCII,  'z',      MOD_CTRL};  // Ctrl+Z Undo
  cfg.padActionPC[6]  = {ACT_ASCII,  'y',      MOD_CTRL};  // Ctrl+Y Redo
  cfg.padActionPC[7]  = {ACT_ASCII,  's',      MOD_CTRL};  // Ctrl+S Save
  cfg.padActionPC[8]  = {ACT_ASCII,  'a',      MOD_CTRL};  // Ctrl+A Select All
  cfg.padActionPC[9]  = {ACT_SPECIAL,SK_F5,    0};          // F5 Refresh
  cfg.padActionPC[10] = {ACT_ASCII,  'x',      MOD_CTRL};  // Ctrl+X Cut
  cfg.padActionPC[11] = {ACT_ASCII,  'c',      MOD_CTRL};  // Ctrl+C Copy
  cfg.padActionPC[12] = {ACT_ASCII,  'v',      MOD_CTRL};  // Ctrl+V Paste
  cfg.padActionPC[13] = {ACT_SPECIAL,SK_TAB,   MOD_ALT};   // Alt+Tab
  cfg.padActionPC[14] = {ACT_SPECIAL,SK_LEFT,  MOD_ALT};   // Alt+Left (browser back)
  cfg.padActionPC[15] = {ACT_SPECIAL,SK_F4,    MOD_ALT};   // Alt+F4 Close

  // Swipe defaults
  cfg.swipeTimeWindowMs=600;
  cfg.swipeRight={ACT_SPECIAL,SK_RIGHT,0};
  cfg.swipeLeft ={ACT_SPECIAL,SK_LEFT, 0};
  cfg.swipeDown ={ACT_SPECIAL,SK_DOWN, 0};
  cfg.swipeUp   ={ACT_SPECIAL,SK_UP,   0};
}

void applyRuntimeTunables() {
  encoderDebounceUsCache=cfg.encoderDebounceUs;
  haptics.setStrength(cfg.hapticStrength);
}

bool loadConfig() {
  preferences.begin("keypad",true);
  size_t len=preferences.getBytesLength("cfgblob");
  bool ok=false;
  if (len==sizeof(DeviceConfig)) {
    DeviceConfig tmp;
    preferences.getBytes("cfgblob",&tmp,sizeof(tmp));
    if (tmp.magic==CONFIG_MAGIC&&tmp.version==CONFIG_VERSION) { cfg=tmp; ok=true; }
  }
  preferences.end();
  if (!ok) setDefaultConfig();
  applyRuntimeTunables();
  return ok;
}

void saveConfig() {
  preferences.begin("keypad",false);
  preferences.putBytes("cfgblob",&cfg,sizeof(cfg));
  preferences.end();
  applyRuntimeTunables();
  Serial.println("[NVS] Saved.");
}

void factoryReset() {
  preferences.begin("keypad",false); preferences.clear(); preferences.end();
  setDefaultConfig(); saveConfig();
  haptics.play(5); pumpHaptics(300); delay(200); ESP.restart();
}

uint8_t clampU8(long v, uint8_t lo, uint8_t hi) {
  if (v<lo) return lo; if (v>hi) return hi; return (uint8_t)v;
}

void sanitizeKeyAction(KeyAction& a) {
  if (a.type>ACT_MACRO) a.type=ACT_NONE;
  switch (a.type) {
    case ACT_SPECIAL: if (a.code>=NUM_SPECIAL_KEYS) a.code=0; break;
    case ACT_MEDIA:   if (a.code>=NUM_MEDIA_KEYS)   a.code=0; break;
    case ACT_MACRO:   if (a.code>=MAX_MACROS)        a.code=0; break;
    default: break;
  }
  a.modifiers&=(MOD_CTRL|MOD_SHIFT|MOD_ALT|MOD_GUI);
}

// =============================================================================
//                    TTP229 KEYPAD SCAN (PRODUCTION GRADE)
// =============================================================================
uint16_t readTTP229Raw() {
  uint16_t v=0;
  for (int i=0;i<16;i++) {
    digitalWrite(SCL_PIN,LOW); delayMicroseconds(15);
    if (digitalRead(SDO_PIN)==LOW) v|=(1<<i);
    digitalWrite(SCL_PIN,HIGH); delayMicroseconds(15);
  }
  return v;
}

uint16_t updateKeypadState() {
  static uint16_t lastRaw=0, validated=0;
  static unsigned long lastChangeMs=0, lastScanMs=0;
  unsigned long now=millis();
  // TTP229 requires SCL HIGH for >= 2.5ms (Tout) between frames to reset register
  if (now-lastScanMs<8) return validated;
  lastScanMs=now;

  uint16_t raw=readTTP229Raw();
  if (raw!=lastRaw) {
    lastRaw=raw;
    lastChangeMs=now;
  } else if ((now-lastChangeMs)>=cfg.padDebounceMs) {
    if (validated!=raw) {
      validated=raw;
      if (validated) Serial.printf("[TTP229] 0x%04X\n",validated);
    }
  }
  return validated;
}

// =============================================================================
//                    BLE KEYBOARD HELPERS
// =============================================================================
void applyModifiers(uint8_t mods, bool down) {
  if (down) {
    if (mods&MOD_CTRL)  bleKeyboard->press(KEY_LEFT_CTRL);
    if (mods&MOD_SHIFT) bleKeyboard->press(KEY_LEFT_SHIFT);
    if (mods&MOD_ALT)   bleKeyboard->press(KEY_LEFT_ALT);
    if (mods&MOD_GUI)   bleKeyboard->press(KEY_LEFT_GUI);
  } else {
    if (mods&MOD_CTRL)  bleKeyboard->release(KEY_LEFT_CTRL);
    if (mods&MOD_SHIFT) bleKeyboard->release(KEY_LEFT_SHIFT);
    if (mods&MOD_ALT)   bleKeyboard->release(KEY_LEFT_ALT);
    if (mods&MOD_GUI)   bleKeyboard->release(KEY_LEFT_GUI);
  }
}

void bleHoldablePress(const KeyAction& a) {
  applyModifiers(a.modifiers,true);
  if (a.type==ACT_ASCII)    bleKeyboard->press((uint8_t)a.code);
  else if (a.type==ACT_SPECIAL) bleKeyboard->press(specialKeyTable[a.code].code);
}
void bleHoldableRelease(const KeyAction& a) {
  if (a.type==ACT_ASCII)    bleKeyboard->release((uint8_t)a.code);
  else if (a.type==ACT_SPECIAL) bleKeyboard->release(specialKeyTable[a.code].code);
  applyModifiers(a.modifiers,false);
}
void bleTap(const KeyAction& a) {
  if (a.type==ACT_MEDIA) {
    MediaKeyReport rep={mediaKeyTable[a.code].code[0],mediaKeyTable[a.code].code[1]};
    bleKeyboard->write(rep);
  } else if (a.type==ACT_MACRO) {
    bleKeyboard->print(cfg.macros[a.code]);
  }
}

void dispatchAction(const KeyAction& a, bool down, bool clickHaptic) {
  if (a.type==ACT_NONE) return;
  bool conn=bleKeyboard&&bleKeyboard->isConnected();
  if (conn) {
    if (a.type==ACT_ASCII||a.type==ACT_SPECIAL) {
      if (down) bleHoldablePress(a); else bleHoldableRelease(a);
      if (down&&a.type==ACT_SPECIAL&&a.code==SK_ESC) {
        MediaKeyReport acb={KEY_AC_BACK[0],KEY_AC_BACK[1]};
        bleKeyboard->write(acb);
      }
    } else if ((a.type==ACT_MEDIA||a.type==ACT_MACRO)&&down) {
      bleTap(a);
    }
  }
  if (down&&clickHaptic) haptics.play(1);
}

// Single-shot press+release — used for encoder ticks and swipe dispatches
void dispatchTap(const KeyAction& a) {
  if (a.type==ACT_NONE) return;
  bool conn=bleKeyboard&&bleKeyboard->isConnected();
  if (conn) {
    if (a.type==ACT_ASCII) {
      bleKeyboard->press((uint8_t)a.code);
      delay(12);
      bleKeyboard->release((uint8_t)a.code);
    } else if (a.type==ACT_SPECIAL) {
      uint8_t k=specialKeyTable[a.code].code;
      bleKeyboard->press(k);
      delay(12);
      bleKeyboard->release(k);
      if (a.code==SK_ESC) {
        MediaKeyReport acb={KEY_AC_BACK[0],KEY_AC_BACK[1]};
        bleKeyboard->write(acb);
      }
    } else if (a.type==ACT_MEDIA||a.type==ACT_MACRO) {
      bleTap(a);
    }
  }
  haptics.play(4);
}

void dispatchEncoderTapEspNow(bool cw) {
  uint32_t bit=1UL<<(cw?ESPNOW_BIT_ENCODER_CW:ESPNOW_BIT_ENCODER_CCW);
  espNowOutData.logicalState=espNowLastBitmap|bit;
  esp_now_send(maTouchMac,(uint8_t*)&espNowOutData,sizeof(espNowOutData));
  delay(15);
  espNowOutData.logicalState=espNowLastBitmap;
  esp_now_send(maTouchMac,(uint8_t*)&espNowOutData,sizeof(espNowOutData));
  haptics.play(4);
}

// =============================================================================
//                    ENCODER COMBO PROCESSING
// =============================================================================
bool encShiftTurnedWhileHeld[MAX_ENC_COMBOS]={false};
bool encShiftWasHeld[MAX_ENC_COMBOS]={false};
bool slotPrev[NUM_SLOTS]={false};

void processEncoderDetents(uint16_t rawState) {
  portENTER_CRITICAL(&mux);
  int32_t accum=encoderAccum; encoderAccum=0;
  portEXIT_CRITICAL(&mux);

  static int32_t detentCounter=0;
  static unsigned long lastActivity=0;

  if (accum) {
    if ((accum>0&&detentCounter<0)||(accum<0&&detentCounter>0)) detentCounter=0;
    detentCounter+=accum; lastActivity=millis();
  }
  if (detentCounter&&(millis()-lastActivity)>200) detentCounter=0;

  int steps=cfg.encoderDetentSteps>0?cfg.encoderDetentSteps:2;
  bool cwFired=false, ccwFired=false;
  if      (detentCounter>= steps) { detentCounter-=steps; cwFired=true;  }
  else if (detentCounter<=-steps) { detentCounter+=steps; ccwFired=true; }
  if (!cwFired&&!ccwFired) return;

  // Find active combo — first matching shift pad wins, else use no-shift (255) combo
  int activeCombo=-1;
  for (int c=0;c<MAX_ENC_COMBOS;c++) {
    uint8_t sp=cfg.encoderCombos[c].shiftPadIndex;
    if (sp<NUM_PADS&&(rawState&(1<<sp))) {
      activeCombo=c;
      encShiftTurnedWhileHeld[c]=true;
      break;
    }
  }
  if (activeCombo<0) {
    for (int c=0;c<MAX_ENC_COMBOS;c++) {
      if (cfg.encoderCombos[c].shiftPadIndex==255) { activeCombo=c; break; }
    }
  }
  if (activeCombo<0) return;

  const EncoderCombo& ec=cfg.encoderCombos[activeCombo];
  if (cfg.outputMode==MODE_ESPNOW) dispatchEncoderTapEspNow(cwFired);
  else dispatchTap(cwFired?ec.cw:ec.ccw);
}

// =============================================================================
//                    MAIN OUTPUT DISPATCH & SMART GESTURE ENGINE
// =============================================================================
const KeyAction& actionForSlot(int slot) {
  if (slot<NUM_PADS) return (cfg.operatingMode==1)?cfg.padActionPC[slot]:cfg.padAction[slot];
  return cfg.combos[slot-NUM_PADS].action;
}

uint16_t buildEncShiftMask() {
  uint16_t m=0;
  for (int c=0;c<MAX_ENC_COMBOS;c++)
    if (cfg.encoderCombos[c].shiftPadIndex<NUM_PADS)
      m|=(1<<cfg.encoderCombos[c].shiftPadIndex);
  return m;
}

// Swipe state machine
struct SwipeState {
  uint8_t seq[8];
  uint8_t seqLen;
  unsigned long startMs;
  bool fired;
};
SwipeState swipeSession = {{}, 0, 0, false};

void updateOutputsAndTransmit(uint16_t rawState) {
  uint16_t consumedMask=0;
  bool comboActive[MAX_COMBOS];
  for (int c=0;c<MAX_COMBOS;c++) {
    if (cfg.combos[c].mask&&(rawState&cfg.combos[c].mask)==cfg.combos[c].mask) {
      comboActive[c]=true; consumedMask|=cfg.combos[c].mask;
    } else comboActive[c]=false;
  }

  uint16_t encShiftMask=buildEncShiftMask();
  bool slotNow[NUM_SLOTS]={};
  unsigned long now=millis();

  // Check swipe pads currently pressed
  bool anySwipeHeld = false;
  for (int i=0; i<NUM_PADS; i++) {
    if (isSwipePad(i) && ((rawState >> i) & 1)) {
      anySwipeHeld = true;
      break;
    }
  }

  // --- SWIPE GESTURE & TAP ENGINE ---
  if (anySwipeHeld) {
    for (int i=0; i<NUM_PADS; i++) {
      bool isHeld = (rawState >> i) & 1;
      bool wasHeld = (lastRawForSwipe >> i) & 1;
      if (isHeld && !wasHeld && isSwipePad(i)) {
        if (swipeSession.seqLen == 0 || (now - swipeSession.startMs > cfg.swipeTimeWindowMs)) {
          swipeSession.seq[0] = i;
          swipeSession.seqLen = 1;
          swipeSession.startMs = now;
          swipeSession.fired = false;
        } else if (swipeSession.seqLen < 8 && swipeSession.seq[swipeSession.seqLen - 1] != i) {
          swipeSession.seq[swipeSession.seqLen++] = i;
        }

        // Detect horizontal swipe
        if (!swipeSession.fired && swipeSession.seqLen >= 2) {
          int8_t hd = detectSwipeDir(swipeSession.seq, swipeSession.seqLen, H_SWIPE_PADS, H_SWIPE_LEN);
          if (hd == 1) {
            dispatchTap(cfg.swipeRight);
            swipeSession.fired = true;
            Serial.println("[SWIPE] Right");
          } else if (hd == -1) {
            dispatchTap(cfg.swipeLeft);
            swipeSession.fired = true;
            Serial.println("[SWIPE] Left");
          }
        }
        // Detect vertical swipe
        if (!swipeSession.fired && swipeSession.seqLen >= 2) {
          int8_t vd = detectSwipeDir(swipeSession.seq, swipeSession.seqLen, V_SWIPE_PADS, V_SWIPE_LEN);
          if (vd == 1) {
            dispatchTap(cfg.swipeDown);
            swipeSession.fired = true;
            Serial.println("[SWIPE] Down");
          } else if (vd == -1) {
            dispatchTap(cfg.swipeUp);
            swipeSession.fired = true;
            Serial.println("[SWIPE] Up");
          }
        }
      }
    }
  } else {
    // All swipe pads released — fire standalone tap if no swipe occurred!
    if (swipeSession.seqLen > 0) {
      if (!swipeSession.fired && (now - swipeSession.startMs <= cfg.swipeTimeWindowMs)) {
        uint8_t tappedPad = swipeSession.seq[0];
        bool wasShiftTurned = false;
        for (int c=0; c<MAX_ENC_COMBOS; c++) {
          if (cfg.encoderCombos[c].shiftPadIndex == tappedPad && encShiftTurnedWhileHeld[c]) {
            wasShiftTurned = true; break;
          }
        }
        if (!wasShiftTurned) {
          const KeyAction& a = actionForSlot(tappedPad);
          if (a.type != ACT_NONE) {
            dispatchTap(a);
            Serial.printf("[SWIPE_PAD] Pad%d tapped -> own action fired\n", tappedPad + 1);
          }
        }
      }
      swipeSession.seqLen = 0;
      swipeSession.fired = false;
    }
  }

  // --- STANDARD & SHIFT PADS ---
  for (int i=0;i<NUM_PADS;i++) {
    bool held=(rawState>>i)&1;

    // Encoder shift pads (that are not swipe pads)
    if (cfg.outputMode==MODE_BLE_KEYBOARD&&(encShiftMask&(1<<i))) {
      int ci=-1;
      for (int c=0;c<MAX_ENC_COMBOS;c++)
        if (cfg.encoderCombos[c].shiftPadIndex==(uint8_t)i) { ci=c; break; }
      if (ci>=0) {
        bool wasHeld=encShiftWasHeld[ci];
        if (held&&!wasHeld) {
          encShiftTurnedWhileHeld[ci]=false; haptics.play(1);
        } else if (!held&&wasHeld) {
          if (!encShiftTurnedWhileHeld[ci] && !isSwipePad(i)) {
            const KeyAction& a=actionForSlot(i);
            if (a.type!=ACT_NONE) {
              dispatchTap(a);
              Serial.printf("[ENC_SHIFT] Pad%d tapped -> own action\n",i+1);
            }
          }
          encShiftTurnedWhileHeld[ci]=false;
        }
        encShiftWasHeld[ci]=held;
      }
      slotNow[i]=false; continue;
    }

    // Swipe pads are handled by swipe / tap engine above
    if (isSwipePad(i)) {
      slotNow[i]=false; continue;
    }

    slotNow[i]=held&&!(consumedMask&(1<<i));
  }

  lastRawForSwipe=rawState;
  for (int c=0;c<MAX_COMBOS;c++) slotNow[NUM_PADS+c]=comboActive[c];

  if (cfg.outputMode==MODE_ESPNOW) {
    uint32_t bm=0;
    for (int s=0;s<NUM_SLOTS;s++) if (slotNow[s]) bm|=(1UL<<s);
    if (bm!=espNowLastBitmap) {
      espNowOutData.logicalState=bm;
      esp_now_send(maTouchMac,(uint8_t*)&espNowOutData,sizeof(espNowOutData));
      espNowLastBitmap=bm;
      for (int s=0;s<NUM_SLOTS;s++) if (slotNow[s]!=slotPrev[s]&&slotNow[s]) haptics.play(1);
    }
  } else {
    for (int s=0;s<NUM_SLOTS;s++)
      if (slotNow[s]!=slotPrev[s]) dispatchAction(actionForSlot(s),slotNow[s],true);
  }
  for (int s=0;s<NUM_SLOTS;s++) slotPrev[s]=slotNow[s];
}

// =============================================================================
//                    WEB SERVER AUTH
// =============================================================================
bool isAuthorized() {
  if (!strlen(cfg.adminPass)) return true;
  if (server.hasArg("pass")&&server.arg("pass")==cfg.adminPass) return true;
  return false;
}

// =============================================================================
//                    WEB UI — HTML / CSS / JS
// =============================================================================
#include "webpage.h"

// =============================================================================
//                    JSON SERIALIZATION
// =============================================================================
void configToJson(DynamicJsonDocument& doc) {
  doc["padDebounceMs"]     =cfg.padDebounceMs;
  doc["encoderDebounceUs"] =cfg.encoderDebounceUs;
  doc["encoderDetentSteps"]=cfg.encoderDetentSteps;
  doc["hapticStrength"]    =cfg.hapticStrength;
  doc["staSsid"]           =cfg.staSsid;
  doc["apSsid"]            =cfg.apSsid;
  doc["operatingMode"]     =cfg.operatingMode;
  doc["swipeTimeWindowMs"] =cfg.swipeTimeWindowMs;

  JsonArray sp=doc.createNestedArray("specialKeys");
  for (int i=0;i<NUM_SPECIAL_KEYS;i++) sp.add(specialKeyTable[i].name);
  JsonArray mk=doc.createNestedArray("mediaKeys");
  for (int i=0;i<NUM_MEDIA_KEYS;i++) mk.add(mediaKeyTable[i].name);

  auto addA=[&](JsonObject o,const KeyAction& a){
    o["type"]=a.type; o["code"]=a.code; o["modifiers"]=a.modifiers;
  };

  JsonArray pads=doc.createNestedArray("pads");
  for (int i=0;i<NUM_PADS;i++) addA(pads.createNestedObject(),cfg.padAction[i]);

  JsonArray padsPC=doc.createNestedArray("padsPC");
  for (int i=0;i<NUM_PADS;i++) addA(padsPC.createNestedObject(),cfg.padActionPC[i]);

  JsonArray combos=doc.createNestedArray("combos");
  for (int i=0;i<MAX_COMBOS;i++) {
    JsonObject o=combos.createNestedObject();
    o["mask"]=cfg.combos[i].mask;
    addA(o.createNestedObject("action"),cfg.combos[i].action);
  }

  JsonArray ecs=doc.createNestedArray("encoderCombos");
  for (int i=0;i<MAX_ENC_COMBOS;i++) {
    JsonObject o=ecs.createNestedObject();
    o["shiftPadIndex"]=cfg.encoderCombos[i].shiftPadIndex;
    addA(o.createNestedObject("cw"), cfg.encoderCombos[i].cw);
    addA(o.createNestedObject("ccw"),cfg.encoderCombos[i].ccw);
  }

  JsonArray macros=doc.createNestedArray("macros");
  for (int i=0;i<MAX_MACROS;i++) macros.add(cfg.macros[i]);

  auto addNamed=[&](const char* key,const KeyAction& a){
    JsonObject o=doc.createNestedObject(key);
    o["type"]=a.type; o["code"]=a.code; o["modifiers"]=a.modifiers;
  };
  addNamed("swipeRight",cfg.swipeRight); addNamed("swipeLeft",cfg.swipeLeft);
  addNamed("swipeDown", cfg.swipeDown);  addNamed("swipeUp",  cfg.swipeUp);
}

KeyAction parseAction(JsonObjectConst o) {
  KeyAction a;
  a.type     =clampU8(o["type"]     |0,0,ACT_MACRO);
  a.code     =clampU8(o["code"]     |0,0,255);
  a.modifiers=clampU8(o["modifiers"]|0,0,15);
  sanitizeKeyAction(a);
  return a;
}

bool jsonToConfig(DynamicJsonDocument& doc, String& err) {
  DeviceConfig nc=cfg;
  nc.padDebounceMs     =clampU8(doc["padDebounceMs"]|15,1,100);
  nc.encoderDebounceUs =(uint16_t)constrain((int)(doc["encoderDebounceUs"]|1200),0,10000);
  nc.encoderDetentSteps=clampU8(doc["encoderDetentSteps"]|2,1,16);
  nc.hapticStrength    =clampU8(doc["hapticStrength"]|255,0,255);
  nc.operatingMode     =clampU8(doc["operatingMode"]|0,0,1);
  nc.swipeTimeWindowMs =(uint16_t)constrain((int)(doc["swipeTimeWindowMs"]|600),50,5000);

  strlcpy(nc.staSsid,doc["staSsid"]|"",sizeof(nc.staSsid));
  if (doc.containsKey("staPass"))   strlcpy(nc.staPass,  doc["staPass"]  |"",sizeof(nc.staPass));
  if (doc.containsKey("apPass"))    strlcpy(nc.apPass,   doc["apPass"]   |"",sizeof(nc.apPass));
  if (doc.containsKey("adminPass")) strlcpy(nc.adminPass,doc["adminPass"]|"",sizeof(nc.adminPass));

  if (!doc["pads"].is<JsonArray>()||doc["pads"].size()!=(unsigned)NUM_PADS)
    { err="pads must have 16 entries"; return false; }
  for (int i=0;i<NUM_PADS;i++) nc.padAction[i]=parseAction(doc["pads"][i]);

  if (doc["padsPC"].is<JsonArray>()&&doc["padsPC"].size()==(unsigned)NUM_PADS)
    for (int i=0;i<NUM_PADS;i++) nc.padActionPC[i]=parseAction(doc["padsPC"][i]);

  if (!doc["combos"].is<JsonArray>()||doc["combos"].size()!=(unsigned)MAX_COMBOS)
    { err="combos must have 6 entries"; return false; }
  for (int i=0;i<MAX_COMBOS;i++) {
    JsonObjectConst co=doc["combos"][i];
    nc.combos[i].mask=(uint16_t)constrain((int)(co["mask"]|0),0,0xFFFF);
    nc.combos[i].action=parseAction(co["action"]);
  }

  if (doc["encoderCombos"].is<JsonArray>()&&doc["encoderCombos"].size()==(unsigned)MAX_ENC_COMBOS) {
    for (int i=0;i<MAX_ENC_COMBOS;i++) {
      JsonObjectConst eo=doc["encoderCombos"][i];
      nc.encoderCombos[i].shiftPadIndex=eo.containsKey("shiftPadIndex") ? clampU8(eo["shiftPadIndex"].as<int>(),0,255) : 255;
      nc.encoderCombos[i].cw =parseAction(eo["cw"]);
      nc.encoderCombos[i].ccw=parseAction(eo["ccw"]);
    }
  }

  if (!doc["macros"].is<JsonArray>()||doc["macros"].size()!=(unsigned)MAX_MACROS)
    { err="macros must have 6 entries"; return false; }
  for (int i=0;i<MAX_MACROS;i++) strlcpy(nc.macros[i],doc["macros"][i]|"",MACRO_LEN);

  if (doc["swipeRight"].is<JsonObject>()) nc.swipeRight=parseAction(doc["swipeRight"]);
  if (doc["swipeLeft"].is<JsonObject>())  nc.swipeLeft =parseAction(doc["swipeLeft"]);
  if (doc["swipeDown"].is<JsonObject>())  nc.swipeDown =parseAction(doc["swipeDown"]);
  if (doc["swipeUp"].is<JsonObject>())    nc.swipeUp   =parseAction(doc["swipeUp"]);

  nc.magic=CONFIG_MAGIC; nc.version=CONFIG_VERSION; nc.outputMode=cfg.outputMode;
  cfg=nc; return true;
}

// =============================================================================
//                    HTTP HANDLERS
// =============================================================================
void handleRoot() { server.send_P(200,"text/html",INDEX_HTML); }

void handleApiStatus() {
  DynamicJsonDocument doc(512);
  doc["mode"]           =cfg.outputMode;
  doc["bleConnected"]   =bleActuallyActive&&bleKeyboard&&bleKeyboard->isConnected();
  doc["bleName"]        ="Custom Keypad";
  doc["bleSafeMode"]    =bleSafeModeThisBoot;
  doc["vibeEnabled"]    =haptics.isEnabled();
  doc["firmwareVersion"]=FIRMWARE_VERSION;
  doc["uptimeSec"]      =(uint32_t)(millis()/1000);
  doc["freeHeap"]       =ESP.getFreeHeap();
  doc["otaActive"]      =otaActive;
  if (configMode) {
    doc["apIp"]   =apIP.toString();
    doc["staIp"]  =(WiFi.status()==WL_CONNECTED)?WiFi.localIP().toString():"";
    doc["hostname"]="keypad.local";
  }
  String out; serializeJson(doc,out);
  server.send(200,"application/json",out);
}

void handleApiConfigGet() {
  DynamicJsonDocument doc(16384);
  configToJson(doc);
  String out; serializeJson(doc,out);
  server.send(200,"application/json",out);
}

void handleApiConfigPost() {
  if (!isAuthorized()) { server.send(401,"application/json","{\"ok\":false,\"error\":\"unauthorized\"}"); return; }
  if (!server.hasArg("plain")) { server.send(400,"application/json","{\"ok\":false,\"error\":\"no body\"}"); return; }
  DynamicJsonDocument doc(16384);
  if (deserializeJson(doc,server.arg("plain"))) { server.send(400,"application/json","{\"ok\":false,\"error\":\"bad json\"}"); return; }
  String err;
  if (!jsonToConfig(doc,err)) { server.send(400,"application/json","{\"ok\":false,\"error\":\""+err+"\"}"); return; }
  saveConfig(); haptics.play(2);
  server.send(200,"application/json","{\"ok\":true}");
}

void handleApiReset() {
  if (!isAuthorized()) { server.send(401,"application/json","{\"ok\":false}"); return; }
  server.send(200,"application/json","{\"ok\":true}");
  delay(100); factoryReset();
}
void handleApiReboot() {
  if (!isAuthorized()) { server.send(401,"application/json","{\"ok\":false}"); return; }
  server.send(200,"application/json","{\"ok\":true}");
  haptics.play(3); pumpHaptics(300); delay(200); ESP.restart();
}
void handleApiIdentify() {
  if (!isAuthorized()) { server.send(401,"application/json","{\"ok\":false}"); return; }
  bool was=haptics.isEnabled(); haptics.setEnabled(true); haptics.play(5);
  server.send(200,"application/json","{\"ok\":true}");
  pumpHaptics(300); haptics.setEnabled(was);
}
void handleNotFound() {
  server.sendHeader("Location","http://"+apIP.toString()+"/",true);
  server.send(302,"text/plain","");
}

void handleApiOtaStart() {
  if (!isAuthorized()) { server.send(401,"application/json","{\"ok\":false,\"error\":\"unauthorized\"}"); return; }
  Serial.println("[OTA] Manual OTA Start requested from Web UI...");
  WiFi.mode(WIFI_AP_STA);
  if (strlen(cfg.staSsid)>0) {
    if (WiFi.status()!=WL_CONNECTED) {
      Serial.printf("[WIFI] Connecting to SSID: %s ...\n", cfg.staSsid);
      WiFi.begin(cfg.staSsid, cfg.staPass);
      unsigned long start=millis();
      while (WiFi.status()!=WL_CONNECTED && (millis()-start<7000)) {
        delay(250);
        Serial.print(".");
      }
      Serial.println();
    }
  }
  if (!otaActive) {
    ArduinoOTA.setHostname("keypad");
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    otaActive=true;
  }
  DynamicJsonDocument doc(512);
  doc["ok"]=true;
  doc["otaActive"]=otaActive;
  doc["apIp"]=apIP.toString();
  doc["staIp"]=(WiFi.status()==WL_CONNECTED)?WiFi.localIP().toString():"";
  doc["hostname"]="keypad.local";
  doc["ssid"]=cfg.staSsid;
  String out; serializeJson(doc,out);
  server.send(200,"application/json",out);
}

void sendHaierPower() {
  Serial.println("[IR] Transmitting Haier TV Power (SAMSUNG protocol)...");
  // Old TVs in standby often miss a single frame, so fire several
  // separate bursts with a gap between them rather than relying on
  // one sendSAMSUNG() call's internal repeat count.
  for (uint8_t i = 0; i < 4; i++) {
    irSend.sendSAMSUNG(HAIER_POWER_CODE, 32, 1);
    delay(45);
  }
  Serial.println("[IR] Haier Power transmission finished.");
}

void handleApiIrHaier() {
  if (!isAuthorized()) { server.send(401,"application/json","{\"ok\":false}"); return; }
  sendHaierPower();
  server.send(200,"application/json","{\"ok\":true,\"msg\":\"Haier power sent\"}");
}

// =============================================================================
//                    WEB SERVER START / STOP
// =============================================================================
void startWebServer(bool forcedByDip) {
  if (bleActuallyActive) {
    Serial.println("[SYSTEM] BLE active -- web server refused. Reboot with DIP2 on to enter config mode.");
    return;
  }
  configMode=true; configModeForcedByDip=forcedByDip; configStartTime=millis();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP,apIP,IPAddress(255,255,255,0));
  WiFi.softAP(cfg.apSsid,strlen(cfg.apPass)>=8?cfg.apPass:NULL);
  // Do NOT lock channel: allow STA and AP to coexist on router's channel
  if (strlen(cfg.staSsid)>0) WiFi.begin(cfg.staSsid,cfg.staPass);
  dnsServer.start(DNS_PORT,"*",apIP);
  if (MDNS.begin("keypad")) MDNS.addService("http","tcp",80);
  server.on("/",handleRoot);
  server.on("/api/status",HTTP_GET,handleApiStatus);
  server.on("/api/config",HTTP_GET,handleApiConfigGet);
  server.on("/api/config",HTTP_POST,handleApiConfigPost);
  server.on("/api/reset",HTTP_POST,handleApiReset);
  server.on("/api/reboot",HTTP_POST,handleApiReboot);
  server.on("/api/identify",HTTP_POST,handleApiIdentify);
  server.on("/api/ota/start",HTTP_POST,handleApiOtaStart);
  server.on("/api/ir/haier",HTTP_POST,handleApiIrHaier);
  server.onNotFound(handleNotFound);
  server.begin();

  ArduinoOTA.setHostname("keypad");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([](){
    Serial.println("[OTA] Starting...");
    haptics.setEnabled(false); otaProgressLastHapticPct=255;
  });
  ArduinoOTA.onEnd([](){ Serial.println("\n[OTA] Done, rebooting..."); });
  ArduinoOTA.onProgress([](unsigned int p,unsigned int t){
    unsigned int pct=t?(p*100U)/t:0;
    Serial.printf("[OTA] %u%%\r",pct);
    if (pct/25!=otaProgressLastHapticPct/25) {
      otaProgressLastHapticPct=pct;
      haptics.setEnabled(true); haptics.play(4); pumpHaptics(50); haptics.setEnabled(false);
    }
  });
  ArduinoOTA.onError([](ota_error_t e){
    Serial.printf("[OTA] Error[%u]\n",e);
    haptics.setEnabled(dipVibe.stableState); haptics.play(6);
  });
  ArduinoOTA.begin();
  otaActive=true;
  Serial.println("[OTA] Ready at keypad.local");
  haptics.play(2);
}

void stopWebServer() {
  configMode=false; configModeForcedByDip=false;
  if (otaActive) { ArduinoOTA.end(); otaActive=false; }
  server.stop(); dnsServer.stop(); MDNS.end();
  WiFi.softAPdisconnect(true);
  if (cfg.outputMode==MODE_ESPNOW) {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL,WIFI_SECOND_CHAN_NONE);
  }
  haptics.play(3);
}

// =============================================================================
//                                   SETUP
// =============================================================================
void setup() {
  Serial.begin(115200); delay(200);
  irSend.begin();   // initialise IR TX on GPIO 21
  pinMode(SCL_PIN,OUTPUT); pinMode(SDO_PIN,INPUT_PULLUP); digitalWrite(SCL_PIN,HIGH);
  pinMode(ENCODER_PIN_A,INPUT_PULLUP); pinMode(ENCODER_PIN_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A),encoderISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B),encoderISR,CHANGE);

  initDebouncedPin(dipMode,     DIP_MODE_PIN,     true);
  initDebouncedPin(dipWebserver,DIP_WEBSERVER_PIN,true);
  initDebouncedPin(dipVibe,     DIP_VIBE_PIN,     true);
  printDipDiagnostics("boot");

  haptics.begin(MOTOR_PIN);
  haptics.setEnabled(dipVibe.stableState);

  if (loadConfig()) Serial.println("[NVS] Settings loaded.");
  else              Serial.println("[NVS] First boot / version change. Defaults applied.");

  // BLE crash-loop guard
  const uint8_t BLE_CRASH_THRESHOLD=2;
  preferences.begin("keypad",false);
  uint8_t bleFails=preferences.getUChar("bleFails",0);
  bleSafeModeThisBoot=(bleFails>=BLE_CRASH_THRESHOLD);
  if (bleSafeModeThisBoot) preferences.putUChar("bleFails",0);
  else                     preferences.putUChar("bleFails",bleFails+1);
  preferences.end();

  preferences.begin("keypad",false);
  bool padConfigBoot=preferences.getBool("padConfig",false);
  if (padConfigBoot) preferences.putBool("padConfig",false);
  preferences.end();

  bool skipBle=bleSafeModeThisBoot||dipWebserver.stableState||padConfigBoot;

  if (bleSafeModeThisBoot)       Serial.println("[SAFEMODE] BLE crash-loop detected — skipping BLE, starting web server.");
  else if (dipWebserver.stableState) Serial.println("[CONFIG] DIP2 on — skipping BLE, starting web server + OTA.");
  else if (padConfigBoot)       Serial.println("[CONFIG] Pad hold boot — skipping BLE, starting web server + OTA.");

  if (cfg.outputMode!=MODE_BLE_KEYBOARD) { cfg.outputMode=MODE_BLE_KEYBOARD; saveConfig(); }

  if (skipBle) {
    bleKeyboard=nullptr; bleActuallyActive=false;
    haptics.play(bleSafeModeThisBoot?6:2);
    pumpHaptics(bleSafeModeThisBoot?900:300);
  } else {
    releaseWifiEspNowRadio();
    ensureBtControllerIdle();
    bleKeyboard=new BleKeyboard("Custom Keypad","DIY-HID",100);
    bleKeyboard->begin();
    bleActuallyActive=true;
  }

  if (dipWebserver.stableState||bleSafeModeThisBoot||padConfigBoot) startWebServer(true);

  delay(500);
  if (!skipBle) haptics.play(2);

  Serial.printf("[INIT] v%s ready | mode=%s | swipe=%dms | detents=%d\n",
    FIRMWARE_VERSION,
    cfg.operatingMode==1?"PC":"TV/Phone",
    cfg.swipeTimeWindowMs,
    cfg.encoderDetentSteps);
}

// =============================================================================
//                                   LOOP
// =============================================================================
void loop() {
  haptics.process();

  // Clear BLE crash counter after 20s of stable uptime
  if (!bleCrashCounterCleared&&bleActuallyActive&&millis()>20000UL) {
    preferences.begin("keypad",false);
    preferences.putUChar("bleFails",0);
    preferences.end();
    bleCrashCounterCleared=true;
    Serial.println("[SAFEMODE] BLE stable 20s — crash counter cleared.");
  }

  if (pollDebouncedPin(dipVibe)) haptics.setEnabled(dipVibe.stableState);

  if (pollDebouncedPin(dipWebserver)) {
    if (dipWebserver.stableState&&!configMode) {
      Serial.println("[DIP] Config DIP ON — rebooting into config mode...");
      haptics.play(3); pumpHaptics(300); delay(150); ESP.restart();
    } else if (!dipWebserver.stableState&&configMode&&configModeForcedByDip&&!bleSafeModeThisBoot) {
      Serial.println("[DIP] Config DIP OFF — rebooting into BLE mode...");
      haptics.play(3); pumpHaptics(300); delay(150); ESP.restart();
    }
  }

  if (configMode) {
    dnsServer.processNextRequest();
    server.handleClient();
    if (otaActive) ArduinoOTA.handle();
    if (!configModeForcedByDip&&millis()-configStartTime>CONFIG_TIMEOUT) stopWebServer();
    uint16_t rs=updateKeypadState();
    if ((rs&CONFIG_TRIGGER_MASK)!=CONFIG_TRIGGER_MASK) holdStartConfig=0;
    return;
  }

  uint16_t rawState=updateKeypadState();
  processEncoderDetents(rawState);

  // --- IR: Hold Pad 13 + Pad 16 (0x9000) for 1.2s -> send Haier TV Power ---
  if ((rawState & IR_TRIGGER_MASK) == IR_TRIGGER_MASK) {
    if (!holdStartIR) holdStartIR = millis();
    else if (!irFiredThisHold && (millis() - holdStartIR >= IR_HOLD_MS)) {
      sendHaierPower();
      irFiredThisHold = true;
      haptics.play(2);   // double buzz so user knows it fired
    }
  } else {
    holdStartIR = 0;
    irFiredThisHold = false;
  }

  // --- Config Mode: Hold bottom 4 pads (13-16 = 0xF000) for 3.5s ---
  if ((rawState & CONFIG_TRIGGER_MASK) == CONFIG_TRIGGER_MASK) {
    if (!holdStartConfig) holdStartConfig = millis();
    else if (millis() - holdStartConfig > HOLD_DURATION) {
      Serial.println("[CONFIG] Pad hold 3.5s — rebooting into Config/OTA mode...");
      preferences.begin("keypad", false);
      preferences.putBool("padConfig", true);
      preferences.end();
      haptics.play(3); pumpHaptics(300); delay(150); ESP.restart();
      holdStartConfig = 0;
      return;
    }
  } else holdStartConfig = 0;

  updateOutputsAndTransmit(rawState);
  delay(4);
}

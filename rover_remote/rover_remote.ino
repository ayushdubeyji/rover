#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "rover_proto.h"

// ---------------- IR (unchanged from original build) ----------------
#define IR_TX_PIN 21
#define HAIER_POWER_CODE 0x1818D02FUL
IRsend irSend(IR_TX_PIN);

void sendHaierPower() {
  Serial.println("[IR] Transmitting Haier TV Power (SAMSUNG protocol)...");
  for (uint8_t i = 0; i < 4; i++) {
    irSend.sendSAMSUNG(HAIER_POWER_CODE, 32, 1);
    delay(45);
  }
  Serial.println("[IR] Haier Power transmission finished.");
}

// ---------------- Pins ----------------
#define SCL_PIN 0
#define SDO_PIN 1
#define ENCODER_PIN_A 3
#define ENCODER_PIN_B 4
#define MOTOR_PIN 5
#define DIP_CRAWL_PIN 6
#define DIP_PAIR_PIN 7
#define DIP_VIBE_PIN 10

// ---------------- Key map (TTP229 pad index 0..15, laid out 4x4) ----------------
//  [Q ↺ ] [W ↑ ] [E ↻ ] [SPD+]
//  [A ← ] [S ↓ ] [D → ] [SPD-]
//  [↖   ] [STOP] [↗   ] [MODE]
//  [↙   ] [TURBO][↘   ] [ FN ]
enum {
  K_Q = 0, K_W, K_E, K_SPDUP,
  K_A, K_S, K_D, K_SPDDN,
  K_DFL, K_STOP, K_DFR, K_MODE,
  K_DBL, K_TURBO, K_DBR, K_FN
};
#define BIT(k) (1u << (k))
const uint16_t IR_TRIGGER_MASK = BIT(K_DBL) | BIT(K_FN);
const unsigned long IR_HOLD_MS = 1200;

// ---------------- Haptics ----------------
enum Fx { FX_NONE, FX_TICK, FX_CLICK, FX_DOUBLE, FX_LONG, FX_TRIPLE };
uint8_t fx = FX_NONE;
unsigned long fxStart = 0;
bool vibeOn = true;

void hapt(uint8_t e) { if (vibeOn) { fx = e; fxStart = millis(); } }

void hapticTick() {
  if (!fx) return;
  unsigned long t = millis() - fxStart;
  int d = 0;
  switch (fx) {
    case FX_TICK: d = t < 18 ? 170 : -1; break;
    case FX_CLICK: d = t < 35 ? 255 : -1; break;
    case FX_DOUBLE: d = t < 40 ? 255 : t < 90 ? 0 : t < 130 ? 255 : -1; break;
    case FX_LONG: d = t < 220 ? 255 : -1; break;
    case FX_TRIPLE: d = (t < 250) ? (((t / 50) % 2) ? 0 : 255) : -1; break;
  }
  if (d < 0) { d = 0; fx = FX_NONE; }
  ledcWrite(MOTOR_PIN, vibeOn ? d : 0);
}

// ---------------- Encoder ----------------
volatile int32_t encAccum = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoderISR() {
  static uint8_t ab = 3;
  static const int8_t tbl[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  ab = (ab << 2) | (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);
  int8_t c = tbl[ab & 0x0F];
  if (!c) return;
  static uint32_t lastUs = 0;
  uint32_t nowUs = micros();
  if (nowUs - lastUs < 600) return;
  lastUs = nowUs;
  portENTER_CRITICAL_ISR(&mux);
  encAccum += c;
  portEXIT_CRITICAL_ISR(&mux);
}

// ---------------- TTP229 ----------------
uint16_t readPads() {
  uint16_t v = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(10);
    if (digitalRead(SDO_PIN) == LOW) v |= (1 << i);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
  }
  return v;
}

uint16_t pads() {
  static uint16_t last = 0, stable = 0;
  static unsigned long changed = 0;
  uint16_t r = readPads();
  if (r != last) { last = r; changed = millis(); }
  else if (millis() - changed >= 20) stable = (r == 0xFFFF || __builtin_popcount(r) > 4) ? 0 : r;
  return stable;
}

// ---------------- ESP-NOW link ----------------
Preferences prefs;
uint8_t roverMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t roverCh = 1;
volatile bool paired = false;
volatile unsigned long lastAck = 0;
volatile uint8_t ackMode = 0, ackSpeed = 0;
uint16_t seq = 0;

uint8_t speedPct = 60;
uint8_t modeReq = MODE_NORMAL;

void addPeer(const uint8_t* mac, uint8_t ch) {
  if (esp_now_is_peer_exist(mac)) esp_now_del_peer(mac);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = ch;
  p.encrypt = false;
  esp_now_add_peer(&p);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* mac = info->src_addr;
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len != sizeof(RoverPkt)) return;
  const RoverPkt* p = (const RoverPkt*)data;
  if (p->magic != PKT_MAGIC || p->type != PKT_ACK) return;
  if (!paired) memcpy(roverMac, mac, 6);
  paired = true;
  lastAck = millis();
  ackMode = p->mode;
  ackSpeed = p->speed;
}

void sendPkt(uint8_t type, int8_t x, int8_t y, int8_t z, int16_t arg, uint8_t flags) {
  RoverPkt p = {};
  p.magic = PKT_MAGIC;
  p.type = type;
  p.x = x; p.y = y; p.z = z;
  p.speed = speedPct;
  p.mode = modeReq;
  p.flags = flags;
  p.arg = arg;
  p.seq = ++seq;
  esp_now_send(roverMac, (uint8_t*)&p, sizeof(p));
}

void setChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

bool scanForRover() {
  uint8_t bc[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  for (uint8_t ch = 1; ch <= 13; ch++) {
    setChannel(ch);
    memcpy(roverMac, bc, 6);
    addPeer(bc, ch);
    paired = false;
    for (int i = 0; i < 3 && !paired; i++) {
      sendPkt(PKT_PING, 0, 0, 0, 0, 0);
      unsigned long t = millis();
      while (!paired && millis() - t < 40) delay(2);
    }
    if (paired) {
      roverCh = ch;
      addPeer(roverMac, ch);
      prefs.begin("remote", false);
      prefs.putBytes("mac", roverMac, 6);
      prefs.putUChar("ch", ch);
      prefs.end();
      Serial.printf("[LINK] Rover found on ch %d\n", ch);
      return true;
    }
  }
  return false;
}

void linkBegin(bool forceScan) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  prefs.begin("remote", true);
  bool have = prefs.getBytes("mac", roverMac, 6) == 6;
  roverCh = prefs.getUChar("ch", 1);
  prefs.end();
  if (have && !forceScan) {
    setChannel(roverCh);
    addPeer(roverMac, roverCh);
    paired = true;
    lastAck = millis();
  } else {
    scanForRover();
  }
}

// ---------------- Control logic ----------------
uint16_t prevPads = 0;
unsigned long irHoldStart = 0;
bool irFired = false;
unsigned long lastTx = 0;
bool wasDriving = false;

unsigned long modeChangedAt = 0;
void sendMode(uint8_t m) {
  modeReq = m;
  modeChangedAt = millis();
  sendPkt(PKT_MODE, 0, 0, 0, 0, 0);
  hapt(FX_DOUBLE);
}

void handleEncoder(bool fn) {
  static int32_t cnt = 0;
  static unsigned long firstT = 0, lastT = 0;
  portENTER_CRITICAL(&mux);
  int32_t a = encAccum;
  encAccum = 0;
  portEXIT_CRITICAL(&mux);
  if (a) {
    if (cnt == 0) firstT = millis();
    int32_t before = cnt / 4;
    cnt += a;
    lastT = millis();
    if (cnt / 4 != before) hapt(FX_TICK);
  }
  if (cnt == 0) return;
  if (abs(cnt) < 4) {
    if (millis() - lastT > 150) cnt = 0;
    return;
  }
  if (millis() - lastT < 180) return;
  static unsigned long lastRot = 0;
  int detents = cnt / 4;
  bool flick = abs(detents) >= 3 && (lastT - firstT) < 350;
  cnt = 0;
  if (millis() - lastRot < 250) return;
  lastRot = millis();
  int dir = detents > 0 ? 1 : -1;
  int16_t deg = (flick || fn) ? 360 * dir : 15 * detents;
  sendPkt(PKT_ROTATE, 0, 0, 0, deg, 0);
  hapt(abs(deg) >= 360 ? FX_LONG : FX_CLICK);
}

void handleKeys(uint16_t k) {
  uint16_t pressed = k & ~prevPads;
  bool fn = k & BIT(K_FN);
  if (pressed) hapt(FX_CLICK);

  if ((k & IR_TRIGGER_MASK) == IR_TRIGGER_MASK) {
    if (!irHoldStart) irHoldStart = millis();
    else if (!irFired && millis() - irHoldStart >= IR_HOLD_MS) {
      sendHaierPower();
      hapt(FX_TRIPLE);
      irFired = true;
    }
  } else {
    irHoldStart = 0;
    irFired = false;
  }

  if (pressed & BIT(K_STOP)) {
    sendPkt(fn ? PKT_HEADING_RESET : PKT_STOP, 0, 0, 0, 0, 0);
    hapt(FX_LONG);
  }
  if (fn) {
    if (pressed & BIT(K_Q)) sendPkt(PKT_ROTATE, 0, 0, 0, -360, 0);
    if (pressed & BIT(K_E)) sendPkt(PKT_ROTATE, 0, 0, 0, 360, 0);
    if (pressed & BIT(K_A)) sendPkt(PKT_ROTATE, 0, 0, 0, -90, 0);
    if (pressed & BIT(K_D)) sendPkt(PKT_ROTATE, 0, 0, 0, 90, 0);
    if (pressed & BIT(K_S)) sendPkt(PKT_ROTATE, 0, 0, 0, 180, 0);
    if (pressed & BIT(K_MODE)) sendMode((modeReq + MODE_COUNT - 1) % MODE_COUNT);
    if (pressed & BIT(K_SPDUP)) { speedPct = 100; hapt(FX_DOUBLE); }
    if (pressed & BIT(K_SPDDN)) { speedPct = 20; hapt(FX_DOUBLE); }
  } else {
    if (pressed & BIT(K_MODE)) sendMode((modeReq + 1) % MODE_COUNT);
    if (pressed & BIT(K_SPDUP)) speedPct = min(100, speedPct + 10);
    if (pressed & BIT(K_SPDDN)) speedPct = max(10, speedPct - 10);
  }
  prevPads = k;
}

void driveTick(uint16_t k) {
  bool fn = k & BIT(K_FN);
  int x = 0, y = 0, z = 0;
  if (!fn) {
    if (k & BIT(K_W)) y += 100;
    if (k & BIT(K_S)) y -= 100;
    if (k & BIT(K_D)) x += 100;
    if (k & BIT(K_A)) x -= 100;
    if (k & BIT(K_E)) z += 100;
    if (k & BIT(K_Q)) z -= 100;
    if (k & BIT(K_DFL)) { x -= 100; y += 100; }
    if (k & BIT(K_DFR)) { x += 100; y += 100; }
    if (k & BIT(K_DBL)) { x -= 100; y -= 100; }
    if (k & BIT(K_DBR)) { x += 100; y -= 100; }
  }
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);
  z = constrain(z, -100, 100);
  bool driving = x || y || z;
  uint8_t flags = (k & BIT(K_TURBO)) ? FLAG_TURBO : 0;
  if (digitalRead(DIP_CRAWL_PIN) == LOW) flags |= FLAG_CRAWL;

  unsigned long period = driving ? 40 : 200;
  if (driving != wasDriving || millis() - lastTx >= period) {
    sendPkt(PKT_DRIVE, x, y, z, 0, flags);
    lastTx = millis();
    wasDriving = driving;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH);
  pinMode(SDO_PIN, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(DIP_CRAWL_PIN, INPUT_PULLUP);
  pinMode(DIP_PAIR_PIN, INPUT_PULLUP);
  pinMode(DIP_VIBE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR, CHANGE);
  ledcAttach(MOTOR_PIN, 12000, 8);
  ledcWrite(MOTOR_PIN, 0);
  irSend.begin();

  vibeOn = digitalRead(DIP_VIBE_PIN) == LOW;
  linkBegin(digitalRead(DIP_PAIR_PIN) == LOW);
  hapt(paired ? FX_DOUBLE : FX_TRIPLE);
}

void loop() {
  vibeOn = digitalRead(DIP_VIBE_PIN) == LOW;
  hapticTick();
  uint16_t k = pads();
  handleKeys(k);
  handleEncoder(k & BIT(K_FN));
  driveTick(k);

  static bool linkWarned = false;
  bool linked = millis() - lastAck < 3000;
  if (!linked) {
    if (!linkWarned) { hapt(FX_TRIPLE); linkWarned = true; }
    static unsigned long lastScan = 0;
    if (millis() - lastScan > 5000) {
      lastScan = millis();
      if (!scanForRover()) setChannel(roverCh);
    }
  } else {
    linkWarned = false;
    if (ackMode != modeReq && millis() - modeChangedAt > 1500) modeReq = ackMode;
  }
  delay(4);
}

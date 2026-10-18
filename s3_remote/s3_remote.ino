#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <soc/gpio_struct.h>
#include <driver/gpio.h>
#include "pin_defs.h"
#include "rover_proto.h"
#include "qmi8658.h"
#include "haptics.h"
#include "display_ui.h"

// ---------------- Global Objects ----------------
QMI8658Driver qmi;
HapticEngine haptics;
DisplayUI ui;
Preferences prefs;

RemoteTelemetry telemetry;
float localYaw = 0.0f;
float filteredPitch = 0.0f;
float filteredRoll = 0.0f;

// ESP-NOW parameters
uint8_t targetRoverMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool roverLocked = false;
unsigned long lastAckTime = 0;
uint16_t packetSeq = 0;

// Joystick calibration & active deflection parameters
int joyCenterX = 2048;
int joyCenterY = 2048;
const float JOY_ACTIVE_SPAN = 1650.0f; // Full physical throw span (+/-1650 counts) for fine low-speed control


// Rotary Encoder variables
volatile int32_t encDelta = 0;
volatile int32_t encTotalSteps = 0;
volatile uint32_t encRawHits = 0;
volatile uint8_t encLastClk = 1;
volatile uint8_t encLastDt = 1;
volatile uint8_t encLastSw = 1;
volatile int8_t jogState = 0; // 0 = Neutral, +1 = Deflected CW, -1 = Deflected CCW
volatile bool jogArmed = true;  // Strict neutral re-arm: must return to (1,1) before next click
volatile uint32_t jogLastActivityUs = 0;
volatile uint32_t jogDeflectStartUs = 0;
volatile uint32_t jogLastRepeatUs = 0;
volatile uint8_t activeDtPinDetected = 17;
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

// Standard 4-state quadrature lookup table (prev<<2 | curr)
static const int8_t qdec_table[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR processEncoder() {
  uint32_t gpio_in = GPIO.in;
  uint8_t clk = (gpio_in >> ENC_CLK_PIN) & 1;
  uint8_t dt = (gpio_in >> ENC_DT_PIN) & 1; // Strictly GPIO 17 (Phase B), CLK on GPIO 16 (Phase A)

  uint8_t pinState = (clk << 1) | dt;
  static uint8_t lastPinState = 0x03;
  if (pinState == lastPinState) return;

  uint32_t nowUs = micros();
  static uint32_t lastEdgeUs = 0;
  if (nowUs - lastEdgeUs < 100) return; // 100us debounce preserves quadrature phase
  lastEdgeUs = nowUs;

  uint8_t prevPin = lastPinState;
  lastPinState = pinState;

  encLastClk = clk;
  encLastDt = dt;
  encLastSw = (gpio_in >> ENC_SW_PIN) & 1;
  encRawHits++;

  jogLastActivityUs = nowUs;

  // Universal quadrature step lookup
  int8_t step = qdec_table[((prevPin & 0x03) << 2) | (pinState & 0x03)];
  if (step == 0) return;

  static int8_t subStep = 0;
  subStep += step;

  // On detent rest positions (0x03) or accumulated 2 transitions, fire a step
  if (pinState == 0x03 && abs(subStep) >= 1) {
    int8_t dir = (subStep > 0) ? 1 : -1;
    portENTER_CRITICAL_ISR(&encMux);
    encDelta += dir;
    encTotalSteps += dir;
    portEXIT_CRITICAL_ISR(&encMux);
    subStep = 0;
  } else if (subStep >= 2) {
    portENTER_CRITICAL_ISR(&encMux);
    encDelta++;
    encTotalSteps++;
    portEXIT_CRITICAL_ISR(&encMux);
    subStep = 0;
  } else if (subStep <= -2) {
    portENTER_CRITICAL_ISR(&encMux);
    encDelta--;
    encTotalSteps--;
    portEXIT_CRITICAL_ISR(&encMux);
    subStep = 0;
  }
}

void IRAM_ATTR onClkISR() {
  processEncoder();
}

void IRAM_ATTR onDtISR() {
  processEncoder();
}

// ---------------- ESP-NOW Callbacks ----------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* mac = info->src_addr;
#else
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len == sizeof(TelemPkt) && data[0] == PKT_MAGIC && data[1] == PKT_TELEM) {
    if (!roverLocked) {
      memcpy(targetRoverMac, mac, 6);
      roverLocked = true;
      if (esp_now_is_peer_exist(targetRoverMac)) esp_now_del_peer(targetRoverMac);
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, targetRoverMac, 6);
      peer.channel = 0;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
      Serial.println("[REMOTE] Locked to Rover ESP-NOW MAC from Telemetry!");
    } else if (memcmp(mac, targetRoverMac, 6) != 0) {
      return;
    }
    const TelemPkt* t = (const TelemPkt*)data;
    float qMagSq = (float)t->q[0]*t->q[0] + (float)t->q[1]*t->q[1] + (float)t->q[2]*t->q[2] + (float)t->q[3]*t->q[3];
    if (qMagSq > 50000000.0f) {
      telemetry.q.w = t->q[0] / 16384.0f;
      telemetry.q.x = t->q[1] / 16384.0f;
      telemetry.q.y = t->q[2] / 16384.0f;
      telemetry.q.z = t->q[3] / 16384.0f;
      float q0 = telemetry.q.w, q1 = telemetry.q.x, q2 = telemetry.q.y, q3 = telemetry.q.z;
      float sinp = 2.0f * (q0 * q2 - q3 * q1);
      telemetry.imuRollDeg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
      telemetry.imuPitchDeg = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp) : asinf(sinp) * RAD_TO_DEG;
      float y = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
      telemetry.imuYawDeg = fmodf(y + 3600.0f, 360.0f);
      telemetry.roverHeading = (int16_t)lroundf(telemetry.imuYawDeg);
    }
    for (int i = 0; i < 3; i++) {
      telemetry.gyro[i] = t->gyro[i] / 16.0f;
      telemetry.acc[i] = t->acc[i] / 1000.0f;
    }
    telemetry.ultraMm = t->ultraMm;
    telemetry.tofMm[0] = t->tofMm[0]; telemetry.tofMm[1] = t->tofMm[1];
    for (int i = 0; i < 4; i++) telemetry.wheelMmS[i] = t->wheelMmS[i];
    telemetry.posCm[0] = t->posCm[0]; telemetry.posCm[1] = t->posCm[1];
    telemetry.calFlags = t->calFlags; telemetry.magCoverage = t->magCoverage;
    telemetry.hasTelem = true; telemetry.lastTelemMs = millis();
    lastAckTime = millis();
    telemetry.linked = true;
    return;
  }
  if (len != sizeof(RoverPkt)) return;
  const RoverPkt* p = (const RoverPkt*)data;
  if (p->magic != PKT_MAGIC || p->type != PKT_ACK) return;

  if (!roverLocked) {
    memcpy(targetRoverMac, mac, 6);
    roverLocked = true;
    // Add specific unicast peer
    if (esp_now_is_peer_exist(targetRoverMac)) esp_now_del_peer(targetRoverMac);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, targetRoverMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }

  lastAckTime = millis();
  telemetry.linked = true;
  telemetry.roverMode = p->mode;
  telemetry.semiAuto = (p->mode == MODE_AUTO);
  telemetry.roverSpeed = p->speed;
  if (p->flags & FLAG_IMPACT) {
    haptics.play(FX_ALERT);
  }
  if (telemetry.calFlags & CAL_MAG_OK) {
    telemetry.roverHeading = (int16_t)(((p->arg % 360) + 360) % 360);
  }
}

void sendPacket(uint8_t type, int8_t x, int8_t y, int8_t z, int16_t arg, uint8_t flags) {
  RoverPkt pkt = {};
  pkt.magic = PKT_MAGIC;
  pkt.type = type;
  pkt.x = x;
  pkt.y = y;
  pkt.z = z;
  uint8_t effAutoSpd = min(telemetry.autoSpeed, telemetry.speedLimit);
  pkt.speed = (flags & FLAG_TURBO) ? 100 : ((telemetry.semiAuto || telemetry.roverMode == MODE_AUTO) ? effAutoSpd : telemetry.speedLimit);
  if (telemetry.semiAuto) telemetry.roverMode = MODE_AUTO;
  pkt.mode = telemetry.roverMode;
  uint8_t sysFlags = (telemetry.semiAuto || telemetry.roverMode == MODE_AUTO ? FLAG_AUTO : 0) |
                     (telemetry.crashProtect ? 0 : FLAG_CRASH_OFF);
  pkt.flags = flags | sysFlags;
  pkt.seq = ++packetSeq;
  pkt.arg = (type == PKT_DRIVE) ? 0 : arg;

  static const uint8_t bcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (roverLocked && telemetry.linked) {
    esp_now_send(targetRoverMac, (uint8_t*)&pkt, sizeof(pkt));
  } else {
    esp_now_send(bcastMac, (uint8_t*)&pkt, sizeof(pkt));
    if (roverLocked) esp_now_send(targetRoverMac, (uint8_t*)&pkt, sizeof(pkt));
  }
}

// (Display is updated directly in loop() for deterministic, crash-free SPI rendering)

// ---------------- Input Processing ----------------
void calibrateJoystick() {
  long sumX = 0, sumY = 0;
  for (int i = 0; i < 30; i++) {
    sumX += analogRead(JOY_X_PIN);
    sumY += analogRead(JOY_Y_PIN);
    delay(4);
  }
  joyCenterX = sumX / 30;
  joyCenterY = sumY / 30;
  Serial.printf("[REMOTE] Joystick Center Calibrated: X=%d, Y=%d (Active Span: +/-%.0f)\n", joyCenterX, joyCenterY, JOY_ACTIVE_SPAN);
}

void readBattery() {
  static unsigned long lastBatRead = 0;
  if (millis() - lastBatRead < 1000) return;
  lastBatRead = millis();

  // 12-bit ADC, divider is 200k / 100k (3.0x ratio)
  uint32_t mv = analogReadMilliVolts(BATTERY_ADC_PIN);
  float v = (mv * 3.0f) / 1000.0f;
  telemetry.vbat = v;

  // Approximate LiPo curve (3.40V = 0%, 4.20V = 100%)
  int pct = (int)((v - 3.40f) / (4.20f - 3.40f) * 100.0f);
  telemetry.vbatPct = constrain(pct, 0, 100);
}

// Progressive expo filter for ultra-smooth low-speed crawl and full authority at limits
float applyExpo(float val, float expo) {
  float sign = (val >= 0.0f) ? 1.0f : -1.0f;
  float a = fabsf(val);
  // Quadratic-linear blend provides soft, slow, high-precision control on slight stick movements
  return sign * ((1.0f - expo) * a + expo * a * a);
}

// ---------------- Settings Persistence (NVS Preferences) ----------------
void loadPreferences() {
  if (prefs.begin("s3remote", true)) {
    telemetry.speedLimit = prefs.getUChar("spd", 60);
    telemetry.autoSpeed = prefs.getUChar("aspd", 60);
    if (telemetry.autoSpeed < 30) telemetry.autoSpeed = 60;
    telemetry.deadzonePct = prefs.getUChar("dead", 6);
    telemetry.expoPct = prefs.getUChar("expo", 45);
    telemetry.airAxis = prefs.getUChar("aax", 0);
    telemetry.trimFwd = prefs.getChar("tfwd", 0);
    telemetry.trimStrafe = prefs.getChar("tstr", 0);
    telemetry.trimYaw = prefs.getChar("tyaw", 0);
    telemetry.gyroSmoothing = prefs.getUChar("gyros", 4);
    telemetry.brightness = prefs.getUChar("brt", 80);
    telemetry.hapticsOn = prefs.getBool("hapt", true);
    telemetry.hapticStrength = prefs.getUChar("hstr", 100);
    if (telemetry.hapticStrength < 80) telemetry.hapticStrength = 100; // Bump legacy 60% default for small motor
    telemetry.encMode = prefs.getUChar("emode", 2);
    telemetry.encRepeat = prefs.getUChar("erep", 1);
    telemetry.encDtPin = prefs.getUChar("epin", 0);
    telemetry.encSens = prefs.getUChar("esens", 0);
    telemetry.crashProtect = prefs.getBool("cprot", true);
    telemetry.semiAuto = prefs.getBool("sauto", false);
    if (telemetry.semiAuto) telemetry.roverMode = MODE_AUTO;
    prefs.end();
  }
  haptics.enabled = telemetry.hapticsOn;
  haptics.strength = telemetry.hapticStrength;
}

void savePreferences() {
  if (prefs.begin("s3remote", false)) {
    prefs.putUChar("spd", telemetry.speedLimit);
    prefs.putUChar("aspd", telemetry.autoSpeed);
    prefs.putUChar("dead", telemetry.deadzonePct);
    prefs.putUChar("expo", telemetry.expoPct);
    prefs.putUChar("aax", telemetry.airAxis);
    prefs.putChar("tfwd", telemetry.trimFwd);
    prefs.putChar("tstr", telemetry.trimStrafe);
    prefs.putChar("tyaw", telemetry.trimYaw);
    prefs.putUChar("gyros", telemetry.gyroSmoothing);
    prefs.putUChar("brt", telemetry.brightness);
    prefs.putBool("hapt", telemetry.hapticsOn);
    prefs.putUChar("hstr", telemetry.hapticStrength);
    prefs.putUChar("emode", telemetry.encMode);
    prefs.putUChar("erep", telemetry.encRepeat);
    prefs.putUChar("epin", telemetry.encDtPin);
    prefs.putUChar("esens", telemetry.encSens);
    prefs.putBool("cprot", telemetry.crashProtect);
    prefs.putBool("sauto", telemetry.semiAuto);
    prefs.end();
    Serial.println("[REMOTE] Settings saved to NVS Preferences.");
  }
}

// ---------------- Menu Action Handler ----------------
void handleMenuAction(int act) {
  switch (act) {
    case ACT_MODE:
    case ACT_SEMI_AUTO:
    case ACT_AUTO_SPEED:
      sendPacket(PKT_MODE, 0, 0, 0, 0, 0);
      haptics.play(FX_MODE_SWITCH);
      break;
    case ACT_CRASH_PROT:
      sendPacket(PKT_MODE, 0, 0, 0, 0, 0);
      haptics.play(FX_CLICK);
      break;
    case ACT_AIR_AXIS: haptics.play(FX_CLICK); break;
    case ACT_HAPTICS:
      haptics.enabled = telemetry.hapticsOn;
      if (haptics.enabled) haptics.play(FX_CLICK);
      break;
    case ACT_HAPTIC_STR:
      haptics.strength = telemetry.hapticStrength;
      haptics.play(FX_CLICK);
      break;
    case ACT_BRIGHT: ui.setBacklight(telemetry.brightness); break;
    case ACT_ENC_MODE:
    case ACT_ENC_REPEAT:
    case ACT_ENC_PIN:
    case ACT_ENC_SENS:
      haptics.play(FX_CLICK);
      break;
    case ACT_CAL_JOY: calibrateJoystick(); haptics.play(FX_DOUBLE_CLICK); break;
    case ACT_CAL_AIR: qmi.calibrateZero(); localYaw = 0.0f; filteredPitch = 0.0f; filteredRoll = 0.0f; haptics.play(FX_DOUBLE_CLICK); break;
    case ACT_CAL_LEVEL: sendPacket(PKT_CALIB, 0, 0, 0, CAL_LEVEL, 0); break;
    case ACT_ZERO_YAW: localYaw = 0.0f; sendPacket(PKT_HEADING_RESET, 0, 0, 0, 0, 0); break;
    case ACT_SAVE_SETTINGS: savePreferences(); haptics.play(FX_SUCCESS); break;
    case ACT_EXIT: haptics.play(FX_DOUBLE_CLICK); break;
    default: break;
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("[REMOTE] Starting Waveshare ESP32-S3 Remote Controller...");

  // Load persistent user settings
  loadPreferences();
  haptics.enabled = telemetry.hapticsOn;

  // Setup I/O pins
  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_ALT_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(BTN_BRAKE_PIN, INPUT_PULLUP);
  pinMode(BTN_THROTTLE_PIN, INPUT_PULLUP);
  pinMode(BATTERY_ADC_PIN, INPUT);

  // Force hardware internal pullup registers
  gpio_set_pull_mode((gpio_num_t)ENC_CLK_PIN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)ENC_DT_PIN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)ENC_DT_ALT_PIN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)ENC_SW_PIN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)JOY_SW_PIN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t)BTN_BRAKE_PIN, GPIO_PULLUP_ONLY);
  // Setup Encoder ISRs strictly on Header P1 Pin 4 (GPIO 16 CLK) and Pin 5 (GPIO 17 DT)
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), onClkISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), onDtISR, CHANGE);

  // Setup Haptics
  haptics.begin(HAPTIC_PWM_PIN);

  // Setup Display
  ui.begin();

  // Setup Onboard QMI8658
  if (qmi.begin(QMI_SDA_PIN, QMI_SCL_PIN, QMI_I2C_ADDR)) {
    Serial.println("[REMOTE] QMI8658 IMU initialized successfully. Calibrating zero offsets...");
    qmi.calibrateZero();
    Serial.printf("[REMOTE] IMU Calibrated: P0=%.2f, R0=%.2f, Gz0=%.2f\n", qmi.pitchZero, qmi.rollZero, qmi.gzZero);
  } else {
    Serial.println("[REMOTE] QMI8658 IMU not found!");
  }

  // Setup Wi-Fi & ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Discover Rover / Router channel for zero-loss ESP-NOW link (default Ch 9)
  int targetCh = 9;
  int nNets = WiFi.scanNetworks(false, false, false, 150);
  for (int i = 0; i < nNets; i++) {
    String s = WiFi.SSID(i);
    if (s == "Mecanum-Rover" || s == "REMOVED") {
      targetCh = WiFi.channel(i);
      break;
    }
  }
  WiFi.scanDelete();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(targetCh, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.printf("[REMOTE] MAC: %s, Locked Ch: %d\n", WiFi.macAddress().c_str(), WiFi.channel());

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
    
    // Add broadcast peer for initial pairing
    esp_now_peer_info_t bcast = {};
    memset(bcast.peer_addr, 0xFF, 6);
    bcast.channel = 0;
    bcast.encrypt = false;
    esp_now_add_peer(&bcast);
    Serial.println("[REMOTE] ESP-NOW initialized.");
  }

  // Setup OTA
  ArduinoOTA.setHostname("S3-Mecanum-Remote");
  ArduinoOTA.begin();

  // Calibrate Joystick Center
  calibrateJoystick();

  // Boot musical fanfare confirmation
  haptics.play(FX_STARTUP);

  Serial.println("[REMOTE] Setup complete. Running main loop.");
}

// ---------------- Main Loop (Core 1: Sensor & Comm) ----------------
void loop() {
  unsigned long now = millis();

  // High-rate polling backup for rotary encoder
  processEncoder();

  static unsigned long encSpinUntil = 0;
  static int8_t encSpinZ = 0;

  // 1. Check Link Timeout & Auto Channel Hop if Unlinked
  if (telemetry.linked && (now - lastAckTime > 1200)) {
    telemetry.linked = false;
    haptics.play(FX_ALERT);
  }
  static unsigned long lastChanHop = 0;
  if (!telemetry.linked && (now - lastChanHop > 600)) {
    lastChanHop = now;
    static uint8_t hopCh = 1;
    hopCh = (hopCh % 13) + 1;
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(hopCh, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  }

  // 2. Battery Monitoring
  readBattery();

  // 2b. Continuously Poll Onboard IMU (Screen Rotated 180 deg)
  static unsigned long lastImuMs = 0;
  if (qmi.isConnected && (now - lastImuMs >= 10)) {
    float dt = (now - lastImuMs) / 1000.0f;
    lastImuMs = now;
    if (qmi.readSensor()) {
      static float prevGx = 0.0f, prevGy = 0.0f, prevGz = 0.0f;
      float dGx = fabsf(qmi.gx - prevGx), dGy = fabsf(qmi.gy - prevGy), dGz = fabsf(qmi.gz - prevGz);
      prevGx = qmi.gx; prevGy = qmi.gy; prevGz = qmi.gz;

      float cgx = qmi.gx - qmi.gxZero;
      float cgy = qmi.gy - qmi.gyZero;
      float cgz = qmi.gz - qmi.gzZero;
      float gNorm = sqrtf(qmi.ax * qmi.ax + qmi.ay * qmi.ay + qmi.az * qmi.az);
      if (gNorm < 0.1f) gNorm = 1.0f;

      // Upward vertical unit vector (toward ceiling) in QMI8658 frame is (+ax, +ay, +az)/gNorm.
      // Clockwise heading rate (+deg/s when turning Right/East, -deg/s when turning Left/West):
      float worldYawRate = -(cgx * qmi.ax + cgy * qmi.ay + cgz * qmi.az) / gNorm;

      // Variance-based stillness check: immune to initial boot gyro bias even when held in hand!
      bool lowVariance = (dGx < 3.5f && dGy < 3.5f && dGz < 3.5f);
      bool still = lowVariance && (fabsf(worldYawRate) < 8.5f);
      static uint16_t stillCount = 0;
      if (still) {
        stillCount++;
        // Fast auto-adapt 3-axis gyro bias so 3D car has 0.0 deg/s drift in hand or on desk
        qmi.gxZero += (qmi.gx - qmi.gxZero) * 0.06f;
        qmi.gyZero += (qmi.gy - qmi.gyZero) * 0.06f;
        qmi.gzZero += (qmi.gz - qmi.gzZero) * 0.06f;
        worldYawRate = 0.0f;

        // Smart posture auto-leveling when clutch is not engaged:
        if (!telemetry.deadmanEngaged) {
          float curP = atan2f(qmi.ay / gNorm, -qmi.az / gNorm) * (180.0f / (float)M_PI);
          float curR = asinf(constrain(-qmi.ax / gNorm, -1.0f, 1.0f)) * (180.0f / (float)M_PI);
          if (stillCount > 25 && (fabsf(qmi.pitch) > 25.0f || fabsf(qmi.roll) > 25.0f)) {
            qmi.pitchZero = curP;
            qmi.rollZero = curR;
            qmi.pitch = 0.0f;
            qmi.roll = 0.0f;
            filteredPitch = 0.0f;
            filteredRoll = 0.0f;
          } else if (stillCount > 45 && fabsf(qmi.pitch) < 6.0f && fabsf(qmi.roll) < 6.0f) {
            float dp0 = curP - qmi.pitchZero;
            while (dp0 > 180.0f) dp0 -= 360.0f;
            while (dp0 < -180.0f) dp0 += 360.0f;
            qmi.pitchZero += dp0 * 0.03f;
            qmi.rollZero += (curR - qmi.rollZero) * 0.03f;
          }
        }
      } else {
        stillCount = 0;
        if (fabsf(worldYawRate) > 8.5f) {
          localYaw += (worldYawRate * DEG_TO_RAD) * dt;
        }
      }

      // Also integrate joystick steering into heading when driving via Joystick without Rover mag
      if (telemetry.ctrlSource == CTRL_JOYSTICK && abs(telemetry.curZ) > 5 && !(telemetry.calFlags & CAL_MAG_OK)) {
        localYaw += (telemetry.curZ * 1.2f * DEG_TO_RAD) * dt;
      }
      while (localYaw >= 2.0f * (float)M_PI) localYaw -= 2.0f * (float)M_PI;
      while (localYaw < 0.0f) localYaw += 2.0f * (float)M_PI;

      float rawPitch = qmi.pitch * DEG_TO_RAD;
      float rawRoll = qmi.roll * DEG_TO_RAD;
      float alpha = constrain(1.8f / (telemetry.gyroSmoothing + 1.0f), 0.30f, 0.90f);
      filteredPitch += (rawPitch - filteredPitch) * alpha;
      filteredRoll += (rawRoll - filteredRoll) * alpha;

      telemetry.ctrlPitchDeg = filteredPitch * RAD_TO_DEG;
      telemetry.ctrlRollDeg = filteredRoll * RAD_TO_DEG;

      static unsigned long lastImuLog = 0;
      if (now - lastImuLog >= 1000) {
        lastImuLog = now;
        Serial.printf("[CTRL-TILT] R:%+5.1f P:%+5.1f | RoverHdg:%d\n",
                      telemetry.ctrlRollDeg, telemetry.ctrlPitchDeg, telemetry.roverHeading);
      }
    }
  }

  // 3. Encoder Knob Navigation & Diagnostics
  uint32_t nowUs = micros();
  uint32_t curGpio = GPIO.in;
  int rawClk  = (curGpio >> ENC_CLK_PIN) & 1;
  int rawDt17 = (curGpio >> ENC_DT_PIN) & 1;
  int rawDt10 = (curGpio >> ENC_DT_ALT_PIN) & 1;
  int rawSw   = (curGpio >> ENC_SW_PIN) & 1;
  int activeDt = rawDt17;

  uint8_t curPinState = (rawClk << 1) | activeDt;

  static uint32_t neutralSinceUs = 0;
  if (curPinState == 0x03) {
    if (neutralSinceUs == 0) neutralSinceUs = nowUs;
    // Settled at neutral (1,1) for >= 18ms: spring return completed, re-arm for next clean deflection
    if (!jogArmed && (nowUs - neutralSinceUs >= 18000UL)) {
      jogArmed = true;
      jogState = 0;
    }
  } else {
    neutralSinceUs = 0;
  }

  int32_t delta = 0;
  portENTER_CRITICAL(&encMux);
  delta = encDelta;
  encDelta = 0;
  portEXIT_CRITICAL(&encMux);

  if (delta != 0) {
    const char* mStr = (telemetry.encMode == 0 ? "Jog" : (telemetry.encMode == 1 ? "3-Way" : "Cont"));
    Serial.printf("[ENC] CLK:%d DT:%d (IO17) Step:%d Mode:%s\n",
                  rawClk, activeDt, delta, mStr);
    haptics.play(FX_TICK);
    if (ui.inMenu) {
      if (delta > 0) ui.menuNext();
      else ui.menuPrev();
    } else {
      if (ui.view == VIEW_ATTITUDE) {
        // On the Rover 3D HUD Screen: Turn knob to smoothly rotate the 3D rover 360 degrees for inspection
        telemetry.inspectYaw += (float)delta * 10.0f;
        while (telemetry.inspectYaw >= 360.0f) telemetry.inspectYaw -= 360.0f;
        while (telemetry.inspectYaw < 0.0f) telemetry.inspectYaw += 360.0f;
      }
      // Real-time Rover Spin Command based on encoder turns:
      // Clockwise (+delta) -> spin right (+z); Counter-clockwise (-delta) -> spin left (-z)
      int spinPower = constrain((int)telemetry.speedLimit, 45, 85);
      encSpinZ = (delta > 0) ? spinPower : -spinPower;
      unsigned long spinDurationMs = (unsigned long)(abs(delta) * 180);
      encSpinUntil = max(now, encSpinUntil) + spinDurationMs;
      if (encSpinUntil > now + 650) encSpinUntil = now + 650;
      sendPacket(PKT_DRIVE, 0, 0, encSpinZ, 0, 0);
      Serial.printf("[ENC] Realtime Spin: dir=%s, pwr=%d, durMs=%lu\n",
                    (delta > 0 ? "CW" : "CCW"), (int)encSpinZ, (unsigned long)(encSpinUntil - now));
    }
  }

  // Periodic encoder hardware diagnostics every 1000ms
  static unsigned long lastEncDiag = 0;
  if (now - lastEncDiag >= 1000) {
    lastEncDiag = now;
    const char* mStr = (telemetry.encMode == 0 ? "Jog/Spring" : (telemetry.encMode == 1 ? "3-Way Sw" : "Cont"));
    Serial.printf("[ENC] Diagnostic: CLK(16):%d DT(17):%d DT_ALT(10):%d SW(18):%d | Active:IO%d Mode:%s JogSt:%d | Steps:%ld | Hits:%lu\n",
                  rawClk, rawDt17, rawDt10, rawSw, activeDtPinDetected,
                  mStr, jogState, encTotalSteps, encRawHits);
  }

  // 4. Encoder Push Button (IO18):
  // - Press & hold >= 380ms: immediately opens/closes Settings Menu with FX_DOUBLE_CLICK
  // - Short release (< 380ms):
  //     In menu -> selects/adjusts highlighted parameter
  //     Outside menu -> cycles views (Attitude -> Map -> Sensors -> Drive)
  static bool lastEncSw = HIGH;
  static unsigned long encDownAt = 0;
  static bool longPressHandled = false;
  bool encSw = (rawSw == 0) ? LOW : HIGH;

  if (lastEncSw == HIGH && encSw == LOW) {
    encDownAt = now;
    longPressHandled = false;
  }

  // Active trigger WHILE holding knob down (no guessing when to release)
  if (encSw == LOW && !longPressHandled && (now - encDownAt >= 380)) {
    longPressHandled = true;
    ui.toggleMenu();
    haptics.play(FX_DOUBLE_CLICK);
    Serial.printf("[REMOTE] Menu toggled via Knob Long-Press (inMenu=%d)\n", ui.inMenu);
  }

  // Short click action upon release
  if (lastEncSw == LOW && encSw == HIGH) {
    if (!longPressHandled && (now - encDownAt > 20)) {
      if (ui.inMenu) {
        haptics.play(FX_CLICK);
        int act = ui.menuSelect(telemetry);
        handleMenuAction(act);
      } else if (ui.view == VIEW_ATTITUDE && fabsf(telemetry.inspectYaw) > 0.1f) {
        telemetry.inspectYaw = 0.0f;
        localYaw = 0.0f;
        qmi.calibrateZero();
        filteredPitch = 0.0f;
        filteredRoll = 0.0f;
        haptics.play(FX_DOUBLE_CLICK);
        Serial.println("[ENC] 360 Inspection Yaw & IMU Reset to 0 deg");
      } else {
        ui.nextView();
        qmi.calibrateZero();
        filteredPitch = 0.0f;
        filteredRoll = 0.0f;
        haptics.play(FX_CLICK);
      }
    }
  }
  lastEncSw = encSw;
  if (telemetry.hasTelem && now - telemetry.lastTelemMs > 1500) telemetry.hasTelem = false;

  // 5. Joystick Menu Navigation (Active only when ui.inMenu is true)
  static unsigned long joyNavLastMoveMs = 0;
  static unsigned long joyNavFirstMoveMs = 0;
  static bool joyNavNeutral = true;
  const unsigned long JOY_REPEAT_FIRST_DELAY = 380; // ms before auto-repeat begins
  const unsigned long JOY_REPEAT_INTERVAL = 180;    // ms repeat interval while held

  if (ui.inMenu) {
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);
    float nx = (float)(rawX - joyCenterX) / JOY_ACTIVE_SPAN;
    float ny = (float)(joyCenterY - rawY) / JOY_ACTIVE_SPAN; // Stick UP: ny > 0, Stick DOWN: ny < 0

    bool stickDeflected = (fabsf(ny) > 0.50f || fabsf(nx) > 0.50f);

    if (stickDeflected) {
      bool canMove = false;
      if (joyNavNeutral) {
        canMove = true;
        joyNavNeutral = false;
        joyNavFirstMoveMs = now;
        joyNavLastMoveMs = now;
      } else if (now - joyNavFirstMoveMs >= JOY_REPEAT_FIRST_DELAY && now - joyNavLastMoveMs >= JOY_REPEAT_INTERVAL) {
        canMove = true;
        joyNavLastMoveMs = now;
      }

      if (canMove) {
        if (fabsf(ny) >= fabsf(nx)) {
          // Stick UP / DOWN: moves selection between menu items
          if (ny > 0) {
            ui.menuPrev();
            haptics.play(FX_TICK);
            Serial.printf("[MENU-JOY] Stick UP -> Item: %d\n", ui.menuIndex);
          } else {
            ui.menuNext();
            haptics.play(FX_TICK);
            Serial.printf("[MENU-JOY] Stick DOWN -> Item: %d\n", ui.menuIndex);
          }
        } else {
          // Stick LEFT / RIGHT: adjusts parameters
          int dir = (nx > 0) ? +1 : -1;
          int act = ui.menuAdjust(telemetry, dir);
          handleMenuAction(act);
          haptics.play(FX_TICK);
          Serial.printf("[MENU-JOY] Stick %s -> Adjust Item %d (act=%d)\n",
                        (dir > 0 ? "RIGHT" : "LEFT"), ui.menuIndex, act);
        }
      }
    } else if (fabsf(nx) < 0.20f && fabsf(ny) < 0.20f) {
      // Hysteresis release: re-arms single-click deflection
      joyNavNeutral = true;
    }
  }

  // 5b. Joystick Button (IO6):
  // - In Menu: Confirms / Selects / Toggles current menu item (same as IO18)
  // - Long Press (>= 400ms): Toggles Settings Menu
  // - Short Click outside Menu: Cycles Primary Drive Modes (Normal -> Sport -> Tank -> Crawl -> Normal)
  static bool lastJoySw = HIGH;
  static unsigned long joySwDownAt = 0;
  static bool joyLongPressHandled = false;
  bool joySw = digitalRead(JOY_SW_PIN);

  if (lastJoySw == HIGH && joySw == LOW) {
    joySwDownAt = now;
    joyLongPressHandled = false;
  }

  // Long press while holding stick button down
  if (joySw == LOW && !joyLongPressHandled && (now - joySwDownAt >= 400)) {
    joyLongPressHandled = true;
    ui.toggleMenu();
    haptics.play(FX_DOUBLE_CLICK);
    Serial.printf("[REMOTE] Menu toggled via Joystick Button Long-Press (inMenu=%d)\n", ui.inMenu);
  }

  // Release action for short click
  if (lastJoySw == LOW && joySw == HIGH) {
    if (!joyLongPressHandled && (now - joySwDownAt > 20)) {
      if (ui.inMenu) {
        haptics.play(FX_CLICK);
        int act = ui.menuSelect(telemetry);
        handleMenuAction(act);
        Serial.printf("[MENU-JOY] Confirmed Item %d via Joystick Button (IO6)\n", ui.menuIndex);
      } else {
        uint8_t nextMode;
        switch (telemetry.roverMode) {
          case MODE_NORMAL: nextMode = MODE_SPORT; break;
          case MODE_SPORT:  nextMode = MODE_TANK; break;
          case MODE_TANK:   nextMode = MODE_CRAWL; break;
          case MODE_CRAWL:  nextMode = MODE_AUTO; break;
          case MODE_AUTO:   nextMode = MODE_NORMAL; break;
          default:          nextMode = MODE_NORMAL; break;
        }
        telemetry.roverMode = nextMode;
        telemetry.semiAuto = (nextMode == MODE_AUTO);
        sendPacket(PKT_MODE, 0, 0, 0, 0, 0);
        haptics.play(FX_MODE_SWITCH);
        Serial.printf("[REMOTE] Mode switched via Joystick Button to: %d\n", nextMode);
      }
    }
  }
  lastJoySw = joySw;

  // 6. Push Buttons: IO7 = Active Brake, IO8 = Full Throttle / Push-to-Gyrate Clutch
  bool brakeBtn = (digitalRead(BTN_BRAKE_PIN) == LOW);
  bool throttleBtn = (digitalRead(BTN_THROTTLE_PIN) == LOW);

  static bool lastBrakeState = false;
  static bool lastThrottleState = false;

  if (brakeBtn && !lastBrakeState) {
    haptics.play(FX_BRAKE);
    sendPacket(PKT_STOP, 0, 0, 0, 0, 0); // Immediate dynamic brake stop
  }
  if (throttleBtn && !lastThrottleState) {
    if (telemetry.ctrlSource == CTRL_AIR_REMOTE) {
      haptics.play(FX_CLUTCH_ENGAGE);
    } else {
      haptics.play(FX_THROTTLE_BOOST);
    }
  } else if (!throttleBtn && lastThrottleState) {
    if (telemetry.ctrlSource == CTRL_AIR_REMOTE) {
      haptics.play(FX_CLUTCH_RELEASE);
    }
  }
  lastBrakeState = brakeBtn;
  lastThrottleState = throttleBtn;

  telemetry.isBrake = brakeBtn;
  telemetry.isTurbo = throttleBtn;

  // 7. Read Drive Commands
  int8_t cmdX = 0, cmdY = 0, cmdZ = 0;
  uint8_t flags = 0;

  if (ui.inMenu) {
    // Menu active: rover clamped to 0 stop for safety
    cmdX = 0;
    cmdY = 0;
    cmdZ = 0;
    telemetry.deadmanEngaged = false;
  } else if (brakeBtn) {
    // Dynamic brake zeroes all drive commands
    cmdX = 0;
    cmdY = 0;
    cmdZ = 0;
    telemetry.deadmanEngaged = false;
  } else if (telemetry.ctrlSource == CTRL_AIR_REMOTE) {
    // ---------------- Fine-tuned IMU Air Tilt Motion Engine ----------------
    // Push-to-Gyrate Switch: IO8 acts as active deadman clutch switch.
    // When IO8 is NOT pressed: drive commands are clamped to 0 (cmdX=0, cmdY=0, cmdZ=0),
    // artificial horizon tilts visually on screen, status banner displays "HOLD [BOOST/IO8] TO DRIVE".
    // When IO8 IS pressed: tilt angle drives rover at high-speed tank rotation/speed,
    // status banner displays "ENGAGED - DRIVING".
    telemetry.deadmanEngaged = throttleBtn;

    // Static ramp variables for silky-smooth slew-rate acceleration easing
    static float airRampY = 0.0f;
    static float airRampTurn = 0.0f;
    static unsigned long lastAirRampTime = 0;

    if (!throttleBtn) {
      // Clutched / Stopped: gently discharge ramp to zero
      airRampY = 0.0f;
      airRampTurn = 0.0f;
      lastAirRampTime = now;
      cmdX = 0;
      cmdY = 0;
      cmdZ = 0;
    } else {
      // Push-to-Gyrate clutch active! Gradual progressive speed buildup
      float dtRamp = (now - lastAirRampTime) / 1000.0f;
      lastAirRampTime = now;
      if (dtRamp <= 0.0f || dtRamp > 0.1f) dtRamp = 0.02f;

      float pDeg = telemetry.ctrlPitchDeg;
      float rDeg = telemetry.ctrlRollDeg;

      // User selectable Air Gyro Axis mapping:
      float throtDeg = pDeg;
      float steerDeg = -rDeg;
      if (telemetry.airAxis == 1) {
        throtDeg = rDeg;
        steerDeg = -pDeg;
      } else if (telemetry.airAxis == 2) {
        throtDeg = -pDeg;
        steerDeg = -rDeg;
      } else if (telemetry.airAxis == 3) {
        throtDeg = pDeg;
        steerDeg = rDeg;
      } else if (telemetry.airAxis == 4) {
        throtDeg = -pDeg;
        steerDeg = rDeg;
      } else if (telemetry.airAxis == 5) {
        throtDeg = -rDeg;
        steerDeg = pDeg;
      }

      // Ergonomic deadzone (filters hand tremor) and comfortable wrist tilt range
      const float DEADBAND_DEG = 9.0f;
      const float MAX_TILT_DEG = 45.0f;

      // Throttle mapping (Forward / Reverse) with progressive cubic easing (u^3):
      float effThrot = 0.0f;
      if (fabsf(throtDeg) > DEADBAND_DEG) {
        float u = (fabsf(throtDeg) - DEADBAND_DEG) / (MAX_TILT_DEG - DEADBAND_DEG);
        u = constrain(u, 0.0f, 1.0f);
        // Progressive cubic easing: ultra-gentle micro-crawl at small tilts, gradual speed buildup
        float eased = u * u * u;
        effThrot = (throtDeg > 0 ? 1.0f : -1.0f) * eased;
      }

      // Steering / Strafe / Tank Pivot mapping with progressive cubic easing:
      float effTurn = 0.0f;
      if (fabsf(steerDeg) > DEADBAND_DEG) {
        float u = (fabsf(steerDeg) - DEADBAND_DEG) / (MAX_TILT_DEG - DEADBAND_DEG);
        u = constrain(u, 0.0f, 1.0f);
        float eased = u * u * u;
        effTurn = (steerDeg > 0 ? 1.0f : -1.0f) * eased;
      }

      // Scaled responsiveness from user speed limit setting
      float spdFactor = telemetry.speedLimit / 100.0f;
      float targetY = effThrot * 100.0f * spdFactor;
      float targetTurn = effTurn * 100.0f * spdFactor;

      // Gentle, gradual acceleration ramp (~4.2s from 0 to 100%), responsive deceleration for safety
      float rampUpStep = 24.0f * dtRamp;    // ~0.48 units per 20ms: builds speed gradually and smoothly
      float rampDownStep = 150.0f * dtRamp; // ~3.0 units per 20ms: controlled braking
      float rampUpTurnStep = 28.0f * dtRamp;
      float rampDownTurnStep = 150.0f * dtRamp;

      if (fabsf(targetY) > fabsf(airRampY)) {
        if (targetY > airRampY) airRampY = min(targetY, airRampY + rampUpStep);
        else airRampY = max(targetY, airRampY - rampUpStep);
      } else {
        if (targetY > airRampY) airRampY = min(targetY, airRampY + rampDownStep);
        else airRampY = max(targetY, airRampY - rampDownStep);
      }

      if (fabsf(targetTurn) > fabsf(airRampTurn)) {
        if (targetTurn > airRampTurn) airRampTurn = min(targetTurn, airRampTurn + rampUpTurnStep);
        else airRampTurn = max(targetTurn, airRampTurn - rampUpTurnStep);
      } else {
        if (targetTurn > airRampTurn) airRampTurn = min(targetTurn, airRampTurn + rampDownTurnStep);
        else airRampTurn = max(targetTurn, airRampTurn - rampDownTurnStep);
      }

      int finalY = (int)roundf(airRampY);
      int finalTurn = (int)roundf(airRampTurn);

      if (telemetry.roverMode == MODE_TANK) {
        // High-speed Tank mode: Pitch = Fwd/Rev, Roll = Pivot rotation (cmdZ)
        cmdY = constrain(finalY, -100, 100);
        cmdZ = constrain(finalTurn, -100, 100);
        cmdX = 0;
      } else {
        // Omni-directional strafe modes
        cmdY = constrain(finalY, -100, 100);
        cmdX = constrain(finalTurn, -100, 100);
        cmdZ = 0;
      }
    }
  } else {
    // Standard Analog Joystick Mode (VRx on GPIO 2, VRy on GPIO 4)
    telemetry.deadmanEngaged = false;
    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    // Normalize around calibrated center using responsive nominal active span
    float nx = (float)(rawX - joyCenterX) / JOY_ACTIVE_SPAN;
    // Invert Y so pushing stick forward (up) produces positive forward velocity
    float ny = (float)(joyCenterY - rawY) / JOY_ACTIVE_SPAN;

    nx = constrain(nx, -1.0f, 1.0f);
    ny = constrain(ny, -1.0f, 1.0f);

    // Deadzone filter from user setting
    float deadzone = constrain(telemetry.deadzonePct / 100.0f, 0.02f, 0.20f);
    if (fabsf(nx) < deadzone) nx = 0.0f;
    else nx = (nx > 0 ? 1.0f : -1.0f) * (fabsf(nx) - deadzone) / (1.0f - deadzone);

    if (fabsf(ny) < deadzone) ny = 0.0f;
    else ny = (ny > 0 ? 1.0f : -1.0f) * (fabsf(ny) - deadzone) / (1.0f - deadzone);

    // Apply user configurable expo curve
    float expo = constrain(telemetry.expoPct / 100.0f, 0.0f, 0.85f);
    nx = applyExpo(nx, expo);
    ny = applyExpo(ny, expo);

    // Full resolution directional commands [-100 .. +100]
    cmdX = constrain((int)roundf(nx * 100.0f) + telemetry.trimStrafe, -100, 100);
    cmdY = constrain((int)roundf(ny * 100.0f) + telemetry.trimFwd, -100, 100);
    cmdZ = constrain((int)telemetry.trimYaw, -100, 100);

    // Full Throttle Button (IO8): Boosts to 100% full speed
    if (throttleBtn) {
      flags |= FLAG_TURBO;
      if (fabsf(nx) < 0.08f && fabsf(ny) < 0.08f) {
        // Neutral stick: launch full throttle straight forward
        cmdY = 100;
        cmdX = 0;
      } else {
        // Stick active: boost directional magnitude to 100%
        float mag = sqrtf(nx * nx + ny * ny);
        if (mag > 0.05f) {
          cmdX = constrain((int)roundf(nx / mag * 100.0f), -100, 100);
          cmdY = constrain((int)roundf(ny / mag * 100.0f), -100, 100);
        }
      }
    }

    // Inject Rotary Encoder spin pulse if active and joystick is not actively steering
    if (now < encSpinUntil && abs(cmdZ) < 15) {
      cmdZ = encSpinZ;
    }
  }

  // Also inject encoder spin in Air Remote mode if not turning hard
  if (telemetry.ctrlSource == CTRL_AIR_REMOTE && now < encSpinUntil && abs(cmdZ) < 15) {
    cmdZ = encSpinZ;
  }

  if (telemetry.roverMode == MODE_CRAWL) {
    flags |= FLAG_CRAWL;
  }

  telemetry.curX = cmdX;
  telemetry.curY = cmdY;
  telemetry.curZ = cmdZ;

  // 8. Transmit Control Packet (50Hz = every 20ms, or immediately on command change for zero lag)
  static unsigned long lastTxTime = 0;
  static int8_t prevX = 0, prevY = 0, prevZ = 0;
  static uint8_t prevFlags = 0;
  bool isEncSpinning = (now < encSpinUntil);
  bool cmdChanged = (abs(cmdX - prevX) >= 2 || abs(cmdY - prevY) >= 2 || abs(cmdZ - prevZ) >= 2 || flags != prevFlags || isEncSpinning);
  if ((cmdChanged && (now - lastTxTime >= 8)) || (now - lastTxTime >= 20)) {
    lastTxTime = now;
    prevX = cmdX; prevY = cmdY; prevZ = cmdZ; prevFlags = flags;
    if (brakeBtn) {
      sendPacket(PKT_STOP, 0, 0, 0, 0, 0);
    } else {
      sendPacket(PKT_DRIVE, cmdX, cmdY, cmdZ, 0, flags);
    }
  }

  // 9. Update Haptic Engine
  haptics.update();

  // 10. Periodic OTA handle (every 250ms, decoupled from display)
  static unsigned long lastOta = 0;
  if (now - lastOta >= 250) {
    lastOta = now;
    ArduinoOTA.handle();
  }

  // 11. Render Display UI (35 FPS = every 28ms so CPU spends >85% servicing IMU, Encoder, and ESP-NOW)
  static unsigned long lastUiRender = 0;
  if (now - lastUiRender >= 28) {
    lastUiRender = now;
    ui.render(telemetry);
  }
}

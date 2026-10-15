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
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

// Ben Buxton's half-step state table:
// Complete mechanical contact bounce rejection; emits cleanly on every detent
#define DIR_NONE 0x00
#define DIR_CW   0x10
#define DIR_CCW  0x20

static const uint8_t ttable_half[6][4] = {
  // 00             01           10           11
  { 0x3,            0x2,         0x1,         0x0 },            // 0: START (11)
  { 0x23 | DIR_CCW, 0x1,         0x1,         0x0 },            // 1: CCW from 11
  { 0x13 | DIR_CW,  0x2,         0x2,         0x0 },            // 2: CW from 11
  { 0x3,            0x5,         0x4,         0x0 },            // 3: START (00)
  { 0x3,            0x3,         0x4,         0x0 | DIR_CW },   // 4: CW from 00
  { 0x3,            0x5,         0x3,         0x0 | DIR_CCW }   // 5: CCW from 00
};

void IRAM_ATTR processEncoder() {
  uint32_t gpio_in = GPIO.in;
  uint8_t clk = (gpio_in >> ENC_CLK_PIN) & 1;
  uint8_t dt17 = (gpio_in >> ENC_DT_PIN) & 1;
  uint8_t dt10 = (gpio_in >> ENC_DT_ALT_PIN) & 1;
  uint8_t dt = (dt17 != 0) ? dt17 : dt10;
  uint8_t pinState = (clk << 1) | dt;

  static uint8_t lastPinState = 0xFF;
  if (pinState == lastPinState) return;
  lastPinState = pinState;

  encLastClk = clk;
  encLastDt = dt;
  encLastSw = (gpio_in >> ENC_SW_PIN) & 1;
  encRawHits++;

  static uint8_t encState = 0;
  static int8_t halfStepAccum = 0;

  uint8_t res = ttable_half[encState & 0x0F][pinState];
  encState = res & 0x0F;

  if (res & DIR_CW) {
    halfStepAccum++;
    if (halfStepAccum >= 2 || (halfStepAccum >= 1 && (pinState == 0x03 || pinState == 0x00))) {
      portENTER_CRITICAL_ISR(&encMux);
      encDelta++;
      encTotalSteps++;
      portEXIT_CRITICAL_ISR(&encMux);
      halfStepAccum = 0;
    }
  } else if (res & DIR_CCW) {
    halfStepAccum--;
    if (halfStepAccum <= -2 || (halfStepAccum <= -1 && (pinState == 0x03 || pinState == 0x00))) {
      portENTER_CRITICAL_ISR(&encMux);
      encDelta--;
      encTotalSteps--;
      portEXIT_CRITICAL_ISR(&encMux);
      halfStepAccum = 0;
    }
  }
}

void IRAM_ATTR onEncoderISR() {
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
      telemetry.q = {t->q[0] / 16384.0f, t->q[1] / 16384.0f, t->q[2] / 16384.0f, t->q[3] / 16384.0f};
    }
    for (int i = 0; i < 3; i++) { telemetry.gyro[i] = t->gyro[i] / 16.0f; telemetry.acc[i] = t->acc[i] / 1000.0f; }
    telemetry.ultraMm = t->ultraMm;
    telemetry.tofMm[0] = t->tofMm[0]; telemetry.tofMm[1] = t->tofMm[1];
    for (int i = 0; i < 4; i++) telemetry.wheelMmS[i] = t->wheelMmS[i];
    telemetry.posCm[0] = t->posCm[0]; telemetry.posCm[1] = t->posCm[1];
    telemetry.calFlags = t->calFlags; telemetry.magCoverage = t->magCoverage;
    telemetry.hasTelem = true; telemetry.lastTelemMs = millis();
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
  telemetry.roverSpeed = p->speed;
  telemetry.roverHeading = p->arg;
}

void sendPacket(uint8_t type, int8_t x, int8_t y, int8_t z, int16_t arg, uint8_t flags) {
  RoverPkt pkt = {};
  pkt.magic = PKT_MAGIC;
  pkt.type = type;
  pkt.x = x;
  pkt.y = y;
  pkt.z = z;
  pkt.speed = (flags & FLAG_TURBO) ? 100 : telemetry.speedLimit;
  pkt.mode = telemetry.roverMode;
  pkt.flags = flags;
  pkt.arg = arg;
  pkt.seq = ++packetSeq;

  esp_now_send(targetRoverMac, (uint8_t*)&pkt, sizeof(pkt));
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
    telemetry.deadzonePct = prefs.getUChar("dead", 6);
    telemetry.expoPct = prefs.getUChar("expo", 45);
    telemetry.trimFwd = prefs.getChar("tfwd", 0);
    telemetry.trimStrafe = prefs.getChar("tstr", 0);
    telemetry.trimYaw = prefs.getChar("tyaw", 0);
    telemetry.gyroSmoothing = prefs.getUChar("gyros", 4);
    telemetry.brightness = prefs.getUChar("brt", 80);
    telemetry.hapticsOn = prefs.getBool("hapt", true);
    telemetry.hapticStrength = prefs.getUChar("hstr", 60);
    prefs.end();
  }
  haptics.enabled = telemetry.hapticsOn;
  haptics.strength = telemetry.hapticStrength;
}

void savePreferences() {
  if (prefs.begin("s3remote", false)) {
    prefs.putUChar("spd", telemetry.speedLimit);
    prefs.putUChar("dead", telemetry.deadzonePct);
    prefs.putUChar("expo", telemetry.expoPct);
    prefs.putChar("tfwd", telemetry.trimFwd);
    prefs.putChar("tstr", telemetry.trimStrafe);
    prefs.putChar("tyaw", telemetry.trimYaw);
    prefs.putUChar("gyros", telemetry.gyroSmoothing);
    prefs.putUChar("brt", telemetry.brightness);
    prefs.putBool("hapt", telemetry.hapticsOn);
    prefs.putUChar("hstr", telemetry.hapticStrength);
    prefs.end();
    Serial.println("[REMOTE] Settings saved to NVS Preferences.");
  }
}

// ---------------- Menu Action Handler ----------------
void handleMenuAction(int act) {
  switch (act) {
    case ACT_MODE: sendPacket(PKT_MODE, 0, 0, 0, 0, 0); break;
    case ACT_HAPTICS:
      haptics.enabled = telemetry.hapticsOn;
      if (haptics.enabled) haptics.play(FX_CLICK);
      break;
    case ACT_HAPTIC_STR:
      haptics.strength = telemetry.hapticStrength;
      haptics.play(FX_CLICK);
      break;
    case ACT_BRIGHT: ui.setBacklight(telemetry.brightness); break;
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
  gpio_set_pull_mode((gpio_num_t)BTN_THROTTLE_PIN, GPIO_PULLUP_ONLY);

  // Setup Encoder ISR (both primary DT on IO17 and clean alternate DT on IO10)
  attachInterrupt(digitalPinToInterrupt(ENC_CLK_PIN), onEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_PIN), onEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT_ALT_PIN), onEncoderISR, CHANGE);

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
  Serial.printf("[REMOTE] MAC: %s, Ch: %d\n", WiFi.macAddress().c_str(), WiFi.channel());

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

  // Handle automatic default to High-Speed Tank Mode on IMU Navigation Page (VIEW_DRIVE) or Air Remote Mode
  if (telemetry.ctrlSource == CTRL_AIR_REMOTE || ui.view == VIEW_DRIVE) {
    if (telemetry.roverMode != MODE_TANK) {
      telemetry.roverMode = MODE_TANK;
      sendPacket(PKT_MODE, 0, 0, 0, 0, 0);
      haptics.play(FX_MODE_SWITCH);
      Serial.println("[REMOTE] Air Controller Mode: Enforced Tank Mode Only");
    }
  }

  // 1. Check Link Timeout
  if (telemetry.linked && (now - lastAckTime > 1200)) {
    telemetry.linked = false;
    haptics.play(FX_ALERT);
  }

  // 2. Battery Monitoring
  readBattery();

  // 2b. Continuously Poll Onboard IMU (Screen Rotated 180 deg)
  static unsigned long lastImuMs = 0;
  if (qmi.isConnected && (now - lastImuMs >= 10)) {
    float dt = (now - lastImuMs) / 1000.0f;
    lastImuMs = now;
    if (qmi.readSensor()) {
      // Screen is rotated 180 deg so user views display inverted:
      // Forward tilt of screen (top pitched down) = +ax
      // Right tilt of screen (right edge pitched down) = -ay
      // Calibrated pitch & roll from QMI8658 (rotation 2 frame)
      float rawPitch = qmi.pitch * DEG_TO_RAD;
      float rawRoll = qmi.roll * DEG_TO_RAD;

      float alpha = constrain(1.5f / (telemetry.gyroSmoothing + 1.0f), 0.25f, 0.85f);
      filteredPitch += (rawPitch - filteredPitch) * alpha;
      filteredRoll += (rawRoll - filteredRoll) * alpha;

      float compGz = qmi.gz - qmi.gzZero;
      if (fabsf(compGz) > 1.2f) {
        localYaw += (compGz * DEG_TO_RAD) * dt;
      }

      float cr = cosf(filteredRoll * 0.5f), sr = sinf(filteredRoll * 0.5f);
      float cp = cosf(filteredPitch * 0.5f), sp = sinf(filteredPitch * 0.5f);
      float cy = cosf(localYaw * 0.5f), sy = sinf(localYaw * 0.5f);

      Quat localQ;
      localQ.w = cr * cp * cy + sr * sp * sy;
      localQ.x = sr * cp * cy - cr * sp * sy;
      localQ.y = cr * sp * cy + sr * cp * sy;
      localQ.z = cr * cp * sy - sr * sp * cy;

      // Real-time remote IMU attitude: handheld drives attitude when no rover telemetry,
      // or when in Air Remote / IMU drive mode
      if (!telemetry.hasTelem || telemetry.ctrlSource == CTRL_AIR_REMOTE || ui.view == VIEW_DRIVE) {
        telemetry.q = localQ;
      }
      telemetry.gyro[0] = -qmi.gx;
      telemetry.gyro[1] = -qmi.gy;
      telemetry.gyro[2] = qmi.gz;
      telemetry.acc[0] = qmi.ax;
      telemetry.acc[1] = -qmi.ay;
      telemetry.acc[2] = qmi.az;
      telemetry.imuPitchDeg = filteredPitch * RAD_TO_DEG;
      telemetry.imuRollDeg = filteredRoll * RAD_TO_DEG;

      static unsigned long lastImuLog = 0;
      if (now - lastImuLog >= 1000) {
        lastImuLog = now;
        Serial.printf("[IMU-LIVE] R:%+5.1f P:%+5.1f Y:%+5.1f | Ax:%+.2f Ay:%+.2f Az:%+.2f | Gz:%+.1f\n",
                      filteredRoll * RAD_TO_DEG, filteredPitch * RAD_TO_DEG, localYaw * RAD_TO_DEG,
                      qmi.ax, qmi.ay, qmi.az, qmi.gz);
      }
    }
  }

  // 3. Encoder Knob Navigation & Diagnostics
  int32_t delta = 0;
  portENTER_CRITICAL(&encMux);
  delta = encDelta;
  encDelta = 0;
  portEXIT_CRITICAL(&encMux);

  uint32_t curGpio = GPIO.in;
  int rawClk  = (curGpio >> ENC_CLK_PIN) & 1;
  int rawDt17 = (curGpio >> ENC_DT_PIN) & 1;
  int rawDt10 = (curGpio >> ENC_DT_ALT_PIN) & 1;
  int rawSw   = (curGpio >> ENC_SW_PIN) & 1;
  int activeDt = (rawDt17 != 0) ? rawDt17 : rawDt10;

  if (delta != 0) {
    Serial.printf("[ENC] Raw CLK:%d DT:%d Step:%d\n", rawClk, activeDt, delta);
    haptics.play(FX_TICK);
    if (ui.inMenu) {
      if (delta > 0) ui.menuNext();
      else ui.menuPrev();
    } else if (ui.view == VIEW_ATTITUDE) {
      // On the Rover 3D HUD Screen: Turn knob to smoothly rotate the 3D rover 360 degrees for inspection!
      telemetry.inspectYaw += (float)delta * 10.0f;
      while (telemetry.inspectYaw >= 360.0f) telemetry.inspectYaw -= 360.0f;
      while (telemetry.inspectYaw < 0.0f) telemetry.inspectYaw += 360.0f;
      Serial.printf("[ENC] 360 Inspection Yaw: delta=%d, inspectYaw=%.1f deg\n", delta, telemetry.inspectYaw);
    } else {
      // On other screens (IMU Air Drive, Map, Sensors): Adjust Speed Limit
      int newSpd = (int)telemetry.speedLimit + (int)(delta * 5);
      telemetry.speedLimit = constrain(newSpd, 10, 100);
      Serial.printf("[ENC] Speed Limit Adjusted: delta=%d, newSpd=%d\n", delta, telemetry.speedLimit);
    }
  }

  // Periodic encoder hardware diagnostics every 1000ms
  static unsigned long lastEncDiag = 0;
  if (now - lastEncDiag >= 1000) {
    lastEncDiag = now;
    Serial.printf("[ENC] Diagnostic: Raw CLK(16):%d DT(17):%d DT_ALT(10):%d SW(18):%d | Steps:%ld | Hits:%lu\n",
                  rawClk, rawDt17, rawDt10, rawSw, encTotalSteps, encRawHits);
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
        haptics.play(FX_DOUBLE_CLICK);
        Serial.println("[ENC] 360 Inspection Yaw Reset to 0 deg");
      } else {
        ui.nextView();
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
        if (ui.view == VIEW_DRIVE || telemetry.ctrlSource == CTRL_AIR_REMOTE) {
          telemetry.roverMode = MODE_TANK;
          haptics.play(FX_CLICK);
          Serial.println("[REMOTE] Air Controller active: Locked to Tank Mode");
        } else {
          uint8_t nextMode;
          switch (telemetry.roverMode) {
            case MODE_NORMAL: nextMode = MODE_SPORT; break;
            case MODE_SPORT:  nextMode = MODE_TANK; break;
            case MODE_TANK:   nextMode = MODE_CRAWL; break;
            case MODE_CRAWL:  nextMode = MODE_NORMAL; break;
            default:          nextMode = MODE_NORMAL; break;
          }
          telemetry.roverMode = nextMode;
          sendPacket(PKT_MODE, 0, 0, 0, 0, 0);
          haptics.play(FX_MODE_SWITCH);
          Serial.printf("[REMOTE] Mode switched via Joystick Button to: %d\n", nextMode);
        }
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
    if (ui.view == VIEW_DRIVE || telemetry.ctrlSource == CTRL_AIR_REMOTE) {
      haptics.play(FX_CLUTCH_ENGAGE);
    } else {
      haptics.play(FX_THROTTLE_BOOST);
    }
  } else if (!throttleBtn && lastThrottleState) {
    if (ui.view == VIEW_DRIVE || telemetry.ctrlSource == CTRL_AIR_REMOTE) {
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
  } else if (ui.view == VIEW_DRIVE || telemetry.ctrlSource == CTRL_AIR_REMOTE) {
    // ---------------- Fine-tuned IMU Air Tilt Motion Engine ----------------
    // Push-to-Gyrate Switch: IO8 acts as active deadman clutch switch.
    // When IO8 is NOT pressed: drive commands are clamped to 0 (cmdX=0, cmdY=0, cmdZ=0),
    // artificial horizon tilts visually on screen, status banner displays "HOLD [BOOST/IO8] TO DRIVE".
    // When IO8 IS pressed: tilt angle drives rover at high-speed tank rotation/speed,
    // status banner displays "ENGAGED - DRIVING".
    telemetry.deadmanEngaged = throttleBtn;

    if (!throttleBtn) {
      // Clutched / Stopped
      cmdX = 0;
      cmdY = 0;
      cmdZ = 0;
    } else {
      // Push-to-Gyrate clutch active!
      flags |= FLAG_TURBO;
      float pDeg = telemetry.imuPitchDeg;
      float rDeg = telemetry.imuRollDeg;

      const float DEADBAND_DEG = 3.5f;
      const float MAX_TILT_DEG = 25.0f;

      // Pitch mapping (Forward / Reverse):
      float effPitch = 0.0f;
      if (fabsf(pDeg) > DEADBAND_DEG) {
        float u = (fabsf(pDeg) - DEADBAND_DEG) / (MAX_TILT_DEG - DEADBAND_DEG);
        u = constrain(u, 0.0f, 1.0f);
        float eased = u * u * (3.0f - 2.0f * u); // SmoothStep cubic ease
        effPitch = (pDeg > 0 ? 1.0f : -1.0f) * eased;
      }

      // Roll mapping (Steering / Strafe / Tank Pivot):
      float effRoll = 0.0f;
      if (fabsf(rDeg) > DEADBAND_DEG) {
        float u = (fabsf(rDeg) - DEADBAND_DEG) / (MAX_TILT_DEG - DEADBAND_DEG);
        u = constrain(u, 0.0f, 1.0f);
        float eased = u * u * (3.0f - 2.0f * u);
        effRoll = (rDeg > 0 ? 1.0f : -1.0f) * eased;
      }

      // Scaled responsiveness from user speed limit setting
      float spdFactor = telemetry.speedLimit / 100.0f;
      int targetY = (int)roundf(effPitch * 100.0f * spdFactor);
      int targetTurn = (int)roundf(effRoll * 100.0f * spdFactor);

      if (telemetry.roverMode == MODE_TANK) {
        // High-speed Tank mode: Pitch = Fwd/Rev, Roll = High-speed pivot rotation (cmdZ)
        cmdY = constrain(targetY, -100, 100);
        cmdZ = constrain(targetTurn, -100, 100);
        cmdX = 0;
      } else {
        // Omni-directional strafe modes
        cmdY = constrain(targetY, -100, 100);
        cmdX = constrain(targetTurn, -100, 100);
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
  }

  telemetry.curX = cmdX;
  telemetry.curY = cmdY;
  telemetry.curZ = cmdZ;

  // 8. Transmit Control Packet (40Hz = every 25ms, rate-limited to protect RF buffer)
  static unsigned long lastTxTime = 0;
  if (now - lastTxTime >= 25) {
    lastTxTime = now;
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

  // 11. Render Display UI (~50-60 FPS = every 16ms)
  static unsigned long lastUiRender = 0;
  if (now - lastUiRender >= 16) {
    lastUiRender = now;
    ui.render(telemetry);
  }

  delay(1);
}

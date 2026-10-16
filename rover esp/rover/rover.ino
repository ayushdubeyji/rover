#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "rover_proto.h"
#include "dashboard.h"
#include "compass.h"
#include "fusion.h"
#include <Adafruit_VL53L0X.h>

// ---- Sensor wiring (see WIRING notes) ----
#define PIN_US_TRIG 25
#define PIN_US_ECHO 34      // via 5V->3.3V divider (1k/2k)
#define PIN_TOF1_XSHUT 26   // front-left
#define PIN_TOF2_XSHUT 27   // front-right
#define TOF1_ADDR 0x32
#define TOF2_ADDR 0x33
// Magnetometer axis remap into body frame (X fwd, Y left, Z up). Adjust after the axis test.
#ifndef MAG_MAP_X
#define MAG_MAP_X ((float)rx)
#define MAG_MAP_Y ((float)ry)
#define MAG_MAP_Z ((float)rz)
#endif

const char* ssid = "REMOVED";
const char* password = "REMOVED";

WebServer server(80);
Preferences prefs;

// IMU and Magnetometer Sensors
Adafruit_MPU6050 mpu;
RoverCompass compass;
bool imuReady = false;
bool magReady = false;
uint8_t mpuAddr = 0x68;
bool orientationSeeded = false;
float gyroZ_offset = 0.0f;
float targetHeading = 0.0f;
RoverFusion fusion;
Adafruit_VL53L0X tof1, tof2;
bool tof1Ready = false, tof2Ready = false;
float roll = 0, pitch = 0, lastAcc[3] = {0, 0, 1}, lastMag[3] = {0, 0, 0};
WiFiUDP telemUdp;
uint16_t ultraMm = 0, tofMm[2] = {0, 0};
float odomX = 0, odomY = 0;           // cm, world frame
int16_t wheelMmS[4] = {0, 0, 0, 0};   // from encoders (RP2040 "E," line)
unsigned long lastSensors = 0, lastTelemTx = 0, lastCalSave = 0;
uint16_t telemSeq = 0;
void updateOdometry(float dt);

// RP2040 motor config
int inv[4] = {0, 0, 0, 0};
int strafeInv = 0, rotInv = 0, minDuty = 25000, spinMs = 1400, accelStep = 25;
// Drive tuning
int crawlPct = 35, cruisePct = 50, accelRate = 250, jerkRate = 1500, trimF = 0, trimS = 0;

const char* MODE_NAMES[MODE_COUNT] = {"Normal", "Crawl", "Cruise", "Sport", "Field-Oriented",
                                      "Heading Lock", "Drift", "Orbit", "Tank"};
uint8_t mode = MODE_TANK;

struct Input { float x, y, z; int speed; uint8_t flags; unsigned long t; };
Input webIn = {0, 0, 0, 60, 0, 0}, nowIn = {0, 0, 0, 60, 0, 0};

struct Axis { float v, a; };
Axis ax[3] = {};
float heading = 0;

unsigned long rotEnd = 0, wheelUntil = 0;
int rotDir = 0;

int wheels[4] = {0, 0, 0, 0};
int rpMacro = 0;
unsigned long lastTelemetry = 0, lastCfgSend = 0, lastTick = 0, lastOut = 0;
String rxLine;

// ESP-NOW rx mailbox
volatile bool pktReady = false;
RoverPkt rxPkt;
uint8_t rxMac[6];
uint8_t remoteMac[6];
bool remoteKnown = false;
unsigned long lastRemote = 0;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  const uint8_t* mac = info->src_addr;
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len != sizeof(RoverPkt) || data[0] != PKT_MAGIC || pktReady) return;
  memcpy(&rxPkt, data, sizeof(RoverPkt));
  memcpy(rxMac, mac, 6);
  pktReady = true;
}

void sendCfg() {
  Serial2.printf("K,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", inv[0], inv[1], inv[2], inv[3],
                 strafeInv, rotInv, minDuty, accelStep, spinMs);
}

void loadCfg() {
  prefs.begin("rover", true);
  for (int i = 0; i < 4; i++) inv[i] = prefs.getInt(("i" + String(i)).c_str(), 0);
  strafeInv = prefs.getInt("si", 0);
  rotInv = prefs.getInt("ri", 0);
  minDuty = prefs.getInt("md", 25000);
  spinMs = prefs.getInt("sp", 1400);
  accelStep = prefs.getInt("ac", 25);
  if (accelStep < 20) accelStep = 25;
  crawlPct = prefs.getInt("cp", 35);
  cruisePct = prefs.getInt("cr", 50);
  accelRate = prefs.getInt("ar", 250);
  jerkRate = prefs.getInt("jr", 1500);
  trimF = prefs.getInt("tf", 0);
  trimS = prefs.getInt("ts", 0);
  mode = prefs.getUChar("mo", MODE_TANK) % MODE_COUNT;
  prefs.end();
}

void saveCfg() {
  prefs.begin("rover", false);
  for (int i = 0; i < 4; i++) prefs.putInt(("i" + String(i)).c_str(), inv[i]);
  prefs.putInt("si", strafeInv);
  prefs.putInt("ri", rotInv);
  prefs.putInt("md", minDuty);
  prefs.putInt("sp", spinMs);
  prefs.putInt("ac", accelStep);
  prefs.putInt("cp", crawlPct);
  prefs.putInt("cr", cruisePct);
  prefs.putInt("ar", accelRate);
  prefs.putInt("jr", jerkRate);
  prefs.putInt("tf", trimF);
  prefs.putInt("ts", trimS);
  prefs.putUChar("mo", mode);
  prefs.end();
}

// ---------------- Motion core ----------------
void emergencyStop() {
  for (auto& a : ax) a = {0, 0};
  rotEnd = 0;
  webIn.x = webIn.y = webIn.z = 0;
  nowIn.x = nowIn.y = nowIn.z = 0;
  Serial2.print("S\n");
}

void startRotate(int deg) {
  if (!deg) return;
  rotDir = deg > 0 ? 1 : -1;
  rotEnd = millis() + (unsigned long)((long)spinMs * abs(deg) / 360);
  for (auto& a : ax) a = {0, 0};
}

float expo(float v, float e) { return (1 - e) * v + e * v * v * v; }

// Jerk-limited (S-curve) profile: velocity approaches target with bounded accel and jerk.
void profile(Axis& s, float target, float amax, float jmax, float dt) {
  float err = target - s.v;
  float aDes = (err > 0 ? 1 : -1) * min(amax, sqrtf(2 * jmax * fabsf(err)));
  float da = aDes - s.a;
  float step = jmax * dt;
  s.a += constrain(da, -step, step);
  float nv = s.v + s.a * dt;
  if ((err > 0 && nv > target) || (err < 0 && nv < target) || fabsf(err) < 0.002f) { nv = target; s.a = 0; }
  s.v = nv;
}

void controlTick() {
  unsigned long now = millis();
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;
  if (dt <= 0 || dt > 0.2f) dt = 0.02f;

  float degPerSecFull = 360000.0f / spinMs;

  // 1. Read IMU & Compass, run 9-DoF fusion
  static unsigned long lastI2cRetry = 0;
  if ((!imuReady || !magReady) && (now - lastI2cRetry >= 1500)) {
    lastI2cRetry = now;
    if (!imuReady) initIMU();
    if (!magReady) initMag();
  }

  if (imuReady) {
    float gy[3] = {0, 0, 0};
    float ac[3] = {0, 0, 1};
    bool ok = readRawMPU(gy, ac);
    if (ok) {
      lastAcc[0] = ac[0]; lastAcc[1] = ac[1]; lastAcc[2] = ac[2];
      float mg[3];
      bool haveMag = false;
      if (magReady) {
        int16_t rx, ry, rz;
        if (compass.readRaw(rx, ry, rz)) {
          mg[0] = MAG_MAP_X; mg[1] = MAG_MAP_Y; mg[2] = MAG_MAP_Z;
          lastMag[0] = mg[0]; lastMag[1] = mg[1]; lastMag[2] = mg[2];
          haveMag = true;
        }
      }
      if (!orientationSeeded) {
        fusion.initOrientation(ac, haveMag ? mg : nullptr);
        orientationSeeded = true;
      }
      fusion.update(gy, ac, haveMag ? mg : nullptr, dt);
      float r, p, y;
      fusion.euler(r, p, y);
      roll = r; pitch = p; heading = y;
      updateOdometry(dt);
    } else {
      static uint8_t imuFailCount = 0;
      if (++imuFailCount > 20) {
        imuReady = false;
        fusion.gyroCal = false;
        fusion.q0 = 1.0f; fusion.q1 = 0.0f; fusion.q2 = 0.0f; fusion.q3 = 0.0f;
        imuFailCount = 0;
      }
    }
  } else if (magReady) {
    int16_t rx, ry, rz;
    if (compass.readRaw(rx, ry, rz)) {
      lastMag[0] = MAG_MAP_X;
      lastMag[1] = MAG_MAP_Y;
      lastMag[2] = MAG_MAP_Z;
      heading = atan2f(MAG_MAP_Y, MAG_MAP_X) * RAD_TO_DEG;
      float halfH = heading * DEG_TO_RAD * 0.5f;
      fusion.q0 = cosf(halfH);
      fusion.q1 = 0.0f;
      fusion.q2 = 0.0f;
      fusion.q3 = sinf(halfH);
      fusion.magCal = true;
    }
  } else {
    fusion.gyroCal = false;
  }


  if (rotEnd) {
    if ((long)(now - rotEnd) >= 0) {
      rotEnd = 0;
      Serial2.print("D,0,0,0,0\n");
    } else {
      if (!imuReady) heading += rotDir * degPerSecFull * dt;
      Serial2.printf("D,0,0,%d,100\n", rotDir * 100);
    }
    return;
  }

  if ((long)(now - wheelUntil) < 0) return;
  bool remActive = (now - nowIn.t < 400);
  Input in = remActive ? nowIn : webIn;
  bool fresh = now - in.t < 350;
  float x = fresh ? in.x : 0, y = fresh ? in.y : 0, z = fresh ? in.z : 0;
  int spd = in.speed;
  float amax = accelRate / 100.0f, jmax = jerkRate / 100.0f;

  switch (mode) {
    case MODE_NORMAL:
      // Linear and responsive across full throw (s3_remote applies user-configured expo)
      break;
    case MODE_CRAWL:
      // Controlled crawl speed with solid starting torque without cubic motor choke
      spd = max(35, crawlPct);
      amax *= 0.6f;
      break;
    case MODE_CRUISE: {
      float m = sqrtf(x * x + y * y);
      if (m > 0.05f) { x /= m; y /= m; } else { x = y = 0; }
      spd = cruisePct;
      break;
    }
    case MODE_SPORT:
      spd = 100; amax *= 2.5f; jmax *= 3.0f;
      break;
    case MODE_FIELD: {
      float r = -heading * DEG_TO_RAD;
      float fx = x * cosf(r) - y * sinf(r);
      float fy = x * sinf(r) + y * cosf(r);
      x = fx; y = fy;
      break;
    }
    case MODE_HEADING_LOCK:
      if (fabsf(z) > 0.08f) {
        targetHeading = heading;
      } else {
        float hErr = targetHeading - heading;
        if (hErr > 180.0f) hErr -= 360.0f;
        if (hErr < -180.0f) hErr += 360.0f;
        z = constrain(hErr * 0.035f, -0.7f, 0.7f);
      }
      break;
    case MODE_DRIFT:
      x += -0.6f * z * fabsf(y);
      break;
    case MODE_ORBIT:
      z = -0.7f * x;
      break;
    case MODE_TANK:
      if (fabsf(x) > fabsf(z)) z = x;
      x = 0;
      break;
  }
  if (mode != MODE_HEADING_LOCK) z += (trimF * y + trimS * x) / 100.0f;

  if (in.flags & FLAG_TURBO) {
    spd = 100;
    amax *= 4.0f;
    jmax *= 5.0f;
  }
  if (in.flags & FLAG_CRAWL) spd = min(spd, crawlPct);
  spd = constrain(spd, 0, 100);

  // Fast, low-latency response for remote input while preserving smooth transitions
  float effAmax = max(amax, 8.0f);
  float effJmax = max(jmax, 45.0f);
  float tgt[3] = {constrain(x, -1.0f, 1.0f), constrain(y, -1.0f, 1.0f), constrain(z, -1.0f, 1.0f)};
  for (int i = 0; i < 3; i++) profile(ax[i], tgt[i], effAmax, effJmax, dt);

  if (!imuReady) {
    heading += ax[2].v * (spd / 100.0f) * degPerSecFull * dt;
    if (heading > 180) heading -= 360;
    if (heading < -180) heading += 360;
  }

  bool moving = fabsf(ax[0].v) > 0.01f || fabsf(ax[1].v) > 0.01f || fabsf(ax[2].v) > 0.01f;
  static bool wasMoving = false;
  if (moving || wasMoving || now - lastOut > 100) {
    lastOut = now;
    wasMoving = moving;
    Serial2.printf("D,%d,%d,%d,%d\n", (int)lroundf(ax[0].v * 100), (int)lroundf(ax[1].v * 100),
                   (int)lroundf(ax[2].v * 100), moving ? spd : 0);
  }
}

// ---------------- ESP-NOW ----------------
void handlePacket() {
  if (!pktReady) return;
  RoverPkt p = rxPkt;
  uint8_t mac[6];
  memcpy(mac, rxMac, 6);
  pktReady = false;

  if (!esp_now_is_peer_exist(mac)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peer);
  }
  memcpy(remoteMac, mac, 6);
  remoteKnown = true;
  lastRemote = millis();

  switch (p.type) {
    case PKT_DRIVE:
      mode = p.mode % MODE_COUNT;
      nowIn = {p.x / 100.0f, p.y / 100.0f, p.z / 100.0f, p.speed, p.flags, millis()};
      break;
    case PKT_STOP: emergencyStop(); break;
    case PKT_MODE: mode = p.mode % MODE_COUNT; break;
    case PKT_ROTATE: startRotate(p.arg); break;
    case PKT_CALIB:
      if (p.arg == CAL_LEVEL) fusion.calibrateLevel(prefs);
      else if (p.arg == CAL_MAG_RESET) fusion.resetMagCal();
      else if (p.arg == CAL_SAVE) fusion.save(prefs);
      else if (p.arg == CAL_ZERO_YAW) fusion.zeroYaw();
      else if (p.arg == CAL_GYRO) { fusion.gyroCal = false; }
      break;
    case PKT_HEADING_RESET: targetHeading = heading = 0; fusion.zeroYaw(); break;
  }

  RoverPkt ack = {};
  ack.magic = PKT_MAGIC;
  ack.type = PKT_ACK;
  ack.mode = mode;
  ack.speed = p.speed;
  ack.seq = p.seq;
  ack.arg = (int16_t)heading;
  esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
}

// ---------------- HTTP ----------------
void ok() { server.send(200, "text/plain", "OK"); }

void handleDrive() {
  webIn = {constrain(server.arg("x").toInt(), -100, 100) / 100.0f,
           constrain(server.arg("y").toInt(), -100, 100) / 100.0f,
           constrain(server.arg("z").toInt(), -100, 100) / 100.0f,
           (int)constrain(server.arg("s").toInt(), 0, 100),
           (uint8_t)(server.arg("t").toInt() ? FLAG_TURBO : 0), millis()};
  ok();
}

void handleWheel() {
  Serial2.printf("W,%d,%d\n", constrain(server.arg("i").toInt(), 0, 3), constrain(server.arg("v").toInt(), -100, 100));
  webIn.t = 0;
  wheelUntil = millis() + 250;
  ok();
}

void handleMacro() {
  Serial2.printf("M,%d,%d\n", server.arg("id").toInt(), server.arg("d").toInt());
  ok();
}

void handleMode() {
  if (server.hasArg("m")) mode = server.arg("m").toInt() % MODE_COUNT;
  if (server.hasArg("cp")) crawlPct = constrain(server.arg("cp").toInt(), 5, 60);
  if (server.hasArg("cr")) cruisePct = constrain(server.arg("cr").toInt(), 5, 100);
  if (server.hasArg("ar")) accelRate = constrain(server.arg("ar").toInt(), 20, 2000);
  if (server.hasArg("jr")) jerkRate = constrain(server.arg("jr").toInt(), 50, 10000);
  if (server.hasArg("tf")) trimF = constrain(server.arg("tf").toInt(), -30, 30);
  if (server.hasArg("ts")) trimS = constrain(server.arg("ts").toInt(), -30, 30);
  if (server.hasArg("hr")) { targetHeading = heading = 0; fusion.zeroYaw(); }
  if (server.hasArg("save")) saveCfg();
  String j = "{\"m\":" + String(mode) + ",\"cp\":" + crawlPct + ",\"cr\":" + cruisePct + ",\"ar\":" + accelRate +
             ",\"jr\":" + jerkRate + ",\"tf\":" + trimF + ",\"ts\":" + trimS + ",\"names\":[";
  for (int i = 0; i < MODE_COUNT; i++) j += String(i ? "," : "") + "\"" + MODE_NAMES[i] + "\"";
  server.send(200, "application/json", j + "]}");
}

void handleCfg() {
  if (server.hasArg("i0")) {
    for (int i = 0; i < 4; i++) inv[i] = server.arg("i" + String(i)).toInt();
    strafeInv = server.arg("si").toInt();
    rotInv = server.arg("ri").toInt();
    minDuty = constrain(server.arg("md").toInt(), 0, 60000);
    spinMs = constrain(server.arg("sp").toInt(), 200, 10000);
    accelStep = constrain(server.arg("ac").toInt(), 1, 100);
    saveCfg();
    sendCfg();
  }
  String j = "{\"i\":[" + String(inv[0]) + "," + String(inv[1]) + "," + String(inv[2]) + "," + String(inv[3]) +
             "],\"si\":" + strafeInv + ",\"ri\":" + rotInv + ",\"md\":" + minDuty + ",\"sp\":" + spinMs +
             ",\"ac\":" + accelStep + "}";
  server.send(200, "application/json", j);
}

void handleStatus() {
  bool link = millis() - lastTelemetry < 1000;
  bool rem = remoteKnown && millis() - lastRemote < 1000;
  String j = "{\"link\":" + String(link ? 1 : 0) + ",\"rem\":" + (rem ? 1 : 0) + ",\"w\":[" + wheels[0] + "," +
             wheels[1] + "," + wheels[2] + "," + wheels[3] + "],\"macro\":" + (rpMacro || rotEnd ? 1 : 0) +
             ",\"mode\":" + mode + ",\"hd\":" + (int)heading + ",\"spd\":" + nowIn.speed + ",\"rssi\":" + WiFi.RSSI() +
             ",\"ch\":" + WiFi.channel() + ",\"imu\":" + (imuReady ? 1 : 0) + ",\"mag\":" + (magReady ? 1 : 0) +
             ",\"ip\":\"" +
             (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + "\"}";
  server.send(200, "application/json", j);
}

void handleTelem() {
  char b[480];
  snprintf(b, sizeof(b),
    "{\"q\":[%.4f,%.4f,%.4f,%.4f],\"g\":[%.4f,%.4f,%.4f],\"a\":[%.4f,%.4f,%.4f],\"m\":[%.2f,%.2f,%.2f],"
    "\"us\":%u,\"tof\":[%u,%u],\"w\":[%d,%d,%d,%d],\"pos\":[%.1f,%.1f],\"hd\":%.2f,\"r\":%.2f,\"p\":%.2f,"
    "\"cal\":%u,\"mc\":%.2f,\"t\":%lu}",
    fusion.q0, fusion.q1, fusion.q2, fusion.q3,
    fusion.gyroRaw[0] - fusion.gyroBias[0], fusion.gyroRaw[1] - fusion.gyroBias[1], fusion.gyroRaw[2] - fusion.gyroBias[2],
    lastAcc[0] * 9.80665f, lastAcc[1] * 9.80665f, lastAcc[2] * 9.80665f,
    lastMag[0], lastMag[1], lastMag[2],
    ultraMm, tofMm[0], tofMm[1],
    wheelMmS[0], wheelMmS[1], wheelMmS[2], wheelMmS[3],
    odomX, odomY, heading, roll, pitch,
    (fusion.gyroCal ? 1 : 0) | (fusion.magCal ? 2 : 0) | (fusion.levelCal ? 4 : 0) | (fusion.stationary ? 8 : 0),
    fusion.magCoverage, millis());
  server.send(200, "application/json", b);
}

void readTelemetry() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      if (rxLine.startsWith("E,")) {
        int n = 0, pos = 2;
        while (n < 4 && pos < (int)rxLine.length()) {
          int comma = rxLine.indexOf(',', pos);
          if (comma < 0) comma = rxLine.length();
          wheelMmS[n++] = rxLine.substring(pos, comma).toInt();
          pos = comma + 1;
        }
      }
      if (rxLine.startsWith("T,")) {
        int v[5] = {0, 0, 0, 0, 0}, n = 0, pos = 2;
        while (n < 5 && pos < (int)rxLine.length()) {
          int comma = rxLine.indexOf(',', pos);
          if (comma < 0) comma = rxLine.length();
          v[n++] = rxLine.substring(pos, comma).toInt();
          pos = comma + 1;
        }
        for (int i = 0; i < 4; i++) wheels[i] = v[i];
        rpMacro = v[4];
        lastTelemetry = millis();
      }
      rxLine = "";
    } else if (c != '\r' && rxLine.length() < 64) {
      rxLine += c;
    }
  }
}

void readRangeSensors() {
  if (millis() - lastSensors < 60) return;
  lastSensors = millis();
  digitalWrite(PIN_US_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, 12000);
  if (us > 0) ultraMm = (uint16_t)(us * 0.1715f);
  else ultraMm = 0;
  VL53L0X_RangingMeasurementData_t m;
  if (tof1Ready) { tof1.rangingTest(&m, false); tofMm[0] = (m.RangeStatus != 4) ? m.RangeMilliMeter : 0; }
  if (tof2Ready) { tof2.rangingTest(&m, false); tofMm[1] = (m.RangeStatus != 4) ? m.RangeMilliMeter : 0; }
}

void updateOdometry(float dt) {
  float vx = (wheelMmS[0] + wheelMmS[1] + wheelMmS[2] + wheelMmS[3]) / 4.0f;
  float vy = (-wheelMmS[0] + wheelMmS[1] + wheelMmS[2] - wheelMmS[3]) / 4.0f;
  float th = heading * DEG_TO_RAD;
  odomX += (vx * cosf(th) - vy * sinf(th)) * dt / 10.0f;
  odomY += (vx * sinf(th) + vy * cosf(th)) * dt / 10.0f;
}

void sendTelemetry() {
  if (millis() - lastTelemTx < 50) return;
  lastTelemTx = millis();

  if (remoteKnown) {
    TelemPkt t = {};
    t.magic = PKT_MAGIC; t.type = PKT_TELEM;
    t.q[0] = (int16_t)(fusion.q0 * 16384); t.q[1] = (int16_t)(fusion.q1 * 16384);
    t.q[2] = (int16_t)(fusion.q2 * 16384); t.q[3] = (int16_t)(fusion.q3 * 16384);
    for (int i = 0; i < 3; i++) {
      t.gyro[i] = (int16_t)((fusion.gyroRaw[i] - fusion.gyroBias[i]) * RAD_TO_DEG * 16);
      t.acc[i] = (int16_t)(lastAcc[i] * 1000);
    }
    t.ultraMm = ultraMm; t.tofMm[0] = tofMm[0]; t.tofMm[1] = tofMm[1];
    for (int i = 0; i < 4; i++) t.wheelMmS[i] = wheelMmS[i];
    t.posCm[0] = (int16_t)odomX; t.posCm[1] = (int16_t)odomY;
    t.calFlags = ((imuReady && fusion.gyroCal) ? CAL_GYRO_OK : 0) | ((magReady && fusion.magCal) ? CAL_MAG_OK : 0) |
                 (fusion.levelCal ? CAL_LEVEL_OK : 0) | (fusion.stationary ? CAL_STILL : 0);
    t.magCoverage = (uint8_t)(fusion.magCoverage * 100);
    t.seq = ++telemSeq;
    esp_now_send(remoteMac, (uint8_t*)&t, sizeof(t));
  }

  if (WiFi.status() == WL_CONNECTED) {
    char b[384];
    snprintf(b, sizeof(b),
      "{\"q\":[%.4f,%.4f,%.4f,%.4f],\"g\":[%.4f,%.4f,%.4f],\"a\":[%.4f,%.4f,%.4f],\"m\":[%.2f,%.2f,%.2f],"
      "\"us\":%u,\"tof\":[%u,%u],\"w\":[%d,%d,%d,%d],\"pos\":[%.1f,%.1f],\"hd\":%.2f,\"r\":%.2f,\"p\":%.2f}",
      fusion.q0, fusion.q1, fusion.q2, fusion.q3,
      fusion.gyroRaw[0] - fusion.gyroBias[0], fusion.gyroRaw[1] - fusion.gyroBias[1], fusion.gyroRaw[2] - fusion.gyroBias[2],
      lastAcc[0] * 9.80665f, lastAcc[1] * 9.80665f, lastAcc[2] * 9.80665f,
      lastMag[0], lastMag[1], lastMag[2],
      ultraMm, tofMm[0], tofMm[1],
      wheelMmS[0], wheelMmS[1], wheelMmS[2], wheelMmS[3],
      odomX, odomY, heading, roll, pitch);
    telemUdp.beginPacket(IPAddress(192, 168, 0, 111), 8888);
    telemUdp.write((const uint8_t*)b, strlen(b));
    telemUdp.endPacket();
  }
}

bool readRawMPU(float g[3], float a[3]) {
  if (!imuReady) return false;
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddr, 14) != 14) return false;
  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read(); // Skip temp
  int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());

  // Check for disconnected bus signature (all 0x00 or all 0xFF)
  if ((ax == 0 && ay == 0 && az == 0) || (ax == -1 && ay == -1 && az == -1)) return false;

  // Accel +/- 4g (8192 LSB/g) -> g
  a[0] = (float)ax / 8192.0f;
  a[1] = (float)ay / 8192.0f;
  a[2] = (float)az / 8192.0f;

  // Gyro +/- 500 deg/s (65.5 LSB/(deg/s)) -> rad/s
  g[0] = ((float)gx / 65.5f) * DEG_TO_RAD;
  g[1] = ((float)gy / 65.5f) * DEG_TO_RAD;
  g[2] = ((float)gz / 65.5f) * DEG_TO_RAD;
  return true;
}

bool initIMU() {
  if (imuReady) return true;
  uint8_t addrs[2] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    uint8_t addr = addrs[i];
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue; // Strict I2C ACK check
    Wire.beginTransmission(addr);
    Wire.write(0x75); // WHO_AM_I
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)addr, 1) == 1) {
      uint8_t who = Wire.read();
      if (who == 0x00 || who == 0xFF) continue;
      Serial.printf("[IMU] Detected MPU/ICM at 0x%02X (WHO_AM_I=0x%02X)\n", addr, who);
      mpuAddr = addr;
      
      // Wake up MPU & auto-select best clock source
      Wire.beginTransmission(addr); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission();
      delay(20);
      // Sample rate divider = 0
      Wire.beginTransmission(addr); Wire.write(0x19); Wire.write(0x00); Wire.endTransmission();
      // DLPF = 42Hz
      Wire.beginTransmission(addr); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
      // Gyro +/- 500 deg/s
      Wire.beginTransmission(addr); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
      // Accel +/- 4g
      Wire.beginTransmission(addr); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();
      
      // CRITICAL: Enable I2C Bypass mode to expose auxiliary compass to main I2C bus!
      Wire.beginTransmission(addr); Wire.write(0x6A); Wire.write(0x00); Wire.endTransmission(); // USER_CTRL: disable master mode
      Wire.beginTransmission(addr); Wire.write(0x37); Wire.write(0x02); Wire.endTransmission(); // INT_PIN_CFG: BYPASS_EN = 1
      delay(10);

      imuReady = true;
      float testG[3], testA[3];
      if (readRawMPU(testG, testA)) {
        float sumGz = 0;
        for (int k = 0; k < 30; k++) {
          if (readRawMPU(testG, testA)) sumGz += testG[2];
          delay(4);
        }
        gyroZ_offset = sumGz / 30.0f;
        Serial.printf("[IMU] Initialized successfully. GyroZ offset: %.4f\n", gyroZ_offset);
        return true;
      }
      imuReady = false;
    }
  }
  return false;
}

bool initMag() {
  if (magReady) return true;
  // Ensure bypass is active on MPU before probing compass
  if (imuReady) {
    Wire.beginTransmission(mpuAddr); Wire.write(0x6A); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(mpuAddr); Wire.write(0x37); Wire.write(0x02); Wire.endTransmission();
    delay(5);
  }
  if (compass.begin()) {
    magReady = true;
    const char *cname = "Unknown";
    if (compass.chip == MAG_QMC5883L) cname = "QMC5883L (0x0D)";
    else if (compass.chip == MAG_HMC5883L) cname = "HMC5883L (0x1E)";
    else if (compass.chip == MAG_AK8963)   cname = "AK8963/MPU9250 (0x0C)";
    else if (compass.chip == MAG_LIS3MDL)  cname = "LIS3MDL (0x1C)";
    else if (compass.chip == MAG_IST8310)  cname = "IST8310 (0x0E)";
    else if (compass.chip == MAG_MMC5603)  cname = "MMC5603 (0x30)";
    Serial.printf("[MAG] Magnetometer ready: %s\n", cname);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  loadCfg();

  // Hardware I2C Bus recovery sequence to clear stuck slave clock-stretching
  pinMode(19, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  for (int i = 0; i < 16; i++) {
    pinMode(22, OUTPUT); digitalWrite(22, LOW); delayMicroseconds(5);
    pinMode(22, INPUT_PULLUP); delayMicroseconds(5);
  }

  // Probe Candidate I2C Pin Pairs for MPU6050 and Magnetometer:
  // Pair 0: (SDA=19, SCL=22), Pair 1: (SDA=21, SCL=22), Pair 2: (SDA=22, SCL=19)
  const int candidatePairs[3][2] = {{19, 22}, {21, 22}, {22, 19}};
  int activeSda = 19, activeScl = 22;
  for (int p = 0; p < 3; p++) {
    int sdaP = candidatePairs[p][0];
    int sclP = candidatePairs[p][1];
    Wire.end();
    pinMode(sdaP, INPUT_PULLUP);
    pinMode(sclP, INPUT_PULLUP);
    for (int i = 0; i < 9; i++) {
      pinMode(sclP, OUTPUT); digitalWrite(sclP, LOW); delayMicroseconds(5);
      pinMode(sclP, INPUT_PULLUP); delayMicroseconds(5);
    }
    Wire.begin(sdaP, sclP, 100000);
    Wire.setTimeOut(30);
    if (initIMU() || initMag()) {
      activeSda = sdaP; activeScl = sclP;
      Serial.printf("[I2C] Locked onto active bus: SDA=%d, SCL=%d (IMU=%d, MAG=%d)\n", sdaP, sclP, imuReady, magReady);
      break;
    }
  }
  if (!imuReady && !magReady) {
    // Keep default bus active
    Wire.end();
    Wire.begin(activeSda, activeScl, 100000);
    Wire.setTimeOut(30);
  }

  fusion.load(prefs);
  pinMode(PIN_US_TRIG, OUTPUT); pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_TOF1_XSHUT, OUTPUT); pinMode(PIN_TOF2_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF1_XSHUT, LOW); digitalWrite(PIN_TOF2_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_TOF1_XSHUT, HIGH); delay(10);
  tof1Ready = tof1.begin(TOF1_ADDR, false, &Wire);
  digitalWrite(PIN_TOF2_XSHUT, HIGH); delay(10);
  tof2Ready = tof2.begin(TOF2_ADDR, false, &Wire);
  Serial.printf("[TOF] L=%d R=%d\n", tof1Ready, tof2Ready);

  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname("Mecanum-Rover");
  IPAddress local_IP(192, 168, 0, 101);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(192, 168, 0, 1);
  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(ssid, password);
  for (int r = 0; WiFi.status() != WL_CONNECTED && r < 40; r++) delay(500);
  if (WiFi.status() != WL_CONNECTED) WiFi.softAP("Mecanum-Rover", "REMOVED");
  WiFi.setSleep(false);
  Serial.printf("MAC %s  channel %d  IP %s\n", WiFi.macAddress().c_str(), WiFi.channel(), WiFi.localIP().toString().c_str());
  telemUdp.begin(8888);

  esp_now_init();
  esp_now_register_recv_cb(onRecv);

  ArduinoOTA.setHostname("Mecanum-Rover");
  ArduinoOTA.begin();

  server.on("/", []() { server.send_P(200, "text/html", DASHBOARD_HTML); });
  server.on("/d", handleDrive);
  server.on("/s", []() { emergencyStop(); ok(); });
  server.on("/w", handleWheel);
  server.on("/m", handleMacro);
  server.on("/rot", []() { startRotate(server.arg("deg").toInt()); ok(); });
  server.on("/mode", handleMode);
  server.on("/cfg", handleCfg);
  server.on("/status", handleStatus);
  server.on("/telem", handleTelem);
  server.on("/calib", []() {
    String c = server.arg("c");
    if (c == "level") fusion.calibrateLevel(prefs);
    else if (c == "magreset") fusion.resetMagCal();
    else if (c == "save") fusion.save(prefs);
    else if (c == "zero") fusion.zeroYaw();
    ok();
  });
  server.on("/i2c", []() {
    int sda_query = server.hasArg("sda") ? server.arg("sda").toInt() : 19;
    int scl_query = server.hasArg("scl") ? server.arg("scl").toInt() : 22;
    if (server.hasArg("sda")) {
      Wire.end();
      pinMode(scl_query, INPUT_PULLUP);
      pinMode(sda_query, INPUT_PULLUP);
      for (int i = 0; i < 9; i++) {
        pinMode(scl_query, OUTPUT); digitalWrite(scl_query, LOW); delayMicroseconds(5);
        pinMode(scl_query, INPUT_PULLUP); delayMicroseconds(5);
      }
      Wire.begin(sda_query, scl_query, 100000);
      Wire.setTimeOut(30);
      initIMU();
      initMag();
    }
    String s = "{\"sda\":" + String(sda_query) + ",\"scl\":" + String(scl_query) + ",\"devices\":[";
    bool first = true;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) {
        if (!first) s += ",";
        first = false;
        char buf[16];
        snprintf(buf, sizeof(buf), "\"0x%02X\"", a);
        s += buf;
      }
    }
    char extra[80];
    snprintf(extra, sizeof(extra), "],\"imu\":%d,\"mag\":%d,\"mpu\":\"0x%02X\",\"chip\":%d}", imuReady ? 1 : 0, magReady ? 1 : 0, mpuAddr, (int)compass.chip);
    s += extra;
    server.send(200, "application/json", s);
  });
  server.begin();
  sendCfg();
  lastTick = millis();
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  readTelemetry();
  handlePacket();
  readRangeSensors();
  sendTelemetry();
  if (fusion.gyroCal && fusion.magCal && millis() - lastCalSave > 120000) { lastCalSave = millis(); fusion.save(prefs); }
  if (millis() - lastTick >= 20) controlTick();
  if (millis() - lastCfgSend > 3000) {
    lastCfgSend = millis();
    sendCfg();
  }
  static unsigned long lastWifiRetry = 0;
  if (WiFi.status() != WL_CONNECTED && (millis() - lastWifiRetry > 5000)) {
    lastWifiRetry = millis();
    WiFi.reconnect();
  }
}

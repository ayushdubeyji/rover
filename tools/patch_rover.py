import re
p = r'C:\Users\dell\Desktop\rover\rover esp\rover\rover.ino'
s = open(p, encoding='utf-8').read()

def rep(a, b, cnt=1):
    global s
    assert a in s, a[:50]
    s = s.replace(a, b, cnt)

rep('#include "compass.h"\n', '''#include "compass.h"
#include "fusion.h"
#include <Adafruit_VL53L0X.h>

// ---- Sensor wiring (see WIRING notes) ----
#define PIN_US_TRIG 25
#define PIN_US_ECHO 34      // via 5V->3.3V divider (1k/2k)
#define PIN_TOF1_XSHUT 26   // front-left
#define PIN_TOF2_XSHUT 27   // front-right
#define TOF1_ADDR 0x30
#define TOF2_ADDR 0x31
// Magnetometer axis remap into body frame (X fwd, Y left, Z up). Adjust after the axis test.
#ifndef MAG_MAP_X
#define MAG_MAP_X ((float)rx)
#define MAG_MAP_Y ((float)ry)
#define MAG_MAP_Z ((float)rz)
#endif
''')

rep('float targetHeading = 0.0f;\n', '''float targetHeading = 0.0f;
RoverFusion fusion;
Adafruit_VL53L0X tof1, tof2;
bool tof1Ready = false, tof2Ready = false;
float roll = 0, pitch = 0, lastAcc[3] = {0, 0, 1};
uint16_t ultraMm = 0, tofMm[2] = {0, 0};
float odomX = 0, odomY = 0;           // cm, world frame
int16_t wheelMmS[4] = {0, 0, 0, 0};   // from encoders (RP2040 "E," line)
unsigned long lastSensors = 0, lastTelemTx = 0, lastCalSave = 0;
uint16_t telemSeq = 0;
void updateOdometry(float dt);
''')

rep('roll = r; pitch = p; heading = y;', 'roll = r; pitch = p; heading = y;\n      updateOdometry(dt);')
rep('targetHeading = heading = 0; break;', 'targetHeading = heading = 0; fusion.zeroYaw(); break;')
rep('if (server.hasArg("hr")) targetHeading = heading = 0;', 'if (server.hasArg("hr")) { targetHeading = heading = 0; fusion.zeroYaw(); }')

# ESP-NOW rx: accept CALIB packets (same RoverPkt size, new type) - handled in switch
rep('    case PKT_ROTATE: startRotate(p.arg); break;', '''    case PKT_ROTATE: startRotate(p.arg); break;
    case PKT_CALIB:
      if (p.arg == CAL_LEVEL) fusion.calibrateLevel(prefs);
      else if (p.arg == CAL_MAG_RESET) fusion.resetMagCal();
      else if (p.arg == CAL_SAVE) fusion.save(prefs);
      else if (p.arg == CAL_ZERO_YAW) fusion.zeroYaw();
      else if (p.arg == CAL_GYRO) { fusion.gyroCal = false; }
      break;''')

# telemetry + sensors functions before setup()
rep('void setup() {\n', '''void readRangeSensors() {
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
  if (!remoteKnown || millis() - lastTelemTx < 50) return;
  lastTelemTx = millis();
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
  t.calFlags = (fusion.gyroCal ? CAL_GYRO_OK : 0) | (fusion.magCal ? CAL_MAG_OK : 0) |
               (fusion.levelCal ? CAL_LEVEL_OK : 0) | (fusion.stationary ? CAL_STILL : 0);
  t.magCoverage = (uint8_t)(fusion.magCoverage * 100);
  t.seq = ++telemSeq;
  esp_now_send(remoteMac, (uint8_t*)&t, sizeof(t));
}

void setup() {
''')

# setup: fusion load + range sensors, after compass init block
rep('  WiFi.mode(WIFI_AP_STA);', '''  fusion.load(prefs);
  pinMode(PIN_US_TRIG, OUTPUT); pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_TOF1_XSHUT, OUTPUT); pinMode(PIN_TOF2_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF1_XSHUT, LOW); digitalWrite(PIN_TOF2_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_TOF1_XSHUT, HIGH); delay(10);
  tof1Ready = tof1.begin(TOF1_ADDR, false, &Wire);
  digitalWrite(PIN_TOF2_XSHUT, HIGH); delay(10);
  tof2Ready = tof2.begin(TOF2_ADDR, false, &Wire);
  Serial.printf("[TOF] L=%d R=%d\\n", tof1Ready, tof2Ready);

  WiFi.mode(WIFI_AP_STA);''')

rep('  handlePacket();\n', '''  handlePacket();
  readRangeSensors();
  sendTelemetry();
  if (fusion.gyroCal && fusion.magCal && millis() - lastCalSave > 120000) { lastCalSave = millis(); fusion.save(prefs); }
''')

# encoder line from RP2040: "E,fl,fr,rl,rr" mm/s
rep('      if (rxLine.startsWith("T,")) {', '''      if (rxLine.startsWith("E,")) {
        int n = 0, pos = 2;
        while (n < 4 && pos < (int)rxLine.length()) {
          int comma = rxLine.indexOf(',', pos);
          if (comma < 0) comma = rxLine.length();
          wheelMmS[n++] = rxLine.substring(pos, comma).toInt();
          pos = comma + 1;
        }
      }
      if (rxLine.startsWith("T,")) {''')
open(p, 'w', encoding='utf-8').write(s)
print('patched')


#include "display_ui.h"

#define C_BG      0x0000
#define C_CARD    0x18E3
#define C_BORDER  0x3186
#define C_MUTED   0x7BEF
#define C_TEXT    0xFFFF
#define C_GREEN   0x07E0
#define C_CYAN    0x07FF
#define C_ORANGE  0xFD20
#define C_RED     0xF800
#define C_YELLOW  0xFFE0
#define C_BLUE    0x3CD7

#define CW 240
#define CH 156
#define STRIP_Y 156

static const char *MODE_NAMES_REMOTE[MODE_COUNT] = {"NORMAL", "CRAWL", "CRUISE", "SPORT", "FIELD", "HEAD-LOCK", "DRIFT", "ORBIT", "TANK", "AUTO"};
static const char *AIR_AXIS_NAMES[6] = {"Std (P/R)", "Swap (R/P)", "Inv Pitch", "Inv Roll", "Inv Both", "Swap Inv"};
static const char *MENU_ITEMS[DisplayUI::MENU_ITEM_COUNT] = {
  "Drive Mode", "Semi-Auto Mode", "Auto Speed", "Crash Protect", "Control Input", "Air Gyro Axis", "Speed Limit", "Deadzone", "Expo Curve",
  "Trim Fwd/Back", "Trim Strafe", "Trim Yaw", "Gyro Smooth", "Brightness",
  "Haptics", "Haptic Str",
  "Enc Mode", "Enc Repeat", "Enc DT Pin", "Enc Sens",
  "Cal Joystick", "Cal Air Tilt", "Level Rover", "Zero Yaw", "Save Settings", "Exit"
};

void DisplayUI::begin() {
  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
  pinMode(LCD_RST_PIN, OUTPUT);
  digitalWrite(LCD_RST_PIN, HIGH); delay(20);
  digitalWrite(LCD_RST_PIN, LOW);  delay(60);
  digitalWrite(LCD_RST_PIN, HIGH); delay(120);
  pinMode(LCD_DC_PIN, OUTPUT);
  digitalWrite(LCD_DC_PIN, HIGH);

  bus = new Arduino_ESP32SPI(LCD_DC_PIN, GFX_NOT_DEFINED, LCD_SCLK_PIN, LCD_MOSI_PIN);
  // Rotation 2 (180 deg): damaged top 8 mm becomes the bottom info strip. 240x240 needs row offset 80 on rot 2/3.
  tft = new Arduino_ST7789(bus, LCD_RST_PIN, 2, true, LCD_WIDTH, LCD_HEIGHT, 0, 0, 0, 80);
  tft->begin(80000000);
  tft->invertDisplay(false);
  tft->fillScreen(C_BG);

  canvas = new Arduino_Canvas(CW, CH, tft, 0, 0);
  canvas->begin(GFX_SKIP_OUTPUT_BEGIN);

  tft->setTextColor(C_GREEN); tft->setTextSize(2);
  tft->setCursor(40, 60); tft->print("S3 ROVER HUD");
  tft->setTextSize(1); tft->setTextColor(C_TEXT);
  tft->setCursor(70, 90); tft->print("booting...");
  delay(500);
  tft->fillScreen(C_BG);
  setBacklight(100);
}

void DisplayUI::setBacklight(uint8_t percent) {
  analogWrite(LCD_BL_PIN, constrain(percent, 5, 100) * 255 / 100);
}

void DisplayUI::bar(int x, int y, int w, int h, int val, int maxv, uint16_t col) {
  canvas->drawRect(x, y, w, h, C_BORDER);
  int f = constrain((long)val * (w - 2) / maxv, 0, w - 2);
  if (f > 0) canvas->fillRect(x + 1, y + 1, f, h - 2, col);
}

void DisplayUI::drawHeader(const RemoteTelemetry &tm, const char *title) {
  canvas->fillRect(0, 0, CW, 14, C_BG);
  canvas->drawFastHLine(0, 14, CW, C_BORDER);
  canvas->setTextSize(1);
  canvas->setTextColor(C_CYAN);
  canvas->setCursor(4, 3); canvas->print(title);

  // Persistent navigation hint visible across all views
  canvas->setTextColor(C_MUTED);
  canvas->setCursor(68, 3);
  canvas->print(inMenu ? "HOLD:EXIT" : "HOLD:MENU");

  uint16_t lc = tm.linked ? C_GREEN : C_RED;
  canvas->fillCircle(150, 7, 3, lc);
  canvas->setTextColor(lc);
  canvas->setCursor(157, 3); canvas->print(tm.linked ? "LINK" : "OFF");
  canvas->setTextColor(tm.vbatPct < 20 ? C_RED : C_TEXT);
  canvas->setCursor(194, 3); canvas->printf("%.1fV", tm.vbat);
}

void DisplayUI::drawAttitude(const RemoteTelemetry &tm) {
  drawHeader(tm, "ATTITUDE HUD");

  // Smooth decoupled Euler angles so pitch/roll never gimbal-lock or invert when yaw rotates
  static float sRoll = 0.0f, sPitch = 0.0f, sYaw = 0.0f;
  float targetRoll = constrain(tm.imuRollDeg, -65.0f, 65.0f);
  float targetPitch = constrain(tm.imuPitchDeg, -65.0f, 65.0f);
  float targetYaw = fmodf(tm.imuYawDeg + tm.inspectYaw + 3600.0f, 360.0f);


  sRoll += (targetRoll - sRoll) * 0.55f;
  sPitch += (targetPitch - sPitch) * 0.55f;
  float dYaw = targetYaw - sYaw;
  while (dYaw > 180.0f) dYaw -= 360.0f;
  while (dYaw < -180.0f) dYaw += 360.0f;
  sYaw = fmodf(sYaw + dYaw * 0.55f + 360.0f, 360.0f);

  float roll = sRoll;
  float pitch = sPitch;
  float yaw = sYaw;

  // Viewer-frame quaternion: Q = Qx(roll) * Qy(pitch) * Qz(-yaw)
  // First rotates car on ground plane to clockwise compass heading (-yaw around +Z),
  // then tilts in screen viewer frame (+pitch = nose/top down, +roll = right side down)
  float rRad = roll * DEG_TO_RAD * 0.5f;
  float pRad = pitch * DEG_TO_RAD * 0.5f;
  float zRad = -yaw * DEG_TO_RAD * 0.5f;
  float cr = cosf(rRad), sr = sinf(rRad);
  float cp = cosf(pRad), sp = sinf(pRad);
  float cz = cosf(zRad), sz = sinf(zRad);

  Quat sRot;
  sRot.w = cr * cp * cz - sr * sp * sz;
  sRot.x = sr * cp * cz + cr * sp * sz;
  sRot.y = cr * sp * cz - sr * cp * sz;
  sRot.z = sr * sp * cz + cr * cp * sz;

  // Render 3D Solid or PBR digital twin
  if (tm.renderMode == 1) {
    car.draw(canvas, 120, 76, 76.0f, sRot);
  } else {
    car.drawPBR(canvas, 120, 76, 360.0f - yaw, roll, pitch);
  }

  // Tactical Compass Reticle & Cardinal Directions (N, E, S, W) around 3D Rover
  const int cx = 120, cy = 76;
  canvas->drawCircle(cx, cy, 62, 0x18E3);
  canvas->drawCircle(cx, cy, 61, 0x10A2);

  // Cardinal Directions:
  // North (0 deg / Top)
  canvas->drawFastVLine(cx, 13, 6, C_GREEN);
  canvas->setTextColor(C_GREEN); canvas->setTextSize(1);
  canvas->setCursor(cx - 3, 21); canvas->print("N");

  // East (90 deg / Right)
  canvas->drawFastHLine(cx + 56, cy, 6, C_CYAN);
  canvas->setTextColor(C_CYAN);
  canvas->setCursor(cx + 47, cy - 4); canvas->print("E");

  // South (180 deg / Bottom)
  canvas->drawFastVLine(cx, cy + 56, 6, C_MUTED);
  canvas->setTextColor(C_MUTED);
  canvas->setCursor(cx - 3, cy + 46); canvas->print("S");

  // West (270 deg / Left)
  canvas->drawFastHLine(cx - 62, cy, 6, C_CYAN);
  canvas->setTextColor(C_CYAN);
  canvas->setCursor(cx - 53, cy - 4); canvas->print("W");

  // Sub-cardinal 30-degree tick marks
  for (int a = 30; a < 360; a += 30) {
    if (a == 90 || a == 180 || a == 270) continue;
    float rad = a * DEG_TO_RAD;
    float sa = sinf(rad), ca = cosf(rad);
    int x1 = cx + (int)(sa * 58.0f), y1 = cy - (int)(ca * 58.0f);
    int x2 = cx + (int)(sa * 62.0f), y2 = cy - (int)(ca * 62.0f);
    canvas->drawLine(x1, y1, x2, y2, 0x2945);
  }

  // Live heading pointer bug on compass ring
  float hRad = yaw * DEG_TO_RAD;
  int hx = cx + (int)(sinf(hRad) * 60.0f);
  int hy = cy - (int)(cosf(hRad) * 60.0f);
  canvas->fillCircle(hx, hy, 3, C_GREEN);

  // Cardinal quadrant text
  const char *card = "N";
  if (yaw >= 22.5f && yaw < 67.5f) card = "NE";
  else if (yaw >= 67.5f && yaw < 112.5f) card = "E";
  else if (yaw >= 112.5f && yaw < 157.5f) card = "SE";
  else if (yaw >= 157.5f && yaw < 202.5f) card = "S";
  else if (yaw >= 202.5f && yaw < 247.5f) card = "SW";
  else if (yaw >= 247.5f && yaw < 292.5f) card = "W";
  else if (yaw >= 292.5f && yaw < 337.5f) card = "NW";

  CarRenderer::drawGizmo(canvas, 206, 42, 16, sRot);
  canvas->setTextSize(1); canvas->setTextColor(C_MUTED);
  canvas->setCursor(4, 22);  canvas->printf("R %+5.1f", roll);
  canvas->setCursor(4, 32);  canvas->printf("P %+5.1f", pitch);
  canvas->setCursor(4, 42);  canvas->printf("Y %+5.1f", yaw);
  canvas->setCursor(4, 54);  canvas->setTextColor(C_GREEN);
  canvas->printf("%03.0f %s", yaw, card);

  canvas->setCursor(178, 92); canvas->setTextColor(C_MUTED);
  canvas->printf("G %+4.0f", tm.gyro[2]);
  canvas->setCursor(178, 102); canvas->printf("A %.2fg", sqrtf(tm.acc[0]*tm.acc[0] + tm.acc[1]*tm.acc[1] + tm.acc[2]*tm.acc[2]));

  // Inspection hint / status
  if (fabsf(tm.inspectYaw) > 0.5f) {
    canvas->setTextColor(C_CYAN); canvas->setCursor(34, 144);
    canvas->printf("360 INSPECT: %03.0f (CLICK:0)", tm.inspectYaw);
  } else {
    canvas->setTextColor(C_MUTED); canvas->setCursor(46, 144);
    canvas->print("KNOB: 360 ROTATE MODEL");
  }
}

void DisplayUI::drawMap(const RemoteTelemetry &tm) {
  drawHeader(tm, "RADAR / MAP");
  const int cx = 120, cy = 92;
  const float cmPerPx = 2.0f;

  // Concentric radar range rings (20cm, 40cm, 60cm)
  for (int r = 20; r <= 60; r += 20) canvas->drawCircle(cx, cy, r, 0x10A2);
  canvas->drawFastHLine(0, cy, CW, 0x0841); canvas->drawFastVLine(cx, 15, CH - 15, 0x0841);

  // 1. Update Breadcrumb Path Trail
  float dTrailSq = (tm.posCm[0] - lastTrailCmX) * (tm.posCm[0] - lastTrailCmX) + (tm.posCm[1] - lastTrailCmY) * (tm.posCm[1] - lastTrailCmY);
  if (dTrailSq >= 16.0f || pathTrailN == 0) { // 4cm movement step
    lastTrailCmX = tm.posCm[0]; lastTrailCmY = tm.posCm[1];
    pathTrail[pathTrailHead] = {tm.posCm[0], tm.posCm[1]};
    pathTrailHead = (pathTrailHead + 1) % 48;
    if (pathTrailN < 48) pathTrailN++;
  }

  // 2. Update Persisted Obstacle Dot Cloud
  float yaw = tm.imuYawDeg * DEG_TO_RAD;
  if (tm.hasTelem && (tm.calFlags & CAL_MAG_OK)) {
    yaw = (float)tm.roverHeading * DEG_TO_RAD;
  }

  struct RayHit { uint16_t mm; float relAng; uint8_t type; } rays[3] = {
    {tm.ultraMm, 0.0f, 0},
    {tm.tofMm[0], 0.5585f, 1},
    {tm.tofMm[1], -0.5585f, 2}
  };

  for (int i = 0; i < 3; i++) {
    if (rays[i].mm >= 25 && rays[i].mm <= 1500) {
      float distCm = rays[i].mm / 10.0f;
      float hitAng = yaw + rays[i].relAng;
      float wx = tm.posCm[0] + distCm * cosf(hitAng);
      float wy = tm.posCm[1] + distCm * sinf(hitAng);
      sensorDots[sensorDotHead] = {wx, wy, rays[i].type, 0};
      sensorDotHead = (sensorDotHead + 1) % 48;
      if (sensorDotCount < 48) sensorDotCount++;
    }
  }
  for (int i = 0; i < (int)sensorDotCount; i++) {
    if (sensorDots[i].age < 250) sensorDots[i].age++;
  }

  // 3. Render Connected Breadcrumb Path Trajectory (Gradient Cyan to Green)
  for (int i = 0; i < (int)pathTrailN - 1; i++) {
    int idx1 = (pathTrailHead - 1 - i + 48) % 48;
    int idx2 = (pathTrailHead - 2 - i + 48) % 48;
    int px1 = cx - (int)((pathTrail[idx1].y - tm.posCm[1]) / cmPerPx);
    int py1 = cy - (int)((pathTrail[idx1].x - tm.posCm[0]) / cmPerPx);
    int px2 = cx - (int)((pathTrail[idx2].y - tm.posCm[1]) / cmPerPx);
    int py2 = cy - (int)((pathTrail[idx2].x - tm.posCm[0]) / cmPerPx);
    if (px1 > 0 && px1 < CW && py1 > 15 && py1 < CH && px2 > 0 && px2 < CW && py2 > 15 && py2 < CH) {
      uint16_t col = (i < 10) ? C_GREEN : (i < 24 ? C_CYAN : 0x028A);
      canvas->drawLine(px1, py1, px2, py2, col);
      canvas->drawPixel(px1, py1, (i < 8) ? C_GREEN : C_CYAN);
    }
  }

  // 4. Render Persisted World-Frame Sensor Dot Trace Point Cloud
  for (int i = 0; i < (int)sensorDotCount; i++) {
    int px = cx - (int)((sensorDots[i].y - tm.posCm[1]) / cmPerPx);
    int py = cy - (int)((sensorDots[i].x - tm.posCm[0]) / cmPerPx);
    if (px > 2 && px < CW - 2 && py > 17 && py < CH - 2) {
      uint16_t col = (sensorDots[i].type == 0) ? C_RED : (sensorDots[i].type == 1 ? C_YELLOW : C_ORANGE);
      if (sensorDots[i].age > 80) col = 0x6200; // faded older hit
      canvas->fillRect(px - 1, py - 1, 2, 2, col);
    }
  }

  // 5. Render Rover Triangular Heading Glyph
  float fx = sinf(yaw), fy = -cosf(yaw); // screen forward vector (0=N/Up, 90=E/Right)
  float lx = -fy, ly = fx;              // screen right vector (perp)
  canvas->fillTriangle(cx + (int)(fx * 9), cy + (int)(fy * 9),
                       cx + (int)(-fx * 6 + lx * 6), cy + (int)(-fy * 6 + ly * 6),
                       cx + (int)(-fx * 6 - lx * 6), cy + (int)(-fy * 6 - ly * 6), C_GREEN);

  // 6. Render Live Sensor Raycasts
  for (int i = 0; i < 3; i++) {
    if (!rays[i].mm || rays[i].mm > 1800) continue;
    float d = rays[i].mm / 10.0f / cmPerPx + 8;
    float c = cosf(rays[i].relAng), s = sinf(rays[i].relAng);
    float dx = fx * c + lx * s, dy = fy * c + ly * s;
    int ox = cx + (int)(dx * d), oy = cy + (int)(dy * d);
    canvas->drawLine(cx + (int)(dx * 9), cy + (int)(dy * 9), ox, oy, 0x4000);
    canvas->fillCircle(ox, oy, 3, i == 0 ? C_RED : (i == 1 ? C_YELLOW : C_ORANGE));
  }

  // 7. Footer Status
  canvas->setTextSize(1); canvas->setTextColor(C_MUTED);
  canvas->setCursor(4, 144);
  canvas->printf("X:%+.0f Y:%+.0f cm  PTS:%d  TRL:%d", tm.posCm[0], tm.posCm[1], sensorDotCount, pathTrailN);
}

void DisplayUI::drawSensors(const RemoteTelemetry &tm) {
  drawHeader(tm, "SENSORS");
  canvas->setTextSize(1);
  const char *names[3] = {"US ", "TOF L", "TOF R"};
  uint16_t vals[3] = {tm.ultraMm, tm.tofMm[0], tm.tofMm[1]};
  for (int i = 0; i < 3; i++) {
    int y = 20 + i * 14;
    canvas->setTextColor(C_MUTED); canvas->setCursor(4, y + 2); canvas->print(names[i]);
    uint16_t col = vals[i] && vals[i] < 250 ? C_RED : vals[i] && vals[i] < 600 ? C_ORANGE : C_GREEN;
    bar(44, y, 130, 10, vals[i] ? min((int)vals[i], 2000) : 0, 2000, col);
    canvas->setTextColor(C_TEXT); canvas->setCursor(180, y + 2);
    if (vals[i]) canvas->printf("%4dmm", vals[i]); else canvas->print("  ---");
  }
  const char *wn[4] = {"FL", "FR", "RL", "RR"};
  for (int i = 0; i < 4; i++) {
    int x = 4 + i * 58;
    canvas->setTextColor(C_MUTED); canvas->setCursor(x, 66); canvas->print(wn[i]);
    canvas->setTextColor(C_CYAN); canvas->setCursor(x + 14, 66); canvas->printf("%4d", tm.wheelMmS[i]);
  }
  canvas->setTextColor(C_MUTED);
  canvas->setCursor(4, 82);  canvas->printf("GYR %+6.1f %+6.1f %+6.1f dps", tm.gyro[0], tm.gyro[1], tm.gyro[2]);
  canvas->setCursor(4, 94);  canvas->printf("ACC %+5.2f %+5.2f %+5.2f g", tm.acc[0], tm.acc[1], tm.acc[2]);
  canvas->setCursor(4, 112); canvas->setTextColor(C_TEXT); canvas->print("CAL");
  const char *cn[4] = {"GYRO", "MAG", "LEVEL", "STILL"};
  for (int i = 0; i < 4; i++) {
    bool ok = tm.calFlags & (1 << i);
    int x = 30 + i * 52;
    canvas->fillRoundRect(x, 108, 48, 14, 3, ok ? 0x0320 : C_CARD);
    canvas->setTextColor(ok ? C_GREEN : C_MUTED); canvas->setCursor(x + 5, 112); canvas->print(cn[i]);
  }
  canvas->setTextColor(C_MUTED); canvas->setCursor(4, 128);
  canvas->printf("MAG COVERAGE %3d%%", tm.magCoverage);
  bar(110, 126, 120, 10, tm.magCoverage, 100, tm.magCoverage > 70 ? C_GREEN : C_YELLOW);
}

void DisplayUI::drawDrive(const RemoteTelemetry &tm) {
  drawHeader(tm, "IMU AIR DRIVE");
  const int cx = 68, cy = 82;

  // Outer bezel & deadband horizon ring
  canvas->drawCircle(cx, cy, 44, C_BORDER);
  canvas->drawCircle(cx, cy, 43, 0x18C3);
  canvas->drawCircle(cx, cy, 8, 0x10A2); // Deadband zone (3.5 deg)
  canvas->drawFastHLine(cx - 5, cy, 11, C_BORDER);
  canvas->drawFastVLine(cx, cy - 5, 11, C_BORDER);

  // Effective tilt angles according to user airAxis configuration
  float effP = tm.ctrlPitchDeg;
  float effR = -tm.ctrlRollDeg;
  if (tm.airAxis == 1) { effP = tm.ctrlRollDeg; effR = -tm.ctrlPitchDeg; }
  else if (tm.airAxis == 2) { effP = -tm.ctrlPitchDeg; effR = -tm.ctrlRollDeg; }
  else if (tm.airAxis == 3) { effP = tm.ctrlPitchDeg; effR = tm.ctrlRollDeg; }
  else if (tm.airAxis == 4) { effP = -tm.ctrlPitchDeg; effR = tm.ctrlRollDeg; }
  else if (tm.airAxis == 5) { effP = -tm.ctrlRollDeg; effR = tm.ctrlPitchDeg; }

  // Artificial horizon tilt bar
  float pClamped = constrain(effP, -25.0f, 25.0f);
  float rClamped = constrain(effR, -35.0f, 35.0f);
  int hy = cy - (int)(pClamped * 1.4f);
  float rRad = rClamped * DEG_TO_RAD;
  int hdx = (int)(cosf(rRad) * 26.0f);
  int hdy = (int)(sinf(rRad) * 26.0f);
  canvas->drawLine(cx - hdx, hy + hdy, cx + hdx, hy - hdy, 0x2145);

  // Live tilt vector bubble
  int bx = cx + (int)constrain(effR * 1.5f, -38.0f, 38.0f);
  int by = cy - (int)constrain(effP * 1.5f, -38.0f, 38.0f);
  canvas->drawLine(cx, cy, bx, by, 0x2987);

  bool inDeadband = (fabsf(effP) <= 3.5f && fabsf(effR) <= 3.5f);
  uint16_t bubbleCol = (tm.isBrake || tm.isEStop) ? C_RED : tm.deadmanEngaged ? (inDeadband ? C_GREEN : C_CYAN) : C_ORANGE;
  canvas->fillCircle(bx, by, 6, bubbleCol);
  canvas->drawCircle(bx, by, 7, C_TEXT);

  // Right Side Telemetry & Controls
  canvas->setTextSize(1);
  canvas->setTextColor(C_YELLOW);
  canvas->setCursor(128, 20);
  canvas->printf("MODE: %s", MODE_NAMES_REMOTE[tm.roverMode % MODE_COUNT]);

  canvas->setTextColor(C_TEXT);
  canvas->setCursor(128, 32);
  canvas->printf("PITCH: %+5.1f", effP);
  canvas->setCursor(128, 43);
  canvas->printf("ROLL:  %+5.1f", effR);


  // Throttle bar
  canvas->setTextColor(C_MUTED);
  canvas->setCursor(128, 56);
  canvas->print("THROTTLE");
  canvas->setTextColor(C_CYAN);
  canvas->setCursor(188, 56);
  canvas->printf("%+4d%%", tm.curY);
  canvas->drawRect(128, 66, 104, 7, C_BORDER);
  int tLen = constrain(abs(tm.curY) * 50 / 100, 0, 50);
  if (tm.curY > 0) canvas->fillRect(180, 68, tLen, 3, C_GREEN);
  else if (tm.curY < 0) canvas->fillRect(180 - tLen, 68, tLen, 3, C_CYAN);

  // Steering bar
  int steerVal = (tm.curX != 0) ? tm.curX : tm.curZ;
  canvas->setTextColor(C_MUTED);
  canvas->setCursor(128, 77);
  canvas->print("STEERING");
  canvas->setTextColor(C_CYAN);
  canvas->setCursor(188, 77);
  canvas->printf("%+4d%%", steerVal);
  canvas->drawRect(128, 87, 104, 7, C_BORDER);
  int sLen = constrain(abs(steerVal) * 50 / 100, 0, 50);
  if (steerVal > 0) canvas->fillRect(180, 89, sLen, 3, C_GREEN);
  else if (steerVal < 0) canvas->fillRect(180 - sLen, 89, sLen, 3, C_CYAN);

  // Status Badge on Right
  if (tm.isBrake || tm.isEStop) {
    canvas->fillRoundRect(128, 100, 104, 18, 3, 0x4800);
    canvas->drawRoundRect(128, 100, 104, 18, 3, C_RED);
    canvas->setTextColor(C_RED); canvas->setCursor(136, 105);
    canvas->print("ACTIVE BRAKE");
  } else if (tm.deadmanEngaged) {
    canvas->fillRoundRect(128, 100, 104, 18, 3, 0x0280);
    canvas->drawRoundRect(128, 100, 104, 18, 3, C_GREEN);
    canvas->setTextColor(C_GREEN); canvas->setCursor(131, 105);
    canvas->print("ENGAGED - DRV");
  } else {
    canvas->fillRoundRect(128, 100, 104, 18, 3, 0x2940);
    canvas->drawRoundRect(128, 100, 104, 18, 3, C_ORANGE);
    canvas->setTextColor(C_YELLOW); canvas->setCursor(133, 105);
    canvas->print("STANDBY CLUTCH");
  }

  // Full-Width Push-to-Gyrate Status Banner
  if (tm.isBrake || tm.isEStop) {
    canvas->fillRoundRect(6, 122, 228, 18, 3, 0x4800);
    canvas->drawRoundRect(6, 122, 228, 18, 3, C_RED);
    canvas->setTextColor(C_RED);
    canvas->setCursor(57, 127);
    canvas->print("ACTIVE BRAKE APPLIED");
  } else if (tm.deadmanEngaged) {
    canvas->fillRoundRect(6, 122, 228, 18, 3, 0x0320);
    canvas->drawRoundRect(6, 122, 228, 18, 3, C_GREEN);
    canvas->setTextColor(C_GREEN);
    canvas->setCursor(69, 127);
    canvas->print("ENGAGED - DRIVING");
  } else {
    canvas->fillRoundRect(6, 122, 228, 18, 3, 0x2940);
    canvas->drawRoundRect(6, 122, 228, 18, 3, C_ORANGE);
    canvas->setTextColor(C_YELLOW);
    canvas->setCursor(48, 127);
    canvas->print("HOLD [BOOST/IO8] TO DRIVE");
  }

  // Navigation hint
  canvas->setTextColor(C_MUTED);
  canvas->setCursor(4, 144);
  canvas->print("IO7:BRAKE | IO8:PUSH-TO-GYRATE | JOY:MODE");
}

void DisplayUI::drawMenu(const RemoteTelemetry &tm) {
  drawHeader(tm, "SETTINGS");
  const int rows = 6, rowH = 23, top = 18;
  int first = constrain(menuIndex - rows / 2, 0, MENU_ITEM_COUNT - rows);
  for (int r = 0; r < rows; r++) {
    int i = first + r, y = top + r * rowH;
    bool sel = (i == menuIndex);
    canvas->fillRoundRect(4, y, 232, rowH - 3, 4, sel ? C_CARD : C_BG);
    if (sel) canvas->drawRoundRect(4, y, 232, rowH - 3, 4, C_GREEN);
    canvas->setTextSize(1);
    canvas->setTextColor(sel ? C_GREEN : C_TEXT);
    canvas->setCursor(10, y + 7); canvas->print(MENU_ITEMS[i]);
    canvas->setCursor(148, y + 7);
    switch (i) {
      case 0: canvas->setTextColor(C_YELLOW); canvas->print(MODE_NAMES_REMOTE[tm.roverMode % MODE_COUNT]); break;
      case 1: canvas->setTextColor(tm.semiAuto ? C_GREEN : C_MUTED); canvas->print(tm.semiAuto ? "ON (TANK)" : "OFF"); break;
      case 2: canvas->setTextColor(C_GREEN); canvas->printf("%d%%", tm.autoSpeed); break;
      case 3: canvas->setTextColor(tm.crashProtect ? C_GREEN : C_RED); canvas->print(tm.crashProtect ? "ON (ALL)" : "OFF"); break;
      case 4: canvas->setTextColor(C_CYAN); canvas->print(tm.ctrlSource == CTRL_JOYSTICK ? "Joystick" : "Air tilt"); break;
      case 5: canvas->setTextColor(C_GREEN); canvas->print(AIR_AXIS_NAMES[tm.airAxis % 6]); break;
      case 6: canvas->setTextColor(C_GREEN); canvas->printf("%d%%", tm.speedLimit); break;
      case 7: canvas->setTextColor(C_CYAN); canvas->printf("%d%%", tm.deadzonePct); break;
      case 8: {
        const char *expStr = "Linear";
        if (tm.expoPct >= 80) expStr = "Precise";
        else if (tm.expoPct >= 50) expStr = "Sport";
        else if (tm.expoPct >= 20) expStr = "Mild";
        canvas->setTextColor(C_YELLOW); canvas->printf("%s (%d%%)", expStr, tm.expoPct);
        break;
      }
      case 9: canvas->setTextColor(tm.trimFwd == 0 ? C_MUTED : C_CYAN); canvas->printf("%+d%%", tm.trimFwd); break;
      case 10: canvas->setTextColor(tm.trimStrafe == 0 ? C_MUTED : C_CYAN); canvas->printf("%+d%%", tm.trimStrafe); break;
      case 11: canvas->setTextColor(tm.trimYaw == 0 ? C_MUTED : C_CYAN); canvas->printf("%+d%%", tm.trimYaw); break;
      case 12: canvas->setTextColor(C_GREEN); canvas->printf("%d / 10", tm.gyroSmoothing); break;
      case 13: canvas->setTextColor(C_GREEN); canvas->printf("%d%%", tm.brightness); break;
      case 14: canvas->setTextColor(tm.hapticsOn ? C_GREEN : C_RED); canvas->print(tm.hapticsOn ? "ON" : "OFF"); break;
      case 15: canvas->setTextColor(C_GREEN); canvas->printf("%d%%", tm.hapticStrength); break;
      case 16: {
        const char *mNames[3] = {"Jog/Spring", "3-Way Sw", "Continuous"};
        canvas->setTextColor(C_CYAN); canvas->print(mNames[tm.encMode % 3]);
        break;
      }
      case 17: {
        const char *repNames[4] = {"OFF", "Normal", "Fast", "Slow"};
        canvas->setTextColor(tm.encRepeat ? C_GREEN : C_MUTED);
        canvas->print(repNames[tm.encRepeat % 4]);
        break;
      }
      case 18: {
        const char *pinNames[3] = {"Auto", "IO10(ALT)", "IO17(Std)"};
        canvas->setTextColor(C_YELLOW);
        canvas->print(pinNames[tm.encDtPin % 3]);
        break;
      }
      case 19: {
        const char *sensNames[3] = {"High (1st)", "Norm (2st)", "Low (4st)"};
        canvas->setTextColor(C_CYAN); canvas->print(sensNames[tm.encSens % 3]);
        break;
      }
      case 20: canvas->setTextColor(C_MUTED); canvas->print("click"); break;
      case 21: canvas->setTextColor(C_MUTED); canvas->print("click"); break;
      case 22: canvas->setTextColor(tm.calFlags & CAL_LEVEL_OK ? C_GREEN : C_MUTED); canvas->print(tm.calFlags & CAL_LEVEL_OK ? "done" : "click"); break;
      case 23: canvas->setTextColor(C_MUTED); canvas->print("click"); break;
      case 24: canvas->setTextColor(C_GREEN); canvas->print("SAVE NVS"); break;
      case 25: canvas->setTextColor(C_MUTED); canvas->print("close"); break;
      default: break;
    }
  }
}

void DisplayUI::drawStrip(const RemoteTelemetry &tm) {
  static bool stripInited = false;
  if (!stripInited) {
    tft->fillRect(0, STRIP_Y, 240, 240 - STRIP_Y, 0x0000);
    tft->drawFastHLine(0, STRIP_Y - 1, 240, 0x07E0); // Bright green divider line
    stripInited = true;
  }

  char buf[3][24];
  float v = 0;
  for (int i = 0; i < 4; i++) v += abs(tm.wheelMmS[i]) / 4.0f;
  snprintf(buf[0], sizeof buf[0], "SPD %4.2f  BAT %.1fV", v / 1000.0f, tm.vbat);
  snprintf(buf[1], sizeof buf[1], "HDG %3d  %-8s", tm.roverHeading, MODE_NAMES_REMOTE[tm.roverMode % MODE_COUNT]);
  char usStr[8];
  if (tm.ultraMm > 0) snprintf(usStr, sizeof usStr, "%3dcm", tm.ultraMm / 10);
  else snprintf(usStr, sizeof usStr, " --- ");

  char tofStr[8];
  uint16_t minTof = (tm.tofMm[0] > 0 && tm.tofMm[1] > 0) ? min(tm.tofMm[0], tm.tofMm[1]) : max(tm.tofMm[0], tm.tofMm[1]);
  if (minTof > 0) snprintf(tofStr, sizeof tofStr, "%3dcm", minTof / 10);
  else snprintf(tofStr, sizeof tofStr, " --- ");

  snprintf(buf[2], sizeof buf[2], "TOF %s US %s", tofStr, usStr);

  tft->setTextSize(2);
  for (int i = 0; i < 3; i++) {
    if (strcmp(buf[i], stripLast[i]) == 0) continue;
    strcpy(stripLast[i], buf[i]);
    int y = STRIP_Y + 4 + i * 26;
    // Wipe row to solid black
    tft->fillRect(0, y - 2, 240, 24, 0x0000);

    // High-contrast pure bold white typography on solid black:
    uint16_t rowColor = 0xFFFF; // Pure bold white
    if (i == 0 && tm.vbatPct < 20) rowColor = 0xF800; // Low-battery red warning alert

    tft->setTextColor(rowColor, 0x0000);
    // Bold pass 1
    tft->setCursor(12, y);
    tft->print(buf[i]);
    // Bold pass 2 (horizontal 1px shift for bold stroke)
    tft->setCursor(13, y);
    tft->print(buf[i]);
  }
}

void DisplayUI::render(const RemoteTelemetry &tm) {
  canvas->fillScreen(C_BG);
  if (inMenu) drawMenu(tm);
  else switch (view) {
    case VIEW_ATTITUDE: drawAttitude(tm); break;
    case VIEW_MAP:      drawMap(tm); break;
    case VIEW_SENSORS:  drawSensors(tm); break;
    case VIEW_DRIVE:    drawDrive(tm); break;
    default: break;
  }
  canvas->flush();
  drawStrip(tm);
}

int DisplayUI::menuSelect(RemoteTelemetry &tm) {
  if (!inMenu) return ACT_NONE;
  switch (menuIndex) {
    case 0:
      tm.roverMode = (tm.roverMode + 1) % MODE_COUNT;
      tm.semiAuto = (tm.roverMode == MODE_AUTO);
      return ACT_MODE;
    case 1:
      tm.semiAuto = !tm.semiAuto;
      tm.roverMode = tm.semiAuto ? MODE_AUTO : MODE_TANK;
      return ACT_SEMI_AUTO;
    case 2:
      tm.autoSpeed = (tm.autoSpeed >= 100) ? 40 : tm.autoSpeed + 5;
      return ACT_AUTO_SPEED;
    case 3:
      tm.crashProtect = !tm.crashProtect;
      return ACT_CRASH_PROT;
    case 4: tm.ctrlSource = (tm.ctrlSource == CTRL_JOYSTICK) ? CTRL_AIR_REMOTE : CTRL_JOYSTICK; return ACT_CTRL;
    case 5: tm.airAxis = (tm.airAxis + 1) % 6; return ACT_AIR_AXIS;
    case 6: tm.speedLimit = (tm.speedLimit >= 100) ? 10 : tm.speedLimit + 5; return ACT_SPEED;
    case 7: tm.deadzonePct = (tm.deadzonePct >= 20) ? 2 : tm.deadzonePct + 2; return ACT_DEADZONE;
    case 8: tm.expoPct = (tm.expoPct >= 90) ? 0 : tm.expoPct + 15; return ACT_EXPO;
    case 9: tm.trimFwd = (tm.trimFwd >= 10) ? -10 : tm.trimFwd + 2; return ACT_TRIM_FWD;
    case 10: tm.trimStrafe = (tm.trimStrafe >= 10) ? -10 : tm.trimStrafe + 2; return ACT_TRIM_STR;
    case 11: tm.trimYaw = (tm.trimYaw >= 10) ? -10 : tm.trimYaw + 2; return ACT_TRIM_YAW;
    case 12: tm.gyroSmoothing = (tm.gyroSmoothing >= 10) ? 1 : tm.gyroSmoothing + 1; return ACT_GYRO_SMOOTH;
    case 13: tm.brightness = (tm.brightness >= 100) ? 10 : tm.brightness + 10; setBacklight(tm.brightness); return ACT_BRIGHT;
    case 14: tm.hapticsOn = !tm.hapticsOn; return ACT_HAPTICS;
    case 15: tm.hapticStrength = (tm.hapticStrength >= 100) ? 10 : tm.hapticStrength + 10; return ACT_HAPTIC_STR;
    case 16: tm.encMode = (tm.encMode + 1) % 3; return ACT_ENC_MODE;
    case 17: tm.encRepeat = (tm.encRepeat + 1) % 4; return ACT_ENC_REPEAT;
    case 18: tm.encDtPin = (tm.encDtPin + 1) % 3; return ACT_ENC_PIN;
    case 19: tm.encSens = (tm.encSens + 1) % 3; return ACT_ENC_SENS;
    case 20: return ACT_CAL_JOY;
    case 21: return ACT_CAL_AIR;
    case 22: return ACT_CAL_LEVEL;
    case 23: return ACT_ZERO_YAW;
    case 24: return ACT_SAVE_SETTINGS;
    case 25: inMenu = false; return ACT_EXIT;
  }
  return ACT_NONE;
}

int DisplayUI::menuAdjust(RemoteTelemetry &tm, int dir) {
  if (!inMenu) return ACT_NONE;
  switch (menuIndex) {
    case 0: // Drive Mode
      if (dir > 0) tm.roverMode = (tm.roverMode + 1) % MODE_COUNT;
      else tm.roverMode = (tm.roverMode + MODE_COUNT - 1) % MODE_COUNT;
      tm.semiAuto = (tm.roverMode == MODE_AUTO);
      return ACT_MODE;
    case 1: // Semi-Auto Mode
      tm.semiAuto = (dir > 0);
      tm.roverMode = tm.semiAuto ? MODE_AUTO : MODE_TANK;
      return ACT_SEMI_AUTO;
    case 2: // Auto Speed
      tm.autoSpeed = constrain((int)tm.autoSpeed + dir * 5, 40, 100);
      return ACT_AUTO_SPEED;
    case 3: // Crash Protect
      tm.crashProtect = (dir > 0);
      return ACT_CRASH_PROT;
    case 4: // Control Input
      tm.ctrlSource = (tm.ctrlSource == CTRL_JOYSTICK) ? CTRL_AIR_REMOTE : CTRL_JOYSTICK;
      return ACT_CTRL;
    case 5: // Air Gyro Axis
      if (dir > 0) tm.airAxis = (tm.airAxis + 1) % 6;
      else tm.airAxis = (tm.airAxis + 5) % 6;
      return ACT_AIR_AXIS;
    case 6: // Speed Limit
      tm.speedLimit = constrain((int)tm.speedLimit + dir * 5, 10, 100);
      return ACT_SPEED;
    case 7: // Deadzone
      tm.deadzonePct = constrain((int)tm.deadzonePct + dir * 2, 2, 20);
      return ACT_DEADZONE;
    case 8: // Expo Curve
      tm.expoPct = constrain((int)tm.expoPct + dir * 15, 0, 90);
      return ACT_EXPO;
    case 9: // Trim Fwd
      tm.trimFwd = constrain((int)tm.trimFwd + dir * 2, -10, 10);
      return ACT_TRIM_FWD;
    case 10: // Trim Strafe
      tm.trimStrafe = constrain((int)tm.trimStrafe + dir * 2, -10, 10);
      return ACT_TRIM_STR;
    case 11: // Trim Yaw
      tm.trimYaw = constrain((int)tm.trimYaw + dir * 2, -10, 10);
      return ACT_TRIM_YAW;
    case 12: // Gyro Smoothing
      tm.gyroSmoothing = constrain((int)tm.gyroSmoothing + dir * 1, 1, 10);
      return ACT_GYRO_SMOOTH;
    case 13: // Brightness
      tm.brightness = constrain((int)tm.brightness + dir * 10, 10, 100);
      setBacklight(tm.brightness);
      return ACT_BRIGHT;
    case 14: // Haptics
      tm.hapticsOn = (dir > 0);
      return ACT_HAPTICS;
    case 15: // Haptic Strength
      tm.hapticStrength = constrain((int)tm.hapticStrength + dir * 10, 10, 100);
      return ACT_HAPTIC_STR;
    case 16: // Enc Mode
      if (dir > 0) tm.encMode = (tm.encMode + 1) % 3;
      else tm.encMode = (tm.encMode + 2) % 3;
      return ACT_ENC_MODE;
    case 17: // Enc Repeat
      if (dir > 0) tm.encRepeat = (tm.encRepeat + 1) % 4;
      else tm.encRepeat = (tm.encRepeat + 3) % 4;
      return ACT_ENC_REPEAT;
    case 18: // Enc DT Pin
      if (dir > 0) tm.encDtPin = (tm.encDtPin + 1) % 3;
      else tm.encDtPin = (tm.encDtPin + 2) % 3;
      return ACT_ENC_PIN;
    case 19: // Enc Sens
      if (dir > 0) tm.encSens = (tm.encSens + 1) % 3;
      else tm.encSens = (tm.encSens + 2) % 3;
      return ACT_ENC_SENS;
    default:
      break;
  }
  return ACT_NONE;
}


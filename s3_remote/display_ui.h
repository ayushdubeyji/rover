#pragma once
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <SPI.h>
#include "pin_defs.h"
#include "rover_proto.h"
#include "car_render.h"

enum ControlSource { CTRL_JOYSTICK = 0, CTRL_AIR_REMOTE };
enum UiView { VIEW_ATTITUDE = 0, VIEW_MAP, VIEW_SENSORS, VIEW_DRIVE, VIEW_COUNT };

enum UiAction {
  ACT_NONE = 0, ACT_MODE, ACT_SEMI_AUTO, ACT_AUTO_SPEED, ACT_CRASH_PROT, ACT_CTRL, ACT_AIR_AXIS, ACT_SPEED, ACT_DEADZONE, ACT_EXPO,
  ACT_TRIM_FWD, ACT_TRIM_STR, ACT_TRIM_YAW, ACT_GYRO_SMOOTH,
  ACT_BRIGHT, ACT_HAPTICS, ACT_HAPTIC_STR,
  ACT_ENC_MODE, ACT_ENC_REPEAT, ACT_ENC_PIN, ACT_ENC_SENS,
  ACT_CAL_JOY, ACT_CAL_AIR,
  ACT_CAL_LEVEL, ACT_ZERO_YAW, ACT_SAVE_SETTINGS, ACT_EXIT
};

struct RemoteTelemetry {
  float vbat = 4.0f;
  int vbatPct = 95;
  bool linked = false;
  int16_t roverHeading = 0;
  uint8_t roverMode = MODE_TANK;
  uint8_t roverSpeed = 60;
  bool semiAuto = false;
  bool crashProtect = true;
  int8_t curX = 0, curY = 0, curZ = 0;
  bool isTurbo = false, isEStop = false, isBrake = false, deadmanEngaged = false;
  uint8_t ctrlSource = CTRL_JOYSTICK;
  uint8_t airAxis = 0;         // 0=Std(P/R), 1=Swap(R/P), 2=Inv-P, 3=Inv-R, 4=Inv-Both, 5=Swap-Inv
  uint8_t renderMode = 1;      // 1 = CAD 3D Solid (60 FPS fluid), 0 = PBR Digital Twin
  float ctrlPitchDeg = 0.0f;   // Live handheld controller tilt pitch for Air Drive
  float ctrlRollDeg = 0.0f;    // Live handheld controller tilt roll for Air Drive
  float inspectYaw = 0.0f;     // 360-degree inspection rotation on Attitude HUD
  float imuPitchDeg = 0.0f;    // Rover pitch angle from Rover PKT_TELEM quaternion
  float imuRollDeg = 0.0f;     // Rover roll angle from Rover PKT_TELEM quaternion
  float imuYawDeg = 0.0f;      // Rover yaw angle from Rover PKT_TELEM quaternion

  // Finer controller tuning parameters
  uint8_t speedLimit = 50;     // 10% to 100%, 5% steps
  uint8_t autoSpeed = 50;      // 5% to 100%, 5% steps (Auto Navigation speed)
  uint8_t deadzonePct = 6;     // 2% to 20%, 2% steps
  uint8_t expoPct = 45;        // 0% (linear) to 90% (high expo), default 45% for smooth low-speed
  int8_t trimFwd = 0;          // -15% to +15%
  int8_t trimStrafe = 0;       // -15% to +15%
  int8_t trimYaw = 0;          // -15% to +15%
  uint8_t gyroSmoothing = 4;   // 1 to 10
  uint8_t brightness = 80;     // 10% to 100%, 5% steps
  bool hapticsOn = true;
  uint8_t hapticStrength = 100; // 10% to 100%, 10% steps

  // Rotary encoder dedicated parameters
  uint8_t encMode = 0;         // 0=Jog/Spring, 1=3-Way Switch, 2=Continuous
  uint8_t encRepeat = 1;       // 0=Off, 1=Normal (140ms), 2=Fast (80ms), 3=Slow (220ms)
  uint8_t encDtPin = 0;        // 0=Auto-detect, 1=IO10 (ALT), 2=IO17 (Std)
  uint8_t encSens = 0;         // 0=High (1-step), 1=Normal (2-step), 2=Low (4-step)

  // Rover sensor telemetry (PKT_TELEM)
  bool hasTelem = false;
  unsigned long lastTelemMs = 0;
  Quat q;
  float gyro[3] = {0, 0, 0};   // deg/s
  float acc[3] = {0, 0, 1};    // g
  uint16_t ultraMm = 0, tofMm[2] = {0, 0};
  int16_t wheelMmS[4] = {0, 0, 0, 0};
  float posCm[2] = {0, 0};
  uint8_t calFlags = 0, magCoverage = 0;
  uint8_t autoState = 0;       // Autonomous Navigation State (0..11)
};

struct SensorDot {
  float x;
  float y;
  uint8_t type; // 0=US, 1=ToF L, 2=ToF R
  uint8_t age;
};

struct PathTrailNode {
  float x;
  float y;
};

class DisplayUI {
public:
  bool inMenu = false;
  int menuIndex = 0;
  UiView view = VIEW_ATTITUDE;
  static const int MENU_ITEM_COUNT = 26;

  void begin();
  void setBacklight(uint8_t percent);
  void render(const RemoteTelemetry &telemetry);
  void nextView() { view = (UiView)((view + 1) % VIEW_COUNT); }
  void toggleMenu() { inMenu = !inMenu; }
  void menuNext() { if (inMenu) menuIndex = (menuIndex + 1) % MENU_ITEM_COUNT; }
  void menuPrev() { if (inMenu) menuIndex = (menuIndex + MENU_ITEM_COUNT - 1) % MENU_ITEM_COUNT; }
  int menuSelect(RemoteTelemetry &telemetry);
  int menuAdjust(RemoteTelemetry &telemetry, int dir);

private:
  Arduino_DataBus *bus = nullptr;
  Arduino_GFX *tft = nullptr;
  Arduino_Canvas *canvas = nullptr;
  CarRenderer car;
  Quat shown;
  char stripLast[3][24] = {"", "", ""};
  PathTrailNode pathTrail[48];
  uint8_t pathTrailN = 0, pathTrailHead = 0;
  float lastTrailCmX = 0, lastTrailCmY = 0;

  SensorDot sensorDots[48];
  uint8_t sensorDotCount = 0, sensorDotHead = 0;

  void drawHeader(const RemoteTelemetry &tm, const char *title);
  void drawAttitude(const RemoteTelemetry &tm);
  void drawMap(const RemoteTelemetry &tm);
  void drawSensors(const RemoteTelemetry &tm);
  void drawDrive(const RemoteTelemetry &tm);
  void drawMenu(const RemoteTelemetry &tm);
  void drawStrip(const RemoteTelemetry &tm);
  void bar(int x, int y, int w, int h, int val, int maxv, uint16_t col);
};

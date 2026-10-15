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
  ACT_NONE = 0, ACT_MODE, ACT_CTRL, ACT_SPEED, ACT_DEADZONE, ACT_EXPO,
  ACT_TRIM_FWD, ACT_TRIM_STR, ACT_TRIM_YAW, ACT_GYRO_SMOOTH,
  ACT_BRIGHT, ACT_HAPTICS, ACT_HAPTIC_STR, ACT_CAL_JOY, ACT_CAL_AIR,
  ACT_CAL_LEVEL, ACT_ZERO_YAW, ACT_SAVE_SETTINGS, ACT_EXIT
};

struct RemoteTelemetry {
  float vbat = 4.0f;
  int vbatPct = 95;
  bool linked = false;
  int16_t roverHeading = 0;
  uint8_t roverMode = MODE_TANK;
  uint8_t roverSpeed = 60;
  int8_t curX = 0, curY = 0, curZ = 0;
  bool isTurbo = false, isEStop = false, isBrake = false, deadmanEngaged = false;
  uint8_t ctrlSource = CTRL_JOYSTICK;
  uint8_t renderMode = 1;      // 1 = CAD 3D Solid (60 FPS fluid), 0 = PBR Digital Twin
  float inspectYaw = 0.0f;     // 360-degree inspection rotation on Attitude HUD
  float imuPitchDeg = 0.0f;    // Live handheld IMU pitch angle for dedicated IMU drive
  float imuRollDeg = 0.0f;     // Live handheld IMU roll angle for dedicated IMU drive

  // Finer controller tuning parameters
  uint8_t speedLimit = 60;     // 10% to 100%, 5% steps
  uint8_t deadzonePct = 6;     // 2% to 20%, 2% steps
  uint8_t expoPct = 30;        // 0% (linear) to 90% (high expo)
  int8_t trimFwd = 0;          // -15% to +15%
  int8_t trimStrafe = 0;       // -15% to +15%
  int8_t trimYaw = 0;          // -15% to +15%
  uint8_t gyroSmoothing = 4;   // 1 to 10
  uint8_t brightness = 80;     // 10% to 100%, 5% steps
  bool hapticsOn = true;
  uint8_t hapticStrength = 60; // 10% to 100%, 10% steps

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
};

class DisplayUI {
public:
  bool inMenu = false;
  int menuIndex = 0;
  UiView view = VIEW_ATTITUDE;
  static const int MENU_ITEM_COUNT = 18;

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
  int16_t trailX[64], trailY[64];
  uint8_t trailN = 0, trailHead = 0;
  float lastTrailX = 0, lastTrailY = 0;

  void drawHeader(const RemoteTelemetry &tm, const char *title);
  void drawAttitude(const RemoteTelemetry &tm);
  void drawMap(const RemoteTelemetry &tm);
  void drawSensors(const RemoteTelemetry &tm);
  void drawDrive(const RemoteTelemetry &tm);
  void drawMenu(const RemoteTelemetry &tm);
  void drawStrip(const RemoteTelemetry &tm);
  void bar(int x, int y, int w, int h, int val, int maxv, uint16_t col);
};

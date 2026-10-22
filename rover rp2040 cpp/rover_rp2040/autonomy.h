#pragma once
#include <Arduino.h>
#include <math.h>
#include "rover_link.h"
#include "pins.h"

// ==============================================================================
// 12-State Autonomous Navigation Engine & Crash Protection
// ==============================================================================

class RoverAutonomy {
public:
  AutoNavState autoState = AUTO_CRUISE_CLEAR;
  uint32_t autoStateStart = 0;
  uint32_t autoStateUntil = 0;
  float autoTargetHeading = 0.0f;
  int autoEscapeDir = 1;
  bool autoPhaseSwitched = false;

  // 12-Sector Polar Obstacle Histogram (each sector = 30 deg, -180 to +180)
  float polarHist[12] = {0};

  // Breadcrumb Path Trail for Anti-Oscillation & Escapes
  struct PathNode { float x; float y; };
  PathNode pathHist[64] = {};
  uint8_t pathHistHead = 0;
  uint8_t pathHistCount = 0;
  uint32_t lastPathRecordMs = 0;

  // Stuck & Trap Detection
  uint32_t escapeTurnTimes[3] = {0, 0, 0};
  uint8_t escapeTurnIdx = 0;
  uint32_t lastProgressCheckTime = 0;
  float checkOdomX = 0.0f;
  float checkOdomY = 0.0f;
  uint32_t stallStartMs = 0;
  uint16_t prevDF = 0;
  uint16_t prevDR = 0;

  RoverAutonomy() {
    for (int i = 0; i < 12; i++) polarHist[i] = 0.0f;
  }

  static inline float wrapDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }

  // Update polar obstacle histogram from dual ToF (Front 0 deg, Rear 180 deg)
  void updateSensoryFusion(uint16_t dF, uint16_t dR, float heading, float odomX, float odomY, uint32_t now_ms) {
    // 1. Exponential decay of existing polar histogram
    for (int i = 0; i < 12; i++) {
      polarHist[i] *= 0.985f;
    }

    // 2. Project Front ToF hit into body/world frame (0 deg body offset, +11cm fwd)
    if (dF >= 25 && dF <= 2000) {
      float dM = (float)dF / 1000.0f;
      float hitAng = wrapDeg(heading + 0.0f);
      int s = (int)floorf((hitAng + 180.0f) / 30.0f) % 12;
      if (s < 0) s += 12;
      float weight = constrain(1.0f / (dM + 0.1f), 0.5f, 10.0f);
      polarHist[s] += weight;
    }

    // 3. Project Rear ToF hit into body/world frame (180 deg body offset, -10cm rear)
    if (dR >= 25 && dR <= 2000) {
      float dM = (float)dR / 1000.0f;
      float hitAng = wrapDeg(heading + 180.0f);
      int s = (int)floorf((hitAng + 180.0f) / 30.0f) % 12;
      if (s < 0) s += 12;
      float weight = constrain(1.0f / (dM + 0.1f), 0.5f, 10.0f);
      polarHist[s] += weight;
    }

    // 4. Update breadcrumb path history every 250 ms if moved > 5 cm
    if (now_ms - lastPathRecordMs >= 250) {
      lastPathRecordMs = now_ms;
      if (pathHistCount == 0) {
        pathHist[pathHistHead] = { odomX, odomY };
        pathHistHead = (pathHistHead + 1) % 64;
        pathHistCount++;
      } else {
        int prev = (pathHistHead - 1 + 64) % 64;
        float dx = odomX - pathHist[prev].x;
        float dy = odomY - pathHist[prev].y;
        if (sqrtf(dx * dx + dy * dy) >= 5.0f) {
          pathHist[pathHistHead] = { odomX, odomY };
          pathHistHead = (pathHistHead + 1) % 64;
          if (pathHistCount < 64) pathHistCount++;
        }
      }
    }
  }

  // Velocity-scaled dual crash protection (Front 0 deg & Rear 180 deg)
  void applyCrashProtection(float &x, float &y, float &z, float cur_spd,
                            uint16_t dF, uint16_t dR, uint8_t impactDir,
                            uint32_t impactUntil, uint32_t now_ms) {
    // Physical collision reflex takes absolute priority
    if (now_ms < impactUntil && impactDir == IMPACT_FRONT && y > 0.0f) {
      y = -0.30f;
      return;
    }
    if (now_ms < impactUntil && impactDir == IMPACT_REAR && y < 0.0f) {
      y = 0.30f;
      return;
    }

    float v_fwd = fabsf(y) * (cur_spd / 100.0f);
    float inertiaLeadMm = v_fwd * 70.0f * 1.4f;

    // Front Crash Protection: <180 mm hard emergency stop, warn up to 500 mm
    float stopF = 180.0f + inertiaLeadMm * 0.5f;
    float warnF = 500.0f;
    if (y > 0.0f && dF > 0) {
      if (dF <= stopF) {
        y = 0.0f; // Hard crash stop
      } else if (dF < warnF) {
        float scale = constrain((float)(dF - stopF) / (warnF - stopF), 0.0f, 1.0f);
        y *= scale;
      }
    }

    // Rear Crash Protection: <180 mm hard emergency stop, warn up to 450 mm
    float stopR = 180.0f + inertiaLeadMm * 0.5f;
    float warnR = 450.0f;
    if (y < 0.0f && dR > 0) {
      if (dR <= stopR) {
        y = 0.0f; // Hard crash stop in reverse
      } else if (dR < warnR) {
        float scale = constrain((float)(dR - stopR) / (warnR - stopR), 0.0f, 1.0f);
        y *= scale;
      }
    }
  }

  // 12-State Navigation FSM Step
  void runAutonomous(float &out_x, float &out_y, float &out_z, float &out_spd,
                     uint16_t dF, uint16_t dR, float heading, float odomX, float odomY,
                     float dynJerk, uint8_t impactDir, uint32_t impactUntil,
                     bool radar_moving, uint32_t now_ms) {

    updateSensoryFusion(dF, dR, heading, odomX, odomY, now_ms);

    // Priority 1: Physical impact reflex (State 11)
    if (now_ms < impactUntil && impactDir != IMPACT_NONE && autoState != AUTO_IMPACT_REFLEX) {
      autoState = AUTO_IMPACT_REFLEX;
      autoStateStart = now_ms;
      autoStateUntil = now_ms + 500;
      autoPhaseSwitched = true;
    }

    // Stuck & Trap Evaluation
    uint16_t STOP_F = 350;
    uint16_t WARN_F = 800;
    bool isBoxedIn = (dF > 0 && dF < STOP_F);

    // Oscillation Trap: 3 turns within 6s or <12cm progress in 3.0s
    bool isOscillationTrap = false;
    if (now_ms - escapeTurnTimes[escapeTurnIdx] < 6000 && escapeTurnTimes[2] > 0) {
      isOscillationTrap = true;
    }
    if (now_ms - lastProgressCheckTime >= 3000) {
      float netDist = sqrtf((odomX - checkOdomX) * (odomX - checkOdomX) +
                            (odomY - checkOdomY) * (odomY - checkOdomY));
      if (netDist < 12.0f && fabsf(out_y) > 0.12f) {
        isOscillationTrap = true;
      }
      lastProgressCheckTime = now_ms;
      checkOdomX = odomX;
      checkOdomY = odomY;
    }

    // Range-rate Stall & IMU Jerk Discrepancy:
    // Commanded forward motion while range does not change and high jerk or zero delta
    bool isStall = false;
    if (fabsf(out_y) > 0.35f && dF > 0) {
      if (abs((int)dF - (int)prevDF) < 12) {
        if (stallStartMs == 0) stallStartMs = now_ms;
        else if (now_ms - stallStartMs > 900) isStall = true;
      } else {
        stallStartMs = 0; prevDF = dF;
      }
    } else {
      stallStartMs = 0; prevDF = dF;
    }

    // Trigger Stuck Alert State when in forward states
    bool inForwardState = (autoState == AUTO_CRUISE_CLEAR || autoState == AUTO_THINKING_SCAN ||
                           autoState == AUTO_CAUTIOUS_CREEP || autoState == AUTO_VEER_LEFT ||
                           autoState == AUTO_VEER_RIGHT || autoState == AUTO_WALL_FOLLOW);

    if ((isBoxedIn || isOscillationTrap || isStall) && inForwardState) {
      escapeTurnTimes[escapeTurnIdx] = now_ms;
      escapeTurnIdx = (escapeTurnIdx + 1) % 3;
      autoState = AUTO_STUCK_DETECTED;
      autoStateStart = now_ms;
      autoStateUntil = now_ms + 180;
      autoPhaseSwitched = true;
    }

    // --- 12-State Navigation Execution ---
    out_x = 0.0f; // Pure tank / differential kinematics

    switch (autoState) {
      case AUTO_CRUISE_CLEAR: { // State 0: Corridor clear forward cruise
        if (dF > 0 && (dF < WARN_F || (radar_moving && dF < 1200))) {
          autoState = AUTO_THINKING_SCAN;
          autoStateStart = now_ms;
          autoStateUntil = now_ms + 180;
          autoPhaseSwitched = true;
          out_y = 0.25f; out_z = 0.0f;
        } else {
          out_y = 0.95f; out_z = 0.0f;
          out_spd = constrain(out_spd, 20.0f, 85.0f);
        }
        break;
      }

      case AUTO_THINKING_SCAN: { // State 1: Brief scan evaluating polar histogram
        out_y = 0.20f; out_z = 0.0f;
        if (now_ms >= autoStateUntil) {
          if (dF > 0 && dF < STOP_F) {
            autoState = AUTO_PLANNING_PATH;
            autoPhaseSwitched = true;
          } else if (dF > 0 && dF <= WARN_F) {
            autoState = AUTO_CAUTIOUS_CREEP;
            autoPhaseSwitched = true;
          } else {
            autoState = AUTO_CRUISE_CLEAR;
            autoPhaseSwitched = true;
          }
        }
        break;
      }

      case AUTO_PLANNING_PATH: { // State 2: Select lowest-cost polar histogram sector
        out_y = 0.0f; out_z = 0.0f;
        int bestSector = -1;
        float bestCost = 999999.0f;
        for (int s = 0; s < 12; s++) {
          float sectorAng = -180.0f + (float)s * 30.0f + 15.0f;
          float diffFromCurrent = fabsf(wrapDeg(sectorAng - heading));
          float cost = polarHist[s];
          if (diffFromCurrent < 60.0f) cost += 15.0f; // slight penalty for current forward path
          if (cost < bestCost) {
            bestCost = cost;
            bestSector = s;
          }
        }
        autoTargetHeading = -180.0f + (float)bestSector * 30.0f + 15.0f;
        autoEscapeDir = (wrapDeg(autoTargetHeading - heading) >= 0.0f) ? 1 : -1;
        autoState = AUTO_COMPASS_PIVOT;
        autoStateStart = now_ms;
        autoStateUntil = now_ms + 1200;
        autoPhaseSwitched = true;
        break;
      }

      case AUTO_VEER_LEFT: { // State 3: Proportional left arc
        out_z = 0.45f;
        out_y = 0.70f;
        if (dF == 0 || dF > 600) {
          autoState = AUTO_CRUISE_CLEAR;
          autoPhaseSwitched = true;
        } else if (dF > 0 && dF < 280) {
          autoState = AUTO_THINKING_SCAN;
          autoStateStart = now_ms;
          autoStateUntil = now_ms + 180;
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_VEER_RIGHT: { // State 4: Proportional right arc
        out_z = -0.45f;
        out_y = 0.70f;
        if (dF == 0 || dF > 600) {
          autoState = AUTO_CRUISE_CLEAR;
          autoPhaseSwitched = true;
        } else if (dF > 0 && dF < 280) {
          autoState = AUTO_THINKING_SCAN;
          autoStateStart = now_ms;
          autoStateUntil = now_ms + 180;
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_CAUTIOUS_CREEP: { // State 5: Slow precision creep
        out_y = 0.40f;
        out_z = 0.0f;
        out_spd = constrain(out_spd * 0.70f, 15.0f, 40.0f);
        if (dF == 0 || dF > 600) {
          autoState = AUTO_CRUISE_CLEAR;
          autoPhaseSwitched = true;
        } else if (dF > 0 && dF < 260) {
          autoState = AUTO_PLANNING_PATH;
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_STUCK_DETECTED: { // State 6: Stall / boxed-in alert pause
        out_y = 0.0f; out_z = 0.0f;
        if (now_ms >= autoStateUntil) {
          if (isOscillationTrap) {
            autoState = AUTO_SCURVE_ESCAPE;
            autoStateStart = now_ms;
            autoStateUntil = now_ms + 850;
          } else {
            autoState = AUTO_REVERSE_BACKTRACK;
            autoStateStart = now_ms;
            autoStateUntil = now_ms + 750;
          }
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_REVERSE_BACKTRACK: { // State 7: Defensive reverse along breadcrumb
        out_y = -0.80f;
        out_z = -0.15f * (float)autoEscapeDir;
        out_spd = constrain(out_spd * 1.15f, 25.0f, 90.0f);
        bool clearanceOpen = (now_ms - autoStateStart >= 400) && (dF == 0 || dF > 450);
        bool rearBlocked = (dR > 0 && dR < 180);
        if (now_ms >= autoStateUntil || clearanceOpen || rearBlocked) {
          autoState = AUTO_PLANNING_PATH;
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_SCURVE_ESCAPE: { // State 8: Reverse + lateral S-curve arc
        out_y = -0.80f;
        out_z = -0.70f * (float)autoEscapeDir;
        out_spd = constrain(out_spd * 1.15f, 25.0f, 90.0f);
        bool rearBlocked = (dR > 0 && dR < 180);
        if (now_ms >= autoStateUntil || rearBlocked) {
          autoState = AUTO_PLANNING_PATH;
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_COMPASS_PIVOT: { // State 9: In-place compass-locked pivot
        out_y = 0.0f;
        float hErr = wrapDeg(autoTargetHeading - heading);
        out_z = constrain(hErr * 0.035f + 0.35f * (float)autoEscapeDir, -0.85f, 0.85f);
        if (fabsf(hErr) < 35.0f) {
          out_z *= constrain(fabsf(hErr) / 35.0f, 0.45f, 1.0f);
        }
        if (fabsf(hErr) < 18.0f || now_ms >= autoStateUntil) {
          autoState = AUTO_CRUISE_CLEAR;
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_WALL_FOLLOW: { // State 10: Parallel wall contour
        out_y = 0.70f;
        out_z = 0.0f;
        if (dF > 0 && dF < 450) {
          autoState = AUTO_THINKING_SCAN;
          autoStateStart = now_ms;
          autoStateUntil = now_ms + 180;
          autoPhaseSwitched = true;
        }
        break;
      }

      case AUTO_IMPACT_REFLEX: { // State 11: Emergency reverse reflex
        out_y = (impactDir == IMPACT_REAR) ? 0.75f : -0.80f;
        out_z = (impactDir == IMPACT_LEFT ? -0.75f : 0.75f);
        out_spd = max(out_spd, 65.0f);
        if (now_ms >= autoStateUntil) {
          autoState = AUTO_PLANNING_PATH;
          autoPhaseSwitched = true;
        }
        break;
      }

      default:
        autoState = AUTO_CRUISE_CLEAR;
        break;
    }
  }
};

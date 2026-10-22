#pragma once
#include <Arduino_GFX_Library.h>
#include <math.h>
#include "car_model.h"

struct Quat { float w = 1, x = 0, y = 0, z = 0; };

// Orthographic low-poly renderer: painter's algorithm, flat shading, fixed rear-elevated camera.
class CarRenderer {
public:
  CarRenderer() {
    for (int i = 0; i < CAR_TRI_COUNT; i++) {
      order[i] = i;
      m_tri[i][0] = pgm_read_word(&CAR_TRIS[i][0]);
      m_tri[i][1] = pgm_read_word(&CAR_TRIS[i][1]);
      m_tri[i][2] = pgm_read_word(&CAR_TRIS[i][2]);
      m_col[i] = pgm_read_word(&CAR_COLORS[i]);
    }
    for (int i = 0; i < CAR_VERT_COUNT; i++) {
      m_bx[i] = (int8_t)pgm_read_byte(&CAR_VERTS[i][0]) / 127.0f;
      m_by[i] = (int8_t)pgm_read_byte(&CAR_VERTS[i][1]) / 127.0f;
      m_bz[i] = (int8_t)pgm_read_byte(&CAR_VERTS[i][2]) / 127.0f;
    }
  }

  // Fallback PBR stub
  void drawPBR(Arduino_GFX *g, int cx, int cy, float yawDeg, float rollDeg = 0.0f, float pitchDeg = 0.0f) {
    (void)g; (void)cx; (void)cy; (void)yawDeg; (void)rollDeg; (void)pitchDeg;
  }

  static void rotMatrix(const Quat &q, float m[9]) {
    float w = q.w, x = q.x, y = q.y, z = q.z;
    m[0] = 1 - 2*(y*y + z*z); m[1] = 2*(x*y - w*z);     m[2] = 2*(x*z + w*y);
    m[3] = 2*(x*y + w*z);     m[4] = 1 - 2*(x*x + z*z); m[5] = 2*(y*z - w*x);
    m[6] = 2*(x*z - w*y);     m[7] = 2*(y*z + w*x);     m[8] = 1 - 2*(x*x + y*y);
  }

  // Rear 33-degree chase camera matching Three.js PBR camera:
  // In CAD model: +X is Forward, +Y is Left, +Z is Up
  static void toView(float bx, float by, float bz, float &vx, float &vy, float &vd) {
    const float cT = 0.8387f; // cos(33 deg)
    const float sT = 0.5446f; // sin(33 deg)
    vx  = -by;
    vy  =  bz * cT + bx * sT;
    vd  =  bx * cT - bz * sT;
  }

  void draw(Arduino_GFX *g, int cx, int cy, float scale, const Quat &q) {
    float R[9]; rotMatrix(q, R);
    for (int i = 0; i < CAR_VERT_COUNT; i++) {
      float bx = m_bx[i], by = m_by[i], bz = m_bz[i];
      toView(R[0]*bx + R[1]*by + R[2]*bz, R[3]*bx + R[4]*by + R[5]*bz, R[6]*bx + R[7]*by + R[8]*bz, vx[i], vy[i], vd[i]);
      sx[i] = cx + (int)(vx[i] * scale);
      sy[i] = cy - (int)(vy[i] * scale);
    }
    for (int t = 0; t < CAR_TRI_COUNT; t++) {
      uint16_t a = m_tri[t][0], b = m_tri[t][1], c = m_tri[t][2];
      key[t] = (vd[a] + vd[b] + vd[c]) * 0.3333f;
      float ux = vx[b] - vx[a], uy = vy[b] - vy[a], ud = vd[b] - vd[a];
      float wx = vx[c] - vx[a], wy = vy[c] - vy[a], wd = vd[c] - vd[a];
      float nx = uy*wd - ud*wy, ny = ud*wx - ux*wd, nd = ux*wy - uy*wx;
      float nl = sqrtf(nx*nx + ny*ny + nd*nd) + 1e-6f;
      float dot = fabsf((-0.4f*nx + 0.6f*ny - 0.7f*nd) / nl);
      shade[t] = (uint8_t)(195 + 60 * dot);
    }
    // insertion sort (far -> near) with depth hysteresis & deterministic tie-breaker (stops flicker)
    for (int i = 1; i < CAR_TRI_COUNT; i++) {
      uint16_t v = order[i]; float k = key[v]; int j = i - 1;
      while (j >= 0) {
        float diff = key[order[j]] - k;
        if (diff < -0.0025f || (fabsf(diff) <= 0.0025f && order[j] < v)) {
          order[j + 1] = order[j];
          j--;
        } else {
          break;
        }
      }
      order[j + 1] = v;
    }
    g->fillEllipse(cx, cy + (int)(scale * 0.42f), (int)(scale * 0.62f), (int)(scale * 0.16f), 0x2104);
    for (int i = 0; i < CAR_TRI_COUNT; i++) {
      uint16_t t = order[i];
      uint16_t a = m_tri[t][0], b = m_tri[t][1], c = m_tri[t][2];
      int16_t sa_x = sx[a], sa_y = sy[a];
      int16_t sb_x = sx[b], sb_y = sy[b];
      int16_t sc_x = sx[c], sc_y = sy[c];
      if ((sa_x == sb_x && sb_x == sc_x) || (sa_y == sb_y && sb_y == sc_y)) continue;
      uint16_t col = m_col[t];
      uint16_t s = shade[t];
      uint16_t r = (((col >> 11) & 31) * s) >> 8, gg = (((col >> 5) & 63) * s) >> 8, bl = ((col & 31) * s) >> 8;
      g->fillTriangle(sa_x, sa_y, sb_x, sb_y, sc_x, sc_y, (r << 11) | (gg << 5) | bl);
    }
  }

  // Body-axis gizmo: X red (forward), Y green (left), Z blue (up)
  static void drawGizmo(Arduino_GFX *g, int cx, int cy, int len, const Quat &q) {
    float R[9]; rotMatrix(q, R);
    const uint16_t cols[3] = {0xF800, 0x07E0, 0x041F};
    const char labels[3] = {'X', 'Y', 'Z'};
    for (int a = 0; a < 3; a++) {
      float vx_, vy_, vd_;
      toView(R[a], R[3 + a], R[6 + a], vx_, vy_, vd_);
      int ex = cx + (int)(vx_ * len), ey = cy - (int)(vy_ * len);
      g->drawLine(cx, cy, ex, ey, cols[a]);
      g->drawLine(cx + 1, cy, ex + 1, ey, cols[a]);
      g->fillCircle(ex, ey, 2, cols[a]);
      int lx = ex + (ex >= cx ? 4 : -10), ly = ey - 4;
      g->fillRect(lx - 1, ly - 1, 9, 10, 0x0000);
      g->setTextColor(cols[a]); g->setTextSize(1);
      g->setCursor(lx, ly);
      g->print(labels[a]);
    }
    g->fillCircle(cx, cy, 2, 0xFFFF);
  }

private:
  float m_bx[CAR_VERT_COUNT], m_by[CAR_VERT_COUNT], m_bz[CAR_VERT_COUNT];
  float vx[CAR_VERT_COUNT], vy[CAR_VERT_COUNT], vd[CAR_VERT_COUNT];
  int16_t sx[CAR_VERT_COUNT], sy[CAR_VERT_COUNT];
  float key[CAR_TRI_COUNT];
  uint8_t shade[CAR_TRI_COUNT];
  uint16_t order[CAR_TRI_COUNT];
  uint16_t m_tri[CAR_TRI_COUNT][3];
  uint16_t m_col[CAR_TRI_COUNT];
};

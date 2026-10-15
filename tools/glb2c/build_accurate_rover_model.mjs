import sharp from 'sharp';
import fs from 'fs';

// =========================================================================
// ACCURATE 3D MECANUM ROVER MODEL FOR ESP32-S3 CONTROLLER
// Faithfully matches tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt (1).glb
// =========================================================================
// Body Coordinate Frame:
// X_body = +Forward (towards front electronics board and wiring)
// Y_body = +Left (towards left mecanum wheels)
// Z_body = +Up (towards red battery pack top)
// Normalized to int8_t (-127 to +127)

const verts = [];
const tris = [];
const colors = [];

function addVert(x, y, z) {
  const clamp = v => Math.max(-127, Math.min(127, Math.round(v)));
  const v = [clamp(x), clamp(y), clamp(z)];
  verts.push(v);
  return verts.length - 1;
}

function addTri(i0, i1, i2, c565) {
  tris.push([i0, i1, i2]);
  colors.push(c565);
}

function addQuad(i0, i1, i2, i3, c565) {
  addTri(i0, i1, i2, c565);
  addTri(i0, i2, i3, c565);
}

// Convert RGB888 to RGB565
function rgb(r, g, b) {
  return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

// Palette with authentic high-fluorescence neon lime green and high-contrast materials:
const C_WHEEL_GREEN_BRIGHT = rgb(130, 255, 100); // Electric neon lime green mecanum rim/star spokes
const C_WHEEL_GREEN_MED    = rgb(90, 235, 70);   // Luminous lime bevel / bracket
const C_WHEEL_GREEN_DARK   = rgb(65, 205, 55);   // Vibrant lime wheel divider
const C_ROLLER_BLACK       = rgb(14, 16, 18);    // Deep matte black rubber mecanum roller
const C_ROLLER_CHARCOAL    = rgb(35, 40, 45);    // Highlighted rubber roller facet
const C_HUB_RECESS         = rgb(18, 32, 22);    // Deep inner wheel recess
const C_AXLE_NUT           = rgb(210, 225, 245); // Bright chrome axle center nut

const C_CHASSIS_DARK       = rgb(22, 25, 28);   // Main carbon-black chassis deck
const C_CHASSIS_LOWER      = rgb(15, 18, 20);   // Underbelly skid plate
const C_CHASSIS_METAL      = rgb(45, 50, 58);   // Chassis side rails & suspension arms
const C_CHASSIS_HIGHLIGHT  = rgb(60, 68, 78);   // Beveled chassis edge highlight

const C_BATTERY_RED_MAIN   = rgb(235, 35, 35);  // Vibrant high-saturation crimson LiPo body
const C_BATTERY_RED_TOP    = rgb(255, 65, 55);  // Top sunlit battery surface
const C_BATTERY_RED_DARK   = rgb(180, 25, 25);  // Shadowed battery end/creases
const C_STRAP_ORANGE       = rgb(255, 150, 15); // Neon safety-orange battery tie-down strap
const C_LABEL_YELLOW       = rgb(255, 230, 25); // Bright golden-yellow warning label
const C_WIRE_BLUE          = rgb(35, 155, 255); // Electric cobalt blue silicone wire
const C_WIRE_WHITE         = rgb(235, 240, 245); // Balance lead white wiring
const C_PCB_GREEN          = rgb(30, 75, 48);   // Front sensor/driver PCB plate

// Ground truth wheel measurements from GLB:
// FL: Center [+91, +76, -18], FR: Center [+91, -77, -19]
// RL: Center [-91, +76, -14], RR: Center [-91, -77, -16]
const R_TIRE = 36;
const R_RIM  = 26;
const R_HUB  = 12;
const WHEEL_SEGS = 16; // 16 segments for smooth circular wheels

function buildMecanumWheel(cx, cy, cz, isLeft) {
  const yOuter = isLeft ? cy + 22 : cy - 22; // Outer face plane
  const yInner = isLeft ? cy - 22 : cy + 22; // Inner face plane
  
  const outerCenter = addVert(cx, yOuter, cz);
  const innerCenter = addVert(cx, yInner, cz);
  
  const tireRing  = [];
  const rimRing   = [];
  const hubRing   = [];
  const innerRing = [];
  
  for (let s = 0; s < WHEEL_SEGS; s++) {
    const angle = (s * 2 * Math.PI) / WHEEL_SEGS;
    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);
    
    tireRing.push(addVert(cx + R_TIRE * cosA, yOuter, cz + R_TIRE * sinA));
    rimRing.push(addVert(cx + R_RIM * cosA,   yOuter, cz + R_RIM * sinA));
    hubRing.push(addVert(cx + R_HUB * cosA,   yOuter, cz + R_HUB * sinA));
    innerRing.push(addVert(cx + R_TIRE * cosA, yInner, cz + R_TIRE * sinA));
  }
  
  // Outer Face Rings:
  for (let s = 0; s < WHEEL_SEGS; s++) {
    const nxt = (s + 1) % WHEEL_SEGS;
    // Spoke star pattern: alternate bright green spokes and dark recesses
    const isSpoke = (s % 2 === 0);
    const cHub = isSpoke ? C_WHEEL_GREEN_BRIGHT : C_HUB_RECESS;
    const cRim = isSpoke ? C_WHEEL_GREEN_BRIGHT : C_WHEEL_GREEN_MED;
    const cTire = (s % 2 === 0) ? C_ROLLER_BLACK : C_ROLLER_CHARCOAL;
    
    if (isLeft) {
      // Center nut to hub ring
      addTri(outerCenter, hubRing[nxt], hubRing[s], s === 0 ? C_AXLE_NUT : cHub);
      // Hub ring to outer rim ring (green mecanum star spokes!)
      addQuad(hubRing[s], hubRing[nxt], rimRing[nxt], rimRing[s], cRim);
      // Rim to tire outer sidewall
      addQuad(rimRing[s], rimRing[nxt], tireRing[nxt], tireRing[s], cTire);
    } else {
      // Right wheel (reversed winding)
      addTri(outerCenter, hubRing[s], hubRing[nxt], s === 0 ? C_AXLE_NUT : cHub);
      addQuad(hubRing[nxt], hubRing[s], rimRing[s], rimRing[nxt], cRim);
      addQuad(rimRing[nxt], rimRing[s], tireRing[s], tireRing[nxt], cTire);
    }
  }
  
  // Inner Face (facing chassis)
  for (let s = 0; s < WHEEL_SEGS; s++) {
    const nxt = (s + 1) % WHEEL_SEGS;
    if (isLeft) {
      addTri(innerCenter, innerRing[s], innerRing[nxt], C_HUB_RECESS);
    } else {
      addTri(innerCenter, innerRing[nxt], innerRing[s], C_HUB_RECESS);
    }
  }
  
  // Circumferential Mecanum Rollers:
  // Alternating angled roller barrels (black) and green hub dividers!
  const rollOffset = (isLeft ? 1 : (WHEEL_SEGS - 1));
  for (let s = 0; s < WHEEL_SEGS; s++) {
    const nxt = (s + 1) % WHEEL_SEGS;
    const i_s = (s + rollOffset) % WHEEL_SEGS;
    const i_nxt = (nxt + rollOffset) % WHEEL_SEGS;
    const o1 = tireRing[s], o2 = tireRing[nxt];
    const i1 = innerRing[i_s], i2 = innerRing[i_nxt];
    // Alternating mecanum roller vs bracket
    const cTread = (s % 2 === 0) ? C_ROLLER_BLACK : C_WHEEL_GREEN_DARK;
    if (isLeft) {
      addQuad(o1, i1, i2, o2, cTread);
    } else {
      addQuad(o2, i2, i1, o1, cTread);
    }
  }
  
  // Suspension Wishbone / Motor Mount Arm connecting inner wheel to chassis
  const chassisMountY = isLeft ? 40 : -40;
  const m1 = addVert(cx - 15, chassisMountY, cz - 4);
  const m2 = addVert(cx + 15, chassisMountY, cz - 4);
  const m3 = addVert(cx, chassisMountY, cz + 10);
  
  if (isLeft) {
    addTri(m1, m2, innerCenter, C_CHASSIS_METAL);
    addTri(m2, m3, innerCenter, C_CHASSIS_METAL);
    addTri(m3, m1, innerCenter, C_CHASSIS_METAL);
  } else {
    addTri(m2, m1, innerCenter, C_CHASSIS_METAL);
    addTri(m3, m2, innerCenter, C_CHASSIS_METAL);
    addTri(m1, m3, innerCenter, C_CHASSIS_METAL);
  }
}

// 1. Build 4 Mecanum Wheels at exact GLB coordinates
buildMecanumWheel(91, 76, -18, true);   // Front-Left
buildMecanumWheel(91, -77, -19, false); // Front-Right
buildMecanumWheel(-91, 76, -14, true);  // Rear-Left
buildMecanumWheel(-91, -77, -16, false);// Rear-Right

console.log(`After Wheels: ${verts.length} verts, ${tris.length} tris`);

// =========================================================================
// 2. CHASSIS LOWER BELLY & SKID PLATES
// =========================================================================
const bFL = addVert(72, 40, -28);
const bFR = addVert(72, -40, -28);
const bRL = addVert(-72, 40, -28);
const bRR = addVert(-72, -40, -28);
addQuad(bFL, bFR, bRR, bRL, C_CHASSIS_LOWER); // Belly pan

// Front angled ramp
const bF_noseL = addVert(104, 34, -12);
const bF_noseR = addVert(104, -34, -12);
addQuad(bF_noseL, bF_noseR, bFR, bFL, C_CHASSIS_METAL);

// Rear angled ramp
const bR_tailL = addVert(-102, 34, -12);
const bR_tailR = addVert(-102, -34, -12);
addQuad(bRL, bRR, bR_tailR, bR_tailL, C_CHASSIS_METAL);

// =========================================================================
// 3. CHASSIS MAIN UPPER DECK (Carries battery & electronics)
// =========================================================================
const dFL = addVert(65, 36, 4);
const dFR = addVert(65, -36, 4);
const dRL = addVert(-65, 36, 4);
const dRR = addVert(-65, -36, 4);

// Chassis side panels connecting upper deck to lower pan
addQuad(dFL, dRL, bRL, bFL, C_CHASSIS_DARK); // Left side
addQuad(dRR, dFR, bFR, bRR, C_CHASSIS_DARK); // Right side

// Front chassis plate
addQuad(dFL, dFR, bF_noseR, bF_noseL, C_CHASSIS_DARK);
// Rear chassis plate
addQuad(dRR, dRL, bR_tailL, bR_tailR, C_CHASSIS_DARK);

// Exposed upper deck areas (around battery)
const deckFrontL = addVert(48, 36, 4);
const deckFrontR = addVert(48, -36, 4);
const deckRearL  = addVert(-46, 36, 4);
const deckRearR  = addVert(-46, -36, 4);

// Deck sides next to battery
addQuad(deckFrontL, deckRearL, dRL, dFL, C_CHASSIS_HIGHLIGHT);
addQuad(deckRearR, deckFrontR, dFR, dRR, C_CHASSIS_HIGHLIGHT);

// =========================================================================
// 4. FRONT ELECTRONICS TRAY & WIRING HARNESS
// =========================================================================
// Front PCB board deck
const pcbFL = addVert(86, 26, 6);
const pcbFR = addVert(86, -26, 6);
const pcbBL = addVert(50, 26, 6);
const pcbBR = addVert(50, -26, 6);
addQuad(pcbFL, pcbFR, pcbBR, pcbBL, C_PCB_GREEN); // Green PCB plate
addQuad(pcbFL, pcbBL, deckFrontL, dFL, C_CHASSIS_METAL);
addQuad(pcbBR, pcbFR, dFR, deckFrontR, C_CHASSIS_METAL);

// Prominent blue/white power cables routing from battery into PCB
const w1 = addVert(48, 8, 14);
const w2 = addVert(62, 10, 16);
const w3 = addVert(72, 6, 8);
addTri(w1, w2, w3, C_WIRE_BLUE);
const w4 = addVert(48, -8, 14);
const w5 = addVert(62, -10, 16);
const w6 = addVert(72, -6, 8);
addTri(w4, w6, w5, C_WIRE_BLUE);

// Center white balance wire
const w7 = addVert(48, 0, 12);
const w8 = addVert(60, 0, 13);
const w9 = addVert(70, 0, 7);
addTri(w7, w8, w9, C_WIRE_WHITE);

// =========================================================================
// 5. VIBRANT RED LIPO BATTERY PACK (Centered on top deck)
// =========================================================================
// Battery dimensions from GLB: X from -42 to +46, Y from -24 to +24, Z from +4 to +38
const batBL_b = addVert(-42, 24, 4);   // Back Left Bottom
const batBR_b = addVert(-42, -24, 4);  // Back Right Bottom
const batFL_b = addVert(46, 24, 4);    // Front Left Bottom
const batFR_b = addVert(46, -24, 4);   // Front Right Bottom

// Beveled top perimeter for realistic shrink-wrapped battery
const batBL_t = addVert(-38, 22, 36);  // Back Left Top
const batBR_t = addVert(-38, -22, 36); // Back Right Top
const batFL_t = addVert(42, 22, 36);   // Front Left Top
const batFR_t = addVert(42, -22, 36);  // Front Right Top

// Center strap slice vertices on top:
const strL_b = addVert(-4, 24, 4);
const strR_b = addVert(-4, -24, 4);
const strL_t = addVert(-4, 22, 36);
const strR_t = addVert(-4, -22, 36);

const strL2_b = addVert(6, 24, 4);
const strR2_b = addVert(6, -24, 4);
const strL2_t = addVert(6, 22, 36);
const strR2_t = addVert(6, -22, 36);

// Battery Rear Face
addQuad(batBL_b, batBR_b, batBR_t, batBL_t, C_BATTERY_RED_DARK);
// Battery Front Face
addQuad(batFR_b, batFL_b, batFL_t, batFR_t, C_BATTERY_RED_MAIN);

// Battery Left Side (Split by Orange Strap)
addQuad(batFL_b, strL2_b, strL2_t, batFL_t, C_BATTERY_RED_MAIN); // Front section
addQuad(strL2_b, strL_b, strL_t, strL2_t, C_STRAP_ORANGE);       // Orange strap!
addQuad(strL_b, batBL_b, batBL_t, strL_t, C_BATTERY_RED_DARK);   // Rear section

// Battery Right Side (Split by Orange Strap)
addQuad(strR2_b, batFR_b, batFR_t, strR2_t, C_BATTERY_RED_MAIN); // Front section
addQuad(strR_b, strR2_b, strR2_t, strR_t, C_STRAP_ORANGE);       // Orange strap!
addQuad(batBR_b, strR_b, strR_t, batBR_t, C_BATTERY_RED_DARK);   // Rear section

// Battery Top Face (Split into Front, Orange Strap, Label, Rear):
// Front red section
addQuad(batFL_t, strL2_t, strR2_t, batFR_t, C_BATTERY_RED_TOP);
// Center orange hold-down strap!
addQuad(strL2_t, strL_t, strR_t, strR2_t, C_STRAP_ORANGE);

// Label slice
const lblL_t = addVert(-18, 16, 36);
const lblR_t = addVert(-18, -16, 36);
const lblL2_t = addVert(-30, 16, 36);
const lblR2_t = addVert(-30, -16, 36);
addQuad(strL_t, lblL_t, lblR_t, strR_t, C_BATTERY_RED_TOP);
addQuad(lblL_t, lblL2_t, lblR2_t, lblR_t, C_LABEL_YELLOW); // Golden safety warning label
addQuad(lblL2_t, batBL_t, batBR_t, lblR2_t, C_BATTERY_RED_DARK);

// =========================================================================
// 6. REAR CHASSIS DECK & MOUNTING PLATE
// =========================================================================
const rearDeckL = addVert(-78, 28, 6);
const rearDeckR = addVert(-78, -28, 6);
addQuad(deckRearL, deckRearR, rearDeckR, rearDeckL, C_CHASSIS_DARK);
addQuad(rearDeckL, rearDeckR, bR_tailR, bR_tailL, C_CHASSIS_METAL);

console.log(`Total Mecanum Rover Model: ${verts.length} vertices, ${tris.length} triangles`);

// =========================================================================
// 7. WRITE S3_REMOTE/CAR_MODEL.H
// =========================================================================
let h = `#pragma once\n#include <stdint.h>\n#include <pgmspace.h>\n// Authentic Mecanum Rover 3D Model (Precision-matched to Tripo GLB)\n`;
h += `// Vertices: ${verts.length}, Triangles: ${tris.length}\n`;
h += `#define CAR_VERT_COUNT ${verts.length}\n`;
h += `#define CAR_TRI_COUNT ${tris.length}\n`;
h += `static const int8_t CAR_VERTS[CAR_VERT_COUNT][3] PROGMEM = {\n  ` + verts.map(v => `{${v[0]}, ${v[1]}, ${v[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_TRIS[CAR_TRI_COUNT][3] PROGMEM = {\n  ` + tris.map(f => `{${f[0]}, ${f[1]}, ${f[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_COLORS[CAR_TRI_COUNT] PROGMEM = {\n  ` + colors.map(c => '0x' + c.toString(16).toUpperCase().padStart(4, '0')).join(', ') + `\n};\n`;

const targetHeader = 'C:/Users/dell/Desktop/rover/s3_remote/car_model.h';
fs.writeFileSync(targetHeader, h);
console.log(`Successfully written ${targetHeader} (${h.length} bytes, ${tris.length} tris, ${verts.length} verts)`);

// =========================================================================
// 8. RENDER MULTI-ANGLE VISUAL VALIDATION
// =========================================================================
const cT = 0.8387, sT = 0.5446; // 33 deg elevation
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderAngle(yawDeg, pitchDeg, title) {
  const radY = yawDeg * Math.PI / 180;
  const radP = pitchDeg * Math.PI / 180;
  const cosY = Math.cos(radY), sinY = Math.sin(radY);
  const cosP = Math.cos(radP), sinP = Math.sin(radP);

  const V = verts.map(([bx, by, bz]) => {
    const x1 = bx * cosP - bz * sinP;
    const z1 = bx * sinP + bz * cosP;
    const x2 = x1 * cosY - by * sinY;
    const y2 = x1 * sinY + by * cosY;
    return toView(x2 / 127, y2 / 127, z1 / 127);
  });

  const sortedTris = tris.map((f, i) => {
    const va = V[f[0]], vb = V[f[1]], vc = V[f[2]];
    const d = (va[2] + vb[2] + vc[2]) / 3;
    
    // Normal & flat shading
    const ux = vb[0] - va[0], uy = vb[1] - va[1], ud = vb[2] - va[2];
    const wx = vc[0] - va[0], wy = vc[1] - va[1], wd = vc[2] - va[2];
    const nx = uy * wd - ud * wy, ny = ud * wx - ux * wd, nd = ux * wy - uy * wx;
    const nl = Math.sqrt(nx*nx + ny*ny + nd*nd) + 1e-6;
    const dot = Math.abs((-0.4*nx + 0.6*ny - 0.7*nd) / nl);
    const shade = Math.min(1.0, 0.68 + 0.32 * dot);
    
    const col565 = colors[i];
    const r5 = (col565 >> 11) & 31;
    const g6 = (col565 >> 5) & 63;
    const b5 = col565 & 31;
    const r = Math.round((r5 * 255 / 31) * shade);
    const g = Math.round((g6 * 255 / 63) * shade);
    const b = Math.round((b5 * 255 / 31) * shade);
    
    return { f, d, r, g, b };
  }).sort((a, b) => b.d - a.d);

  const cx = 150, cy = 130, scale = 110;
  let polys = '';
  for (const t of sortedTris) {
    const va = V[t.f[0]], vb = V[t.f[1]], vc = V[t.f[2]];
    const p1 = `${(cx + va[0] * scale).toFixed(1)},${(cy - va[1] * scale).toFixed(1)}`;
    const p2 = `${(cx + vb[0] * scale).toFixed(1)},${(cy - vb[1] * scale).toFixed(1)}`;
    const p3 = `${(cx + vc[0] * scale).toFixed(1)},${(cy - vc[1] * scale).toFixed(1)}`;
    polys += `<polygon points="${p1} ${p2} ${p3}" fill="rgb(${t.r},${t.g},${t.b})" stroke="rgba(0,0,0,0.15)" stroke-width="0.3"/>`;
  }
  return `<g><text x="10" y="20" fill="#38bdf8" font-size="12" font-family="monospace">${title}</text>${polys}</g>`;
}

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="300" style="background:#070a13;">
  <g transform="translate(0,0)">${renderAngle(0, 0, "Chase Cam (Yaw 0 deg)")}</g>
  <g transform="translate(300,0)">${renderAngle(45, 15, "Rear 3/4 (Yaw 45 deg, Pitch 15)")}</g>
  <g transform="translate(600,0)">${renderAngle(90, 0, "Side View (Yaw 90 deg)")}</g>
  <g transform="translate(900,0)">${renderAngle(180, 0, "Front View (Yaw 180 deg)")}</g>
</svg>`;

const previewImg = 'C:/Users/dell/Desktop/rover/tools/accurate_mecanum_preview.png';
await sharp(Buffer.from(svg)).png().toFile(previewImg);
console.log(`Rendered preview to ${previewImg}`);

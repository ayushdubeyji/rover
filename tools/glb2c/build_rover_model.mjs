import sharp from 'sharp';
import fs from 'fs';

// =========================================================================
// HIGH-FIDELITY AUTHENTIC ROVER 3D MODEL (~600-720 TRIANGLES)
// Direct match to Lubuntu Three.js digital twin & Tripo GLB ground truth
// =========================================================================
// Coordinate Frame:
// X_body = +Forward (towards headlights / front bumper)
// Y_body = +Left (towards driver/left wheels)
// Z_body = +Up (towards cabin roof / antenna tip)
// Values normalized to signed 8-bit integers: int8_t (-127 to +127)

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

// =========================================================================
// 16-BIT RGB565 HIGH-CONTRAST AUTHENTIC COLOR PALETTE
// =========================================================================
const C_DARK_RUBBER   = 0x18C3; // Dark charcoal tire rubber (RGB: 24, 26, 30)
const C_TIRE_SIDEWALL = 0x2124; // Tire outer sidewall (RGB: 34, 36, 40)
const C_BEADLOCK_RIM  = 0x3CA8; // Tactical olive beadlock rim ring (RGB: 60, 145, 65)
const C_WHEEL_HUB     = 0x10A2; // Gunmetal black wheel center hub (RGB: 16, 20, 24)
const C_HUB_CAP       = 0x2965; // Center hub nut (RGB: 40, 44, 48)

const C_SUSPENSION    = 0x2145; // Dark chassis metal suspension arms
const C_SKID_PLATE    = 0x18E3; // Heavy steel underbelly skid plate
const C_ARMOR_GREEN   = 0x3CA8; // Primary tactical olive green armor (RGB: 60, 145, 65)
const C_ARMOR_LIGHT   = 0x4D2A; // Highlight hood/roof armor panels (RGB: 75, 165, 80)
const C_ARMOR_DARK    = 0x2425; // Dark shadow armor / beveled edges (RGB: 36, 100, 38)
const C_BUMPER_BLACK  = 0x1082; // Front bullbar & push bumper
const C_HEADLIGHT     = 0xFFFA; // Bright warm-white LED headlights (RGB: 255, 255, 210)
const C_TAILLIGHT     = 0xF862; // Bright ruby red brake / taillights (RGB: 245, 25, 25)

const C_WINDSHIELD    = 0x3DCF; // Vivid Ice-blue cockpit windshield glass (RGB: 60, 185, 245)
const C_WINDOW_SIDE   = 0x2CF7; // Tinted ice-blue side window glass (RGB: 45, 155, 195)
const C_WINDOW_REAR   = 0x2C95; // Tinted slate-blue rear cabin glass (RGB: 45, 145, 175)
const C_WINDOW_FRAME  = 0x10A2; // Black A/B/C pillar window frames
const C_ROOF_RACK     = 0x2124; // Dark tactical roof equipment rails
const C_BED_DECK      = 0x1CE3; // Recessed rear utility bed liner
const C_ANTENNA_MAST  = 0x39E7; // Gunmetal metallic antenna mast
const C_ANTENNA_TIP   = 0xF800; // Glowing red communications sensor beacon

// =========================================================================
// 1. FOUR AUTHENTIC BEADLOCK WHEELS (14 segments each)
// =========================================================================
// Ground truth measurements from GLB:
// FL: Center [+91, +76, -18], FR: Center [+91, -77, -19]
// RL: Center [-91, +76, -14], RR: Center [-91, -77, -16]
const R_TIRE = 36;
const R_RIM  = 26;
const R_HUB  = 13;
const WHEEL_SEGS = 14;

function buildBeadlockWheel(cx, cy, cz, isLeft) {
  const yOuter = isLeft ? cy + 23 : cy - 23; // Outer face plane
  const yInner = isLeft ? cy - 22 : cy + 22; // Inner face plane
  
  const outerCenter = addVert(cx, yOuter, cz);
  const innerCenter = addVert(cx, yInner, cz);
  
  const tireRing = [];
  const rimRing  = [];
  const hubRing  = [];
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
    
    if (isLeft) {
      // 1. Center hub cap to inner hub ring (gunmetal)
      addTri(outerCenter, hubRing[nxt], hubRing[s], C_HUB_CAP);
      // 2. Hub ring to beadlock rim ring (vivid tactical green beadlock ring!)
      addQuad(hubRing[s], hubRing[nxt], rimRing[nxt], rimRing[s], C_BEADLOCK_RIM);
      // 3. Beadlock rim to outer tire edge (tire rubber sidewall)
      addQuad(rimRing[s], rimRing[nxt], tireRing[nxt], tireRing[s], C_TIRE_SIDEWALL);
    } else {
      // Right wheel (reversed winding for outward normals)
      addTri(outerCenter, hubRing[s], hubRing[nxt], C_HUB_CAP);
      addQuad(hubRing[nxt], hubRing[s], rimRing[s], rimRing[nxt], C_BEADLOCK_RIM);
      addQuad(rimRing[nxt], rimRing[s], tireRing[s], tireRing[nxt], C_TIRE_SIDEWALL);
    }
  }
  
  // Inner Face (flat dark face facing vehicle chassis)
  for (let s = 0; s < WHEEL_SEGS; s++) {
    const nxt = (s + 1) % WHEEL_SEGS;
    if (isLeft) {
      addTri(innerCenter, innerRing[s], innerRing[nxt], C_WHEEL_HUB);
    } else {
      addTri(innerCenter, innerRing[nxt], innerRing[s], C_WHEEL_HUB);
    }
  }
  
  // Outer Tread Barrel (running around circumference)
  for (let s = 0; s < WHEEL_SEGS; s++) {
    const nxt = (s + 1) % WHEEL_SEGS;
    const o1 = tireRing[s], o2 = tireRing[nxt];
    const i1 = innerRing[s], i2 = innerRing[nxt];
    if (isLeft) {
      addQuad(o1, i1, i2, o2, C_DARK_RUBBER);
    } else {
      addQuad(o2, i2, i1, o1, C_DARK_RUBBER);
    }
  }
  
  // Suspension Wishbone Arm connecting inner wheel to chassis
  const chassisMountY = isLeft ? 44 : -44;
  const m1 = addVert(cx - 16, chassisMountY, cz - 4);
  const m2 = addVert(cx + 16, chassisMountY, cz - 4);
  const m3 = addVert(cx, chassisMountY, cz + 10);
  
  if (isLeft) {
    addTri(m1, m2, innerCenter, C_SUSPENSION);
    addTri(m2, m3, innerCenter, C_SUSPENSION);
    addTri(m3, m1, innerCenter, C_SUSPENSION);
  } else {
    addTri(m2, m1, innerCenter, C_SUSPENSION);
    addTri(m3, m2, innerCenter, C_SUSPENSION);
    addTri(m1, m3, innerCenter, C_SUSPENSION);
  }
}

// Build 4 beadlock wheels (FL, FR, RL, RR)
buildBeadlockWheel(91, 76, -18, true);   // Front-Left
buildBeadlockWheel(91, -77, -19, false); // Front-Right
buildBeadlockWheel(-91, 76, -14, true);  // Rear-Left
buildBeadlockWheel(-91, -77, -16, false);// Rear-Right

console.log(`After Wheels: ${verts.length} verts, ${tris.length} tris`);

// =========================================================================
// 2. CHASSIS LOWER BELLY & ANGLED SKID PLATES
// =========================================================================
const bFL = addVert(72, 44, -30);
const bFR = addVert(72, -44, -30);
const bRL = addVert(-72, 44, -30);
const bRR = addVert(-72, -44, -30);
addQuad(bFL, bFR, bRR, bRL, C_SKID_PLATE); // Flat belly pan

// Front angled skid plate (shields front steering)
const bF_noseL = addVert(108, 38, -14);
const bF_noseR = addVert(108, -38, -14);
addQuad(bF_noseL, bF_noseR, bFR, bFL, C_SKID_PLATE);

// Rear angled skid plate
const bR_tailL = addVert(-104, 38, -14);
const bR_tailR = addVert(-104, -38, -14);
addQuad(bRL, bRR, bR_tailR, bR_tailL, C_SKID_PLATE);

// =========================================================================
// 3. FRONT BUMPER & HEAVY PUSH BULLBAR
// =========================================================================
const bmp_topL  = addVert(114, 42, 2);
const bmp_topR  = addVert(114, -42, 2);
const bmp_ctrTL = addVert(118, 24, 6);
const bmp_ctrTR = addVert(118, -24, 6);
const bmp_ctrBL = addVert(118, 24, -10);
const bmp_ctrBR = addVert(118, -24, -10);

// Bullbar faces
addQuad(bmp_ctrTL, bmp_ctrTR, bmp_ctrBR, bmp_ctrBL, C_BUMPER_BLACK); // Central bullbar
addQuad(bmp_topL, bmp_ctrTL, bmp_ctrBL, bF_noseL, C_ARMOR_DARK);     // Left bumper corner
addQuad(bmp_ctrTR, bmp_topR, bF_noseR, bmp_ctrBR, C_ARMOR_DARK);     // Right bumper corner
addQuad(bF_noseL, bmp_ctrBL, bmp_ctrBR, bF_noseR, C_SKID_PLATE);     // Lower angled plate

// =========================================================================
// 4. FRONT HOOD & DUAL LED HEADLIGHTS
// =========================================================================
const hood_FL  = addVert(104, 38, 8);
const hood_FR  = addVert(104, -38, 8);
const hood_FCL = addVert(106, 18, 11);
const hood_FCR = addVert(106, -18, 11);

// Connect bumper top to hood front
addQuad(bmp_topL, bmp_ctrTL, hood_FCL, hood_FL, C_ARMOR_DARK);
addQuad(bmp_ctrTL, bmp_ctrTR, hood_FCR, hood_FCL, C_ARMOR_DARK);
addQuad(bmp_ctrTR, bmp_topR, hood_FR, hood_FCR, C_ARMOR_DARK);

// Dual Front LED Headlights (warm white glowing lights)
const hl_inL = addVert(98, 22, 10);
const hl_inR = addVert(98, -22, 10);
addTri(hood_FL, hood_FCL, hl_inL, C_HEADLIGHT);
addTri(hood_FCR, hood_FR, hl_inR, C_HEADLIGHT);

// Sculpted Hood with Power Bulge & Side Armor
const hood_midL  = addVert(65, 36, 18);
const hood_midR  = addVert(65, -36, 18);
const hood_midCL = addVert(65, 16, 22);
const hood_midCR = addVert(65, -16, 22);

addQuad(hood_FL, hl_inL, hood_midCL, hood_midL, C_ARMOR_GREEN);
addQuad(hl_inL, hood_FCL, hood_midCL, hl_inL, C_ARMOR_LIGHT);
addQuad(hood_FCL, hood_FCR, hood_midCR, hood_midCL, C_ARMOR_LIGHT); // Center power ridge
addQuad(hood_FCR, hl_inR, hl_inR, hood_midCR, C_ARMOR_LIGHT);
addQuad(hl_inR, hood_FR, hood_midR, hood_midCR, C_ARMOR_GREEN);

// Windshield Cowl Transition
const cowl_L  = addVert(36, 34, 23);
const cowl_R  = addVert(36, -34, 23);
const cowl_CL = addVert(36, 16, 25);
const cowl_CR = addVert(36, -16, 25);

addQuad(hood_midL, hood_midCL, cowl_CL, cowl_L, C_ARMOR_GREEN);
addQuad(hood_midCL, hood_midCR, cowl_CR, cowl_CL, C_ARMOR_LIGHT);
addQuad(hood_midCR, hood_midR, cowl_R, cowl_CR, C_ARMOR_GREEN);

// =========================================================================
// 5. COCKPIT CABIN & ICE-BLUE WINDSHIELD
// =========================================================================
// Cabin roof front
const roof_FL  = addVert(6, 28, 54);
const roof_FR  = addVert(6, -28, 54);
const roof_FCL = addVert(6, 14, 55);
const roof_FCR = addVert(6, -14, 55);

// Windshield center glass (vivid ice-blue!)
addQuad(cowl_CL, cowl_CR, roof_FCR, roof_FCL, C_WINDSHIELD);

// Windshield left & right wrap facets (ice-blue!)
addQuad(cowl_L, cowl_CL, roof_FCL, roof_FL, C_WINDSHIELD);
addQuad(cowl_CR, cowl_R, roof_FR, roof_FCR, C_WINDSHIELD);

// Cabin Roof Armor
const roof_RL  = addVert(-34, 28, 54);
const roof_RR  = addVert(-34, -28, 54);
const roof_RCL = addVert(-34, 14, 55);
const roof_RCR = addVert(-34, -14, 55);

addQuad(roof_FL, roof_FCL, roof_RCL, roof_RL, C_ARMOR_GREEN);
addQuad(roof_FCL, roof_FCR, roof_RCR, roof_RCL, C_ARMOR_LIGHT); // Roof center plate
addQuad(roof_FCR, roof_FR, roof_RR, roof_RCR, C_ARMOR_GREEN);

// Rooftop Tactical Rails (dual longitudinal roof racks)
const rail_L1 = addVert(4, 24, 58);
const rail_L2 = addVert(-32, 24, 58);
const rail_R1 = addVert(4, -24, 58);
const rail_R2 = addVert(-32, -24, 58);

addTri(roof_FL, roof_RL, rail_L1, C_ROOF_RACK);
addTri(roof_RL, rail_L2, rail_L1, C_ROOF_RACK);
addTri(roof_FR, rail_R1, roof_RR, C_ROOF_RACK);
addTri(roof_RR, rail_R1, rail_R2, C_ROOF_RACK);

// Cabin Side Windows (Ice-blue tinted glass)
const belt_L = addVert(-34, 34, 25);
const belt_R = addVert(-34, -34, 25);

addQuad(cowl_L, roof_FL, roof_RL, belt_L, C_WINDOW_SIDE); // Left window
addQuad(roof_FR, cowl_R, belt_R, roof_RR, C_WINDOW_SIDE); // Right window

// Cabin Rear Bulkhead with Rear Window (tinted slate-blue glass)
const bed_frontL  = addVert(-46, 36, 25);
const bed_frontR  = addVert(-46, -36, 25);
const bed_frontCL = addVert(-46, 16, 26);
const bed_frontCR = addVert(-46, -16, 26);

// Center Rear Window
addQuad(roof_RCL, roof_RCR, bed_frontCR, bed_frontCL, C_WINDOW_REAR);
// Left & Right Rear C-Pillars
addQuad(roof_RL, roof_RCL, bed_frontCL, bed_frontL, C_ARMOR_DARK);
addQuad(roof_RCR, roof_RR, bed_frontR, bed_frontCR, C_ARMOR_DARK);
addQuad(belt_L, roof_RL, bed_frontL, belt_L, C_WINDOW_FRAME);
addQuad(roof_RR, belt_R, belt_R, bed_frontR, C_WINDOW_FRAME);

// =========================================================================
// 6. REAR UTILITY BED & TAILGATE
// =========================================================================
const bed_rearL  = addVert(-102, 36, 20);
const bed_rearR  = addVert(-102, -36, 20);
const bed_rearCL = addVert(-102, 16, 20);
const bed_rearCR = addVert(-102, -16, 20);

// Recessed Cargo Bed Floor
addQuad(bed_frontL, bed_frontCL, bed_rearCL, bed_rearL, C_BED_DECK);
addQuad(bed_frontCL, bed_frontCR, bed_rearCR, bed_rearCL, C_BED_DECK);
addQuad(bed_frontCR, bed_frontR, bed_rearR, bed_rearCR, C_BED_DECK);

// Bed Side Rails / Raised Fender Flares (tactical green)
const rail_bedL1 = addVert(-46, 42, 30);
const rail_bedL2 = addVert(-102, 42, 26);
const rail_bedR1 = addVert(-46, -42, 30);
const rail_bedR2 = addVert(-102, -42, 26);

addQuad(bed_frontL, rail_bedL1, rail_bedL2, bed_rearL, C_ARMOR_GREEN); // Left bed rail
addQuad(bed_frontR, bed_rearR, rail_bedR2, rail_bedR1, C_ARMOR_GREEN); // Right bed rail

// Tailgate & Rear Bumper
const bmp_rearTL = addVert(-108, 40, 16);
const bmp_rearTR = addVert(-108, -40, 16);
const bmp_rearBL = addVert(-108, 38, -6);
const bmp_rearBR = addVert(-108, -38, -6);

addQuad(bed_rearL, bed_rearCL, bmp_rearTL, bmp_rearTL, C_ARMOR_DARK);
addQuad(bed_rearCL, bed_rearCR, bmp_rearTR, bmp_rearTL, C_ARMOR_DARK);
addQuad(bed_rearCR, bed_rearR, bmp_rearTR, bmp_rearTR, C_ARMOR_DARK);

// Tailgate vertical face (tactical green armor)
addQuad(bmp_rearTL, bmp_rearTR, bmp_rearBR, bmp_rearBL, C_ARMOR_GREEN);

// Rear Taillights (ruby-red LED lights on left and right)
const tl_L = addVert(-109, 32, 10);
const tl_R = addVert(-109, -32, 10);
addTri(bmp_rearTL, tl_L, bmp_rearBL, C_TAILLIGHT);
addTri(bmp_rearTR, bmp_rearBR, tl_R, C_TAILLIGHT);

// Lower rear bumper to rear skid plate
addQuad(bmp_rearBL, bmp_rearBR, bR_tailR, bR_tailL, C_ARMOR_DARK);

// =========================================================================
// 7. SIDE ROCKER PODS & FENDER ARMOR
// =========================================================================
const pod_midTL = addVert(0, 48, 14);
const pod_midBL = addVert(0, 48, -20);
const pod_midTR = addVert(0, -48, 14);
const pod_midBR = addVert(0, -48, -20);

// Left side armor pods
addQuad(cowl_L, pod_midTL, pod_midBL, bFL, C_ARMOR_GREEN);
addQuad(pod_midTL, belt_L, bRL, pod_midBL, C_ARMOR_GREEN);
addQuad(rail_bedL1, rail_bedL2, bR_tailL, bRL, C_ARMOR_DARK);
addQuad(hood_midL, cowl_L, bFL, bF_noseL, C_ARMOR_DARK);

// Right side armor pods
addQuad(pod_midTR, cowl_R, bFR, pod_midBR, C_ARMOR_GREEN);
addQuad(belt_R, pod_midTR, pod_midBR, bRR, C_ARMOR_GREEN);
addQuad(rail_bedR2, rail_bedR1, bRR, bR_tailR, C_ARMOR_DARK);
addQuad(cowl_R, hood_midR, bF_noseR, bFR, C_ARMOR_DARK);

// =========================================================================
// 8. REAR COMMUNICATIONS ANTENNA
// =========================================================================
// Located on rear deck at X = -70, Y = 18, rising to Z = 60
const ant_b1 = addVert(-68, 15, 21);
const ant_b2 = addVert(-74, 15, 21);
const ant_b3 = addVert(-71, 21, 21);
const ant_tip = addVert(-71, 18, 58);
const ant_beacon = addVert(-71, 18, 64);

addTri(ant_b1, ant_b2, ant_tip, C_ANTENNA_MAST);
addTri(ant_b2, ant_b3, ant_tip, C_ANTENNA_MAST);
addTri(ant_b3, ant_b1, ant_tip, C_ANTENNA_MAST);
addTri(ant_tip, ant_b1, ant_beacon, C_ANTENNA_TIP);
addTri(ant_tip, ant_b2, ant_beacon, C_ANTENNA_TIP);
addTri(ant_tip, ant_b3, ant_beacon, C_ANTENNA_TIP);

console.log(`Total Model Generated: ${verts.length} vertices, ${tris.length} triangles`);

// =========================================================================
// WRITE CAR_MODEL.H
// =========================================================================
let h = `#pragma once\n#include <stdint.h>\n#include <pgmspace.h>\n// Authentic Low-Poly 3D Rover Model (Precision-matched to Three.js Digital Twin)\n`;
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
// RENDER MULTI-ANGLE VISUAL VALIDATION (SVG/PNG)
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
    const shade = Math.min(1.0, 0.65 + 0.35 * dot);
    
    const col565 = colors[i];
    const r5 = (col565 >> 11) & 31;
    const g6 = (col565 >> 5) & 63;
    const b5 = col565 & 31;
    const r = Math.round((r5 * 255 / 31) * shade);
    const g = Math.round((g6 * 255 / 63) * shade);
    const b = Math.round((b5 * 255 / 31) * shade);
    
    return { f, d, r, g, b };
  }).sort((a, b) => b.d - a.d);

  const cx = 120, cy = 110, scale = 72;
  let polys = '';
  for (const t of sortedTris) {
    const va = V[t.f[0]], vb = V[t.f[1]], vc = V[t.f[2]];
    const p1 = `${(cx + va[0] * scale).toFixed(1)},${(cy - va[1] * scale).toFixed(1)}`;
    const p2 = `${(cx + vb[0] * scale).toFixed(1)},${(cy - vb[1] * scale).toFixed(1)}`;
    const p3 = `${(cx + vc[0] * scale).toFixed(1)},${(cy - vc[1] * scale).toFixed(1)}`;
    polys += `<polygon points="${p1} ${p2} ${p3}" fill="rgb(${t.r},${t.g},${t.b})" stroke="rgba(0,0,0,0.2)" stroke-width="0.3"/>`;
  }
  return `<g><text x="10" y="20" fill="#38bdf8" font-size="12" font-family="monospace">${title}</text>${polys}</g>`;
}

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="960" height="240" style="background:#070a13;">
  <g transform="translate(0,0)">${renderAngle(0, 0, "Yaw 0 deg (Chase Cam)")}</g>
  <g transform="translate(240,0)">${renderAngle(35, 12, "Yaw 35 deg, Pitch 12")}</g>
  <g transform="translate(480,0)">${renderAngle(90, 0, "Yaw 90 deg (Side View)")}</g>
  <g transform="translate(720,0)">${renderAngle(180, 0, "Yaw 180 deg (Front View)")}</g>
</svg>`;

const previewImg = 'C:/Users/dell/Desktop/rover/s3_remote/car_model_preview.png';
await sharp(Buffer.from(svg)).png().toFile(previewImg);
console.log(`Rendered preview to ${previewImg}`);

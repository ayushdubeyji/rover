import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(glbPath);

const mat = doc.getRoot().listMaterials()[0];
const tex = mat.getBaseColorTexture();
const imgBuf = Buffer.from(tex.getImage());
const { data: texData, info: texInfo } = await sharp(imgBuf).resize(1024, 1024).removeAlpha().raw().toBuffer({ resolveWithObject: true });
const texW = texInfo.width, texH = texInfo.height;

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const idx = prim.getIndices();
const uva = prim.getAttribute('TEXCOORD_0');

const flatPos = new Float32Array(pos.getArray());
const indices = new Uint32Array(idx.getArray());

// Multi-stage topology-preserving decimation
console.log('Stage 1 decimation: strict simplification to 6000 triangles...');
let [s1] = MeshoptSimplifier.simplify(indices, flatPos, 3, 6000 * 3, 0.03);

console.log('Stage 2 decimation: sloppy simplification to 312 triangles...');
let [sLow] = MeshoptSimplifier.simplifySloppy(s1, flatPos, 3, null, 300 * 3, 0.12);

// Bounding box computation
let mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
for (let i = 0; i < pos.getCount(); i++) {
  const p = pos.getElement(i, []);
  for (let a = 0; a < 3; a++) {
    mn[a] = Math.min(mn[a], p[a]);
    mx[a] = Math.max(mx[a], p[a]);
  }
}
const ctr = mn.map((m, a) => (m + mx[a]) / 2);
const maxHalf = Math.max(...mx.map((x, a) => x - mn[a])) / 2;

// Used vertices and remapping into body frame:
// X_body = (X_gltf - ctrX)/maxHalf * 127  (Forward)
// Y_body = -(Z_gltf - ctrZ)/maxHalf * 127 (Left)
// Z_body = (Y_gltf - ctrY)/maxHalf * 127  (Up)
const used = new Map();
const outV = [];
const outUV = [];
const remap = vi => {
  if (!used.has(vi)) {
    used.set(vi, outV.length);
    const p = pos.getElement(vi, []);
    outV.push([
      Math.max(-127, Math.min(127, Math.round((p[0] - ctr[0]) / maxHalf * 127))),
      Math.max(-127, Math.min(127, Math.round(-(p[2] - ctr[2]) / maxHalf * 127))),
      Math.max(-127, Math.min(127, Math.round((p[1] - ctr[1]) / maxHalf * 127)))
    ]);
    outUV.push(uva.getElement(vi, []));
  }
  return used.get(vi);
};

const outF = [];
for (let i = 0; i < sLow.length; i += 3) {
  outF.push([remap(sLow[i]), remap(sLow[i+1]), remap(sLow[i+2])]);
}

// Clean semantic shading & color assignment
const faceColors = outF.map(f => {
  const v0 = outV[f[0]], v1 = outV[f[1]], v2 = outV[f[2]];
  const bx = (v0[0] + v1[0] + v2[0]) / 3;
  const by = (v0[1] + v1[1] + v2[1]) / 3;
  const bz = (v0[2] + v1[2] + v2[2]) / 3;

  // Sample texture
  let u = (outUV[f[0]][0] + outUV[f[1]][0] + outUV[f[2]][0]) / 3;
  let v = (outUV[f[0]][1] + outUV[f[1]][1] + outUV[f[2]][1]) / 3;
  u = ((u % 1) + 1) % 1;
  v = ((v % 1) + 1) % 1;
  let px = Math.min(texW - 1, Math.max(0, Math.floor(u * texW)));
  let py = Math.min(texH - 1, Math.max(0, Math.floor(v * texH)));
  let pIdx = (py * texW + px) * 3;
  let r = texData[pIdx], g = texData[pIdx + 1], b = texData[pIdx + 2];

  // Semantic rules for authentic rover appearance:
  const isWheel = Math.abs(by) > 48 && bz < -5;
  const isHeadlight = bx > 75 && Math.abs(by) > 20 && bz > -20;
  const isTaillight = bx < -75 && Math.abs(by) > 20 && bz > -20;
  const isWindshield = bz > 18 && Math.abs(bx) < 35 && Math.abs(by) < 32;

  if (isWheel) {
    // Charcoal black tires with tread tone
    r = 24; g = 26; b = 30;
  } else if (isHeadlight) {
    // Bright LED headlight warm-white
    r = 250; g = 250; b = 220;
  } else if (isTaillight) {
    // Ruby red brake/taillights
    r = 220; g = 30; b = 30;
  } else if (isWindshield) {
    // Tinted cockpit canopy (dark cyan/slate glass)
    r = 35; g = 135; b = 175;
  } else {
    // Body armor: tactical rover olive green with clean contrast
    if (g > r && g > b) {
      r = Math.min(255, Math.max(30, Math.round(r * 1.15)));
      g = Math.min(255, Math.max(90, Math.round(g * 1.30)));
      b = Math.min(255, Math.max(30, Math.round(b * 1.15)));
    } else {
      // Dark chassis / metal struts
      r = 40; g = 50; b = 48;
    }
  }

  const c565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
  return { r, g, b, c565 };
});

console.log(`Generated ${outV.length} vertices, ${outF.length} triangles`);

// Render visual validation
const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderAngle(yawDeg, pitchDeg, title) {
  const radY = yawDeg * Math.PI / 180;
  const radP = pitchDeg * Math.PI / 180;
  const cosY = Math.cos(radY), sinY = Math.sin(radY);
  const cosP = Math.cos(radP), sinP = Math.sin(radP);

  const V = outV.map(([bx, by, bz]) => {
    const x1 = bx * cosP - bz * sinP;
    const z1 = bx * sinP + bz * cosP;
    const x2 = x1 * cosY - by * sinY;
    const y2 = x1 * sinY + by * cosY;
    return toView(x2 / 127, y2 / 127, z1 / 127);
  });

  const tris = outF.map((f, i) => ({
    f, i,
    d: (V[f[0]][2] + V[f[1]][2] + V[f[2]][2]) / 3,
    col: faceColors[i]
  })).sort((a, b) => a.d - b.d);

  let polys = '';
  const centerX = 120, centerY = 110, scale = 75;
  for (const t of tris) {
    const va = V[t.f[0]], vb = V[t.f[1]], vc = V[t.f[2]];
    const p1 = `${(centerX + va[0] * scale).toFixed(1)},${(centerY - va[1] * scale).toFixed(1)}`;
    const p2 = `${(centerX + vb[0] * scale).toFixed(1)},${(centerY - vb[1] * scale).toFixed(1)}`;
    const p3 = `${(centerX + vc[0] * scale).toFixed(1)},${(centerY - vc[1] * scale).toFixed(1)}`;
    const { r, g, b } = t.col;
    polys += `<polygon points="${p1} ${p2} ${p3}" fill="rgb(${r},${g},${b})" stroke="rgba(0,0,0,0.35)" stroke-width="0.3"/>`;
  }
  return `<g><text x="10" y="20" fill="#38bdf8" font-size="12" font-family="monospace">${title}</text>${polys}</g>`;
}

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="960" height="220" style="background:#070a13;">
  <g transform="translate(0,0)">${renderAngle(0, 0, "Yaw 0 deg (Chase Cam)")}</g>
  <g transform="translate(240,0)">${renderAngle(35, 12, "Yaw 35 deg, Pitch 12")}</g>
  <g transform="translate(480,0)">${renderAngle(90, 0, "Yaw 90 deg (Side View)")}</g>
  <g transform="translate(720,0)">${renderAngle(180, 0, "Yaw 180 deg (Front View)")}</g>
</svg>`;

const previewPath = 'C:/Users/dell/Desktop/rover/tools/real_car_preview.png';
await sharp(Buffer.from(svg)).png().toFile(previewPath);
console.log(`Saved preview to ${previewPath}`);

// Write authentic car_model.h
let h = `#pragma once\n#include <stdint.h>\n#include <pgmspace.h>\n// Authentic Textured 3D Car Model (Directly decoded from Tripo GLB)\n`;
h += `// Vertices: ${outV.length}, Triangles: ${outF.length}\n`;
h += `#define CAR_VERT_COUNT ${outV.length}\n`;
h += `#define CAR_TRI_COUNT ${outF.length}\n`;
h += `static const int8_t CAR_VERTS[CAR_VERT_COUNT][3] PROGMEM = {\n  ` + outV.map(v => `{${v[0]}, ${v[1]}, ${v[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_TRIS[CAR_TRI_COUNT][3] PROGMEM = {\n  ` + outF.map(f => `{${f[0]}, ${f[1]}, ${f[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_COLORS[CAR_TRI_COUNT] PROGMEM = {\n  ` + faceColors.map(c => '0x' + c.c565.toString(16).toUpperCase().padStart(4, '0')).join(', ') + `\n};\n`;

const targetHeader = 'C:/Users/dell/Desktop/rover/s3_remote/car_model.h';
fs.writeFileSync(targetHeader, h);
console.log(`Successfully written ${targetHeader} (${h.length} bytes, ${outF.length} tris, ${outV.length} verts)`);

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

const mulMat = (a, b) => {
  const o = new Array(16).fill(0);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) for (let k = 0; k < 4; k++) o[c*4+r] += a[k*4+r] * b[c*4+k];
  return o;
};

const verts = [], uvs = [], tris = [];
const mats = [];
const triMat = [];

async function walk(node, parent) {
  const m = mulMat(parent, node.getMatrix());
  const mesh = node.getMesh();
  if (mesh) {
    for (const prim of mesh.listPrimitives()) {
      const pos = prim.getAttribute('POSITION');
      const uv = prim.getAttribute('TEXCOORD_0');
      const idx = prim.getIndices();
      const base = verts.length;
      for (let i = 0; i < pos.getCount(); i++) {
        const p = pos.getElement(i, []);
        verts.push([
          m[0]*p[0] + m[4]*p[1] + m[8]*p[2] + m[12],
          m[1]*p[0] + m[5]*p[1] + m[9]*p[2] + m[13],
          m[2]*p[0] + m[6]*p[1] + m[10]*p[2] + m[14]
        ]);
        uvs.push(uv ? uv.getElement(i, []) : [0, 0]);
      }
      const mat = prim.getMaterial();
      let mi = mats.indexOf(mat);
      if (mi < 0) { mats.push(mat); mi = mats.length - 1; }
      const count = idx ? idx.getCount() : pos.getCount();
      for (let i = 0; i < count; i += 3) {
        tris.push([
          base + (idx ? idx.getScalar(i) : i),
          base + (idx ? idx.getScalar(i+1) : i+1),
          base + (idx ? idx.getScalar(i+2) : i+2)
        ]);
        triMat.push(mi);
      }
    }
  }
  for (const c of node.listChildren()) await walk(c, m);
}

const I = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
for (const n of doc.getRoot().getDefaultScene().listChildren()) await walk(n, I);

// Load texture
const mat = mats[0];
const tex = mat ? mat.getBaseColorTexture() : null;
let texData = null, texW = 0, texH = 0;
if (tex) {
  const imgBuf = Buffer.from(tex.getImage());
  // Resize texture to 1024x1024 for fast sampling in RAM
  const { data, info } = await sharp(imgBuf).resize(1024, 1024).removeAlpha().raw().toBuffer({ resolveWithObject: true });
  texData = data;
  texW = info.width;
  texH = info.height;
  console.log(`Loaded base color texture: ${texW}x${texH}`);
}

const flat = new Float32Array(verts.length * 3);
verts.forEach((v, i) => { flat[i*3] = v[0]; flat[i*3+1] = v[1]; flat[i*3+2] = v[2]; });
const idx32 = new Uint32Array(tris.flat());

// Target ~280 triangles for high visual detail + fast FPS on ESP32
console.log('Simplifying mesh...');
let [sIdx] = MeshoptSimplifier.simplifySloppy(idx32, flat, 3, null, 280 * 3, 0.3);
console.log(`Initial decimation: ${sIdx.length / 3} tris`);

if (sIdx.length / 3 > 340) {
  const [s2] = MeshoptSimplifier.simplifySloppy(sIdx, flat, 3, null, 260 * 3, 0.4);
  if (s2.length > 0) sIdx = s2;
  console.log(`Secondary decimation: ${sIdx.length / 3} tris`);
}

// Bounding box
let mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
verts.forEach(v => {
  for (let a = 0; a < 3; a++) {
    if (v[a] < mn[a]) mn[a] = v[a];
    if (v[a] > mx[a]) mx[a] = v[a];
  }
});
const ext = mx.map((x, a) => x - mn[a]);
const ctr = mn.map((m, a) => (m + mx[a]) / 2);

// Used vertices and remapping
const used = new Map();
const outV = [];
const outF = [];
const outUV = [];
const remap = vi => {
  if (!used.has(vi)) {
    used.set(vi, outV.length);
    outV.push(verts[vi]);
    outUV.push(uvs[vi]);
  }
  return used.get(vi);
};
for (let i = 0; i < sIdx.length; i += 3) {
  outF.push([remap(sIdx[i]), remap(sIdx[i+1]), remap(sIdx[i+2])]);
}

// Map axes:
// X (length: 0.98), Y (height: 0.42), Z (width: 0.75)
// Body frame: X forward, Y left, Z up
// Let's determine if +X or -X is forward. Let's test both and inspect.
// In body frame:
// X_body = X_model (length, forward)
// Y_body = -Z_model (width, left: +Z is right, so -Z is left)
// Z_body = Y_model (height, up: +Y is up)
const scale = 127 / (Math.max(ext[0], ext[1], ext[2]) / 2);

const body = outV.map(v => {
  const fwd = (v[0] - ctr[0]) * scale;
  const lft = -(v[2] - ctr[2]) * scale;
  const up  = (v[1] - ctr[1]) * scale;
  return [
    Math.max(-127, Math.min(127, Math.round(fwd))),
    Math.max(-127, Math.min(127, Math.round(lft))),
    Math.max(-127, Math.min(127, Math.round(up)))
  ];
});

// Sample colors for each triangle from texture
const faceColors = outF.map(f => {
  let u = 0, v = 0;
  for (let k = 0; k < 3; k++) {
    u += outUV[f[k]][0] / 3;
    v += outUV[f[k]][1] / 3;
  }
  u = ((u % 1) + 1) % 1;
  v = ((v % 1) + 1) % 1;
  let px = Math.min(texW - 1, Math.max(0, Math.floor(u * texW)));
  let py = Math.min(texH - 1, Math.max(0, Math.floor(v * texH)));
  let idx = (py * texW + px) * 3;
  let r = texData[idx];
  let g = texData[idx + 1];
  let b = texData[idx + 2];
  // Convert to RGB565
  let c565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
  return { r, g, b, c565 };
});

console.log(`Generated model with ${body.length} vertices and ${outF.length} triangles.`);

// Generate preview SVG
const view = (name, rx, ry) => {
  const cx = Math.cos(rx), sx = Math.sin(rx), cy = Math.cos(ry), sy = Math.sin(ry);
  const P = body.map(([x, y, z]) => {
    const y1 = y * cx - z * sx, z1 = y * sx + z * cx;
    const x2 = x * cy + z1 * sy, z2 = -x * sy + z1 * cy;
    return [x2, y1, z2];
  });
  const order = outF.map((f, i) => [i, (P[f[0]][2] + P[f[1]][2] + P[f[2]][2]) / 3]).sort((a, b) => a[1] - b[1]);
  return order.map(([i]) => {
    const f = outF[i];
    const { r, g, b } = faceColors[i];
    return `<polygon points="${f.map(k => `${P[k][0] * 0.9 + 150},${150 - P[k][1] * 0.9}`)}" fill="rgb(${r},${g},${b})" stroke="rgb(${Math.max(0,r-20)},${Math.max(0,g-20)},${Math.max(0,b-20)})" stroke-width="0.3"/>`;
  }).join('');
};

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="300"><rect width="900" height="300" fill="#181818"/>` +
  `<g transform="translate(0,0)"><text x="10" y="20" fill="#888">Iso Rear-Right</text>${view('a', 0.5, 0.6)}</g>` +
  `<g transform="translate(300,0)"><text x="10" y="20" fill="#888">Side View</text>${view('b', 0, 1.5708)}</g>` +
  `<g transform="translate(600,0)"><text x="10" y="20" fill="#888">Top View</text>${view('c', 1.5708, 0)}</g></svg>`;

const previewPath = 'C:/Users/dell/Desktop/rover/tools/real_car_preview.png';
await sharp(Buffer.from(svg)).png().toFile(previewPath);
console.log(`Saved visual preview to ${previewPath}`);

// Write car_model.h
let h = `#pragma once\n#include <stdint.h>\n#include <pgmspace.h>\n// Authentic Textured 3D Car Model (Sampled directly from user GLB PBR texture)\n`;
h += `// Vertices: ${body.length}, Triangles: ${outF.length}\n`;
h += `#define CAR_VERT_COUNT ${body.length}\n`;
h += `#define CAR_TRI_COUNT ${outF.length}\n`;
h += `static const int8_t CAR_VERTS[CAR_VERT_COUNT][3] PROGMEM = {\n  ` + body.map(v => `{${v[0]}, ${v[1]}, ${v[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_TRIS[CAR_TRI_COUNT][3] PROGMEM = {\n  ` + outF.map(f => `{${f[0]}, ${f[1]}, ${f[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_COLORS[CAR_TRI_COUNT] PROGMEM = {\n  ` + faceColors.map(c => '0x' + c.c565.toString(16).toUpperCase().padStart(4, '0')).join(', ') + `\n};\n`;

const targetHeader = 'C:/Users/dell/Desktop/rover/s3_remote/car_model.h';
fs.writeFileSync(targetHeader, h);
console.log(`Successfully written ${targetHeader} (${h.length} bytes)`);

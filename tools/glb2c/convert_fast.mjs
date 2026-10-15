import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';

const [,, src, outH, targetTris = '220'] = process.argv;
await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(src);

const mulMat = (a, b) => {
  const o = new Array(16).fill(0);
  for (let c = 0; c < 4; c++)
    for (let r = 0; r < 4; r++)
      for (let k = 0; k < 4; k++)
        o[c*4+r] += a[k*4+r] * b[c*4+k];
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
console.log('Source verts:', verts.length, 'tris:', tris.length);

const flat = new Float32Array(verts.length * 3);
verts.forEach((v, i) => { flat[i*3] = v[0]; flat[i*3+1] = v[1]; flat[i*3+2] = v[2]; });
const idx32 = new Uint32Array(tris.flat());

const targetCount = Number(targetTris) * 3;
console.log('Decimating to target indices:', targetCount);
let [sIdx] = MeshoptSimplifier.simplifySloppy(idx32, flat, 3, null, targetCount, 0.25);
console.log('Decimated indices:', sIdx.length, 'tris:', sIdx.length / 3);

// If still too large, decimate further
if (sIdx.length / 3 > 250) {
  const [s2] = MeshoptSimplifier.simplifySloppy(sIdx, flat, 3, null, 180 * 3, 0.4);
  if (s2.length > 0) sIdx = s2;
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
console.log('Extents:', ext.map(x => x.toFixed(3)));

// Remap only used vertices
const used = new Map();
const outV = [];
const outF = [];
const remap = vi => {
  if (!used.has(vi)) {
    used.set(vi, outV.length);
    outV.push(verts[vi]);
  }
  return used.get(vi);
};
for (let i = 0; i < sIdx.length; i += 3) {
  outF.push([remap(sIdx[i]), remap(sIdx[i+1]), remap(sIdx[i+2])]);
}

// Fit body into int8_t [-127, 127]
// Tripo cars usually: +X is length (forward) or +Z is forward. Extents show which is largest.
// Find largest horizontal axis
let forwardAxis = 0; // default X
if (ext[2] > ext[0] && ext[2] > ext[1]) forwardAxis = 2; // Z
else if (ext[0] > ext[1]) forwardAxis = 0; // X
else forwardAxis = 1;

let upAxis = 1; // default Y up
if (ext[1] < ext[0] && ext[1] < ext[2]) upAxis = 1; // Y is height
else if (ext[2] < ext[0] && ext[2] < ext[1]) upAxis = 2;

let leftAxis = [0,1,2].find(a => a !== forwardAxis && a !== upAxis);

const maxDim = Math.max(ext[forwardAxis], ext[leftAxis], ext[upAxis]) / 2;
const scale = 127 / maxDim;

const body = outV.map(v => {
  const fwd = (v[forwardAxis] - ctr[forwardAxis]) * scale;
  const lft = (v[leftAxis] - ctr[leftAxis]) * scale;
  const up  = (v[upAxis] - ctr[upAxis]) * scale;
  return [
    Math.max(-127, Math.min(127, Math.round(fwd))),
    Math.max(-127, Math.min(127, Math.round(lft))),
    Math.max(-127, Math.min(127, Math.round(up)))
  ];
});

// Color mapping: generate vibrant rover body colors (body = sports orange/red, windshield = cyan/glass, tires = dark slate)
const faceCol = outF.map(f => {
  const zAvg = (body[f[0]][2] + body[f[1]][2] + body[f[2]][2]) / 3;
  const yAvg = Math.abs((body[f[0]][1] + body[f[1]][1] + body[f[2]][1]) / 3);
  const xAvg = (body[f[0]][0] + body[f[1]][0] + body[f[2]][0]) / 3;
  // Lower corners = wheels/tires (dark slate)
  if (zAvg < -40 && yAvg > 35) return 0x2124; // dark charcoal tire
  // Upper front/mid = windshield / cabin (cyan / ice blue)
  if (zAvg > 20 && Math.abs(xAvg) < 50) return 0x2DF7; // glass cyan
  // Roof / hood
  if (zAvg > 45) return 0xFD20; // orange hood
  // General chassis
  return 0xF9C0; // fiery orange/amber
});

let h = `#pragma once\n#include <stdint.h>\n#include <pgmspace.h>\n// Auto-generated low-poly car model: verts=${body.length} tris=${outF.length}\n`;
h += `#define CAR_VERT_COUNT ${body.length}\n`;
h += `#define CAR_TRI_COUNT ${outF.length}\n`;
h += `static const int8_t CAR_VERTS[CAR_VERT_COUNT][3] PROGMEM = {\n  ` + body.map(v => `{${v[0]}, ${v[1]}, ${v[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_TRIS[CAR_TRI_COUNT][3] PROGMEM = {\n  ` + outF.map(f => `{${f[0]}, ${f[1]}, ${f[2]}}`).join(', ') + `\n};\n`;
h += `static const uint16_t CAR_COLORS[CAR_TRI_COUNT] PROGMEM = {\n  ` + faceCol.map(c => '0x' + c.toString(16).toUpperCase()).join(', ') + `\n};\n`;

fs.writeFileSync(outH, h);
console.log('Successfully wrote', outH, 'bytes:', h.length);

import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';

const [,, src, outH, targetTris = '320', forwardAxis = '+x', upAxis = '+y'] = process.argv;
await MeshoptDecoder.ready; await MeshoptSimplifier.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(src);

const mulMat = (a, b) => { const o = new Array(16).fill(0); for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) for (let k = 0; k < 4; k++) o[c*4+r] += a[k*4+r] * b[c*4+k]; return o; };
const verts = [], uvs = [], tris = [], triMat = [];
const imgCache = new Map();
async function loadTex(tex) {
  if (!tex) return null;
  if (imgCache.has(tex)) return imgCache.get(tex);
  const { data, info } = await sharp(Buffer.from(tex.getImage())).removeAlpha().raw().toBuffer({ resolveWithObject: true });
  const r = { data, w: info.width, h: info.height }; imgCache.set(tex, r); return r;
}
const mats = [];
const walk = async (node, parent) => {
  const m = mulMat(parent, node.getMatrix());
  const mesh = node.getMesh();
  if (mesh) for (const prim of mesh.listPrimitives()) {
    const pos = prim.getAttribute('POSITION'); const uv = prim.getAttribute('TEXCOORD_0'); const idx = prim.getIndices();
    const base = verts.length;
    for (let i = 0; i < pos.getCount(); i++) {
      const p = pos.getElement(i, []);
      verts.push([m[0]*p[0]+m[4]*p[1]+m[8]*p[2]+m[12], m[1]*p[0]+m[5]*p[1]+m[9]*p[2]+m[13], m[2]*p[0]+m[6]*p[1]+m[10]*p[2]+m[14]]);
      uvs.push(uv ? uv.getElement(i, []) : [0, 0]);
    }
    const mat = prim.getMaterial(); let mi = mats.indexOf(mat);
    if (mi < 0) { mats.push(mat); mi = mats.length - 1; }
    const n = idx ? idx.getCount() : pos.getCount();
    for (let i = 0; i < n; i += 3) {
      const g = k => base + (idx ? idx.getScalar(i + k) : i + k);
      tris.push([g(0), g(1), g(2)]); triMat.push(mi);
    }
  }
  for (const c of node.listChildren()) await walk(c, m);
};
const I = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
for (const n of doc.getRoot().getDefaultScene().listChildren()) await walk(n, I);
console.log('source verts', verts.length, 'tris', tris.length);

// Simplify (keep material boundaries loosely via LockBorder off; colors sampled from nearest original tri afterwards)
const flat = new Float32Array(verts.length * 3); verts.forEach((v, i) => flat.set(v, i * 3));
const weld = new Map(), wmap = new Uint32Array(verts.length); verts.forEach((v, i) => { const k = v.map(x => Math.round(x * 1e5)).join(','); if (!weld.has(k)) weld.set(k, i); wmap[i] = weld.get(k); });
const idx32 = new Uint32Array(tris.flat().map(i => wmap[i]));
const [sIdx] = MeshoptSimplifier.simplify(idx32, flat, 3, Number(targetTris) * 3, 1.0, []);
console.log('simplified tris', sIdx.length / 3);

// Original triangle centroids + colors for color transfer
const col = [];
for (let t = 0; t < tris.length; t++) {
  const mat = mats[triMat[t]]; let c = [0.6, 0.6, 0.6];
  const f = mat ? mat.getBaseColorFactor() : [1, 1, 1, 1];
  const tex = mat ? await loadTex(mat.getBaseColorTexture()) : null;
  if (tex) {
    let u = 0, v = 0; for (const vi of tris[t]) { u += uvs[vi][0] / 3; v += uvs[vi][1] / 3; }
    u = ((u % 1) + 1) % 1; v = ((v % 1) + 1) % 1;
    const o = (Math.min(tex.h - 1, Math.floor(v * tex.h)) * tex.w + Math.min(tex.w - 1, Math.floor(u * tex.w))) * 3;
    c = [tex.data[o] / 255 * f[0], tex.data[o+1] / 255 * f[1], tex.data[o+2] / 255 * f[2]];
  } else c = [f[0], f[1], f[2]];
  col.push(c);
}
const cen = tris.map(t => [0,1,2].map(a => (verts[t[0]][a] + verts[t[1]][a] + verts[t[2]][a]) / 3));
// Spatial grid for nearest lookup
const G = 24; let mn = [1e9,1e9,1e9], mx = [-1e9,-1e9,-1e9];
verts.forEach(v => v.forEach((x, a) => { mn[a] = Math.min(mn[a], x); mx[a] = Math.max(mx[a], x); }));
const ext = mx.map((x, a) => x - mn[a]); const size = Math.max(...ext);
const grid = new Map(); const key = (x, y, z) => x + y * G + z * G * G;
const cell = p => p.map((x, a) => Math.min(G - 1, Math.floor((x - mn[a]) / size * G)));
cen.forEach((p, i) => { const k = key(...cell(p)); (grid.get(k) || grid.set(k, []).get(k)).push(i); });
function nearest(p) {
  const c = cell(p);
  for (let r = 1; r < G; r++) {
    let best = -1, bd = 1e30;
    for (let dx = -r; dx <= r; dx++) for (let dy = -r; dy <= r; dy++) for (let dz = -r; dz <= r; dz++) {
      const L = grid.get(key(c[0]+dx, c[1]+dy, c[2]+dz)); if (!L || c[0]+dx<0||c[1]+dy<0||c[2]+dz<0||c[0]+dx>=G||c[1]+dy>=G||c[2]+dz>=G) continue;
      for (const i of L) { const d = (cen[i][0]-p[0])**2 + (cen[i][1]-p[1])**2 + (cen[i][2]-p[2])**2; if (d < bd) { bd = d; best = i; } }
    }
    if (best >= 0) return best;
  }
  return 0;
}

// Compact vertex set
const used = new Map(), outV = [], outF = [];
const remap = vi => { if (!used.has(vi)) { used.set(vi, outV.length); outV.push(verts[vi]); } return used.get(vi); };
for (let i = 0; i < sIdx.length; i += 3) outF.push([remap(sIdx[i]), remap(sIdx[i+1]), remap(sIdx[i+2])]);

// Axis remap: model axes -> body frame (X forward, Y left, Z up), scale to +/-127
const axis = s => ({ sign: s[0] === '-' ? -1 : 1, a: 'xyz'.indexOf(s[1]) });
const F = axis(forwardAxis), U = axis(upAxis);
const L = [0,1,2].find(a => a !== F.a && a !== U.a);
const ctr = mn.map((m, a) => (m + mx[a]) / 2);
const scale = 127 / (ext[F.a] / 2);
const body = outV.map(v => {
  const f = F.sign * (v[F.a] - ctr[F.a]), u = U.sign * (v[U.a] - ctr[U.a]), l = (v[L] - ctr[L]);
  return [f, l, u].map(x => Math.max(-127, Math.min(127, Math.round(x * scale))));
});
const c565 = c => ((Math.round(c[0]*31) & 31) << 11) | ((Math.round(c[1]*63) & 63) << 5) | (Math.round(c[2]*31) & 31);
const faceCol = outF.map(f => { const p = [0,1,2].map(a => (verts[[...used.keys()][f[0]]][a] + verts[[...used.keys()][f[1]]][a] + verts[[...used.keys()][f[2]]][a]) / 3); return col[nearest(p)]; });

let h = `#pragma once\n#include <stdint.h>\n#include <pgmspace.h>\n// Auto-generated by tools/glb2c/convert.mjs  verts=${body.length} tris=${outF.length}\n`;
h += `#define CAR_VERT_COUNT ${body.length}\n#define CAR_TRI_COUNT ${outF.length}\n`;
h += `static const int8_t CAR_VERTS[CAR_VERT_COUNT][3] PROGMEM = {${body.map(v => `{${v}}`).join(',')}};\n`;
h += `static const uint16_t CAR_TRIS[CAR_TRI_COUNT][3] PROGMEM = {${outF.map(f => `{${f}}`).join(',')}};\n`;
h += `static const uint16_t CAR_COLORS[CAR_TRI_COUNT] PROGMEM = {${faceCol.map(c => '0x' + c565(c).toString(16)).join(',')}};\n`;
fs.writeFileSync(outH, h);
console.log('wrote', outH, 'bytes', h.length, 'bbox ext', ext.map(x => x.toFixed(3)));

// Preview (three views) via SVG -> PNG
const view = (name, rx, ry) => {
  const cx = Math.cos(rx), sx = Math.sin(rx), cy = Math.cos(ry), sy = Math.sin(ry);
  const P = body.map(([x, y, z]) => { const y1 = y*cx - z*sx, z1 = y*sx + z*cx; const x2 = x*cy + z1*sy, z2 = -x*sy + z1*cy; return [x2, y1, z2]; });
  const order = outF.map((f, i) => [i, (P[f[0]][2] + P[f[1]][2] + P[f[2]][2]) / 3]).sort((a, b) => a[1] - b[1]);
  return order.map(([i]) => { const f = outF[i], c = c565(faceCol[i]); const r = ((c >> 11) & 31) * 8, g = ((c >> 5) & 63) * 4, b = (c & 31) * 8;
    return `<polygon points="${f.map(k => `${P[k][0]+140},${140-P[k][1]}`)}" fill="rgb(${r},${g},${b})" stroke="rgb(${r},${g},${b})" stroke-width="0.5"/>`; }).join('');
};
const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="300"><rect width="900" height="300" fill="#222"/>` +
  `<g transform="translate(0,0)">${view('a', 0.5, 0.6)}</g><g transform="translate(300,0)">${view('b', 1.5708, 0)}</g><g transform="translate(600,0)">${view('c', 0.5, 2.6)}</g></svg>`;
await sharp(Buffer.from(svg)).png().toFile(outH.replace(/\.h$/, '_preview.png'));




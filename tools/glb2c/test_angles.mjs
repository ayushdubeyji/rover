import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb');

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

let [s1] = MeshoptSimplifier.simplify(indices, flatPos, 3, 6000 * 3, 0.03);
let [sLow] = MeshoptSimplifier.simplifySloppy(s1, flatPos, 3, null, 300 * 3, 0.12);

// Bounding box
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

// Used vertices map
const used = new Map();
const outV = [];
const outUV = [];
const remap = vi => {
  if (!used.has(vi)) {
    used.set(vi, outV.length);
    const p = pos.getElement(vi, []);
    // Body frame: X_body = (X - ctrX)/maxHalf * 127
    //             Y_body = -(Z - ctrZ)/maxHalf * 127
    //             Z_body = (Y - ctrY)/maxHalf * 127
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

// Face colors
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
  let r = texData[idx], g = texData[idx + 1], b = texData[idx + 2];
  
  // Clean color enhancement
  if (r + g + b < 60) {
    // Tire/chassis black
    r = 30; g = 32; b = 36;
  } else {
    // Enhance contrast
    r = Math.min(255, Math.round(r * 1.25));
    g = Math.min(255, Math.round(g * 1.25));
    b = Math.min(255, Math.round(b * 1.25));
  }
  let c565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
  return { r, g, b, c565 };
});

console.log(`Generated model: ${outV.length} vertices, ${outF.length} triangles`);

// Render test angles
const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderAngle(yawDeg, pitchDeg, title) {
  const radY = yawDeg * Math.PI / 180;
  const radP = pitchDeg * Math.PI / 180;
  const cy = Math.cos(radY), sy = Math.sin(radY);
  const cp = Math.cos(radP), sp = Math.sin(radP);

  const V = outV.map(([bx, by, bz]) => {
    // Pitch (tilt forward/back) around Y_body
    const x1 = bx * cp - bz * sp;
    const z1 = bx * sp + bz * cp;
    // Yaw (turn left/right) around Z_body
    const x2 = x1 * cy - by * sy;
    const y2 = x1 * sy + by * cy;
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
    polys += `<polygon points="${p1} ${p2} ${p3}" fill="rgb(${r},${g},${b})" stroke="rgba(0,0,0,0.3)" stroke-width="0.3"/>`;
  }
  return `<g><text x="10" y="20" fill="#38bdf8" font-size="12" font-family="monospace">${title}</text>${polys}</g>`;
}

const v0 = renderAngle(0, 0, "Yaw 0 deg (Chase Cam)");
const v1 = renderAngle(40, 10, "Yaw 40 deg, Pitch 10");
const v2 = renderAngle(90, 0, "Yaw 90 deg (Side View)");
const v3 = renderAngle(180, 0, "Yaw 180 deg (Front View)");

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="960" height="220" style="background:#070a13;">
  <g transform="translate(0,0)">${v0}</g>
  <g transform="translate(240,0)">${v1}</g>
  <g transform="translate(480,0)">${v2}</g>
  <g transform="translate(720,0)">${v3}</g>
</svg>`;

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/car_rotation_test.png');
console.log('Saved car_rotation_test.png');

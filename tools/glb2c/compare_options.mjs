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

// High quality decimation: first simplify to 8000, then to 350
let [s1] = MeshoptSimplifier.simplify(indices, flatPos, 3, 8000 * 3, 0.05);
let [sLow] = MeshoptSimplifier.simplifySloppy(s1, flatPos, 3, null, 320 * 3, 0.15);

// Sample face colors
const faceColors = [];
for (let i = 0; i < sLow.length; i += 3) {
  let u = 0, v = 0;
  for (let k = 0; k < 3; k++) {
    const uv = uva.getElement(sLow[i + k], []);
    u += uv[0] / 3;
    v += uv[1] / 3;
  }
  u = ((u % 1) + 1) % 1;
  v = ((v % 1) + 1) % 1;
  let px = Math.min(texW - 1, Math.max(0, Math.floor(u * texW)));
  let py = Math.min(texH - 1, Math.max(0, Math.floor(v * texH)));
  let pIdx = (py * texW + px) * 3;
  faceColors.push([texData[pIdx], texData[pIdx + 1], texData[pIdx + 2]]);
}

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

const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderOption(mapFn, title) {
  const verts = [];
  for (let i = 0; i < pos.getCount(); i++) {
    const p = pos.getElement(i, []);
    const [bx, by, bz] = mapFn(
      (p[0] - ctr[0]) / maxHalf,
      (p[1] - ctr[1]) / maxHalf,
      (p[2] - ctr[2]) / maxHalf
    );
    verts.push(toView(bx, by, bz));
  }

  // Triangles and depth sort
  const tris = [];
  for (let i = 0; i < sLow.length; i += 3) {
    const a = sLow[i], b = sLow[i+1], c = sLow[i+2];
    const d = (verts[a][2] + verts[b][2] + verts[c][2]) / 3;
    tris.push({ a, b, c, d, col: faceColors[i / 3] });
  }
  tris.sort((x, y) => x.d - y.d);

  let polys = '';
  const cx = 150, cy = 160, scale = 110;
  for (const t of tris) {
    const va = verts[t.a], vb = verts[t.b], vc = verts[t.c];
    const p1 = `${(cx + va[0] * scale).toFixed(1)},${(cy - va[1] * scale).toFixed(1)}`;
    const p2 = `${(cx + vb[0] * scale).toFixed(1)},${(cy - vb[1] * scale).toFixed(1)}`;
    const p3 = `${(cx + vc[0] * scale).toFixed(1)},${(cy - vc[1] * scale).toFixed(1)}`;
    const [r, g, b] = t.col;
    polys += `<polygon points="${p1} ${p2} ${p3}" fill="rgb(${r},${g},${b})" stroke="rgba(0,0,0,0.2)" stroke-width="0.3"/>`;
  }
  return `<g><text x="20" y="25" fill="#fff" font-size="12" font-family="sans-serif">${title}</text>${polys}</g>`;
}

const optA = renderOption((x, y, z) => [x, -z, y], "Opt A: Fwd=+X, Left=-Z, Up=+Y");
const optB = renderOption((x, y, z) => [x, z, y], "Opt B: Fwd=+X, Left=+Z, Up=+Y");
const optC = renderOption((x, y, z) => [-x, -z, y], "Opt C: Fwd=-X, Left=-Z, Up=+Y");
const optD = renderOption((x, y, z) => [-x, z, y], "Opt D: Fwd=-X, Left=+Z, Up=+Y");

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="320" style="background:#0f172a;">
  <g transform="translate(0,0)">${optA}</g>
  <g transform="translate(300,0)">${optB}</g>
  <g transform="translate(600,0)">${optC}</g>
  <g transform="translate(900,0)">${optD}</g>
</svg>`;

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/compare_options.png');
console.log('Saved compare_options.png');

import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
console.log('Loading source of truth:', glbPath);

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(glbPath);

const mat = doc.getRoot().listMaterials()[0];
const tex = mat.getBaseColorTexture();
const imgBuf = Buffer.from(tex.getImage());
const { data: texData, info: texInfo } = await sharp(imgBuf).resize(1024, 1024).removeAlpha().raw().toBuffer({ resolveWithObject: true });
const texW = texInfo.width, texH = texInfo.height;

const prim = doc.getRoot().listMeshes()[0].listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const idx = prim.getIndices();
const uva = prim.getAttribute('TEXCOORD_0');

const flatPos = new Float32Array(pos.getArray());
const indices = new Uint32Array(idx.getArray());

console.log(`Original triangles: ${indices.length / 3}`);
console.log('Simplifying to 20,000 triangles for ultra-clean PBR sprites...');
let [sHigh] = MeshoptSimplifier.simplify(indices, flatPos, 3, 20000 * 3, 0.005);
console.log(`Simplified triangles: ${sHigh.length / 3}`);

const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

export function renderSvg(yawDeg, pitchDeg, W = 128, H = 96, scale = 50) {
  const radY = yawDeg * Math.PI / 180;
  const radP = pitchDeg * Math.PI / 180;
  const cosY = Math.cos(radY), sinY = Math.sin(radY);
  const cosP = Math.cos(radP), sinP = Math.sin(radP);

  const tris = [];
  for (let i = 0; i < sHigh.length; i += 3) {
    const i0 = sHigh[i], i1 = sHigh[i+1], i2 = sHigh[i+2];
    let u = 0, v = 0;
    const pts = [i0, i1, i2].map(vi => {
      const gx = flatPos[vi*3], gy = flatPos[vi*3+1], gz = flatPos[vi*3+2];
      const uv = uva.getElement(vi, []);
      u += uv[0] / 3;
      v += uv[1] / 3;
      
      const bx = -gx / 0.49;
      const by = gz / 0.49;
      const bz = gy / 0.49;

      const x1 = bx * cosP - bz * sinP;
      const z1 = bx * sinP + bz * cosP;
      const x2 = x1 * cosY - by * sinY;
      const y2 = x1 * sinY + by * cosY;
      return toView(x2, y2, z1);
    });

    u = ((u % 1) + 1) % 1;
    v = ((v % 1) + 1) % 1;
    const px = Math.min(texW - 1, Math.max(0, Math.floor(u * texW)));
    const py = Math.min(texH - 1, Math.max(0, Math.floor(v * texH)));
    const pIdx = (py * texW + px) * 3;
    let r = texData[pIdx], g = texData[pIdx+1], b = texData[pIdx+2];

    const depth = (pts[0][2] + pts[1][2] + pts[2][2]) / 3;
    tris.push({ pts, depth, r, g, b });
  }

  tris.sort((a, b) => b.depth - a.depth);

  const cx = W / 2, cy = H / 2;
  let polys = [];
  for (const t of tris) {
    const p1 = (cx + t.pts[0][0] * scale).toFixed(1) + ',' + (cy - t.pts[0][1] * scale).toFixed(1);
    const p2 = (cx + t.pts[1][0] * scale).toFixed(1) + ',' + (cy - t.pts[1][1] * scale).toFixed(1);
    const p3 = (cx + t.pts[2][0] * scale).toFixed(1) + ',' + (cy - t.pts[2][1] * scale).toFixed(1);
    polys.push(`<polygon points="${p1} ${p2} ${p3}" fill="rgb(${t.r},${t.g},${t.b})" />`);
  }
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" style="background:#000000;">${polys.join('')}</svg>`;
}

// Test rendering 4 angles
const angles = [
  { yaw: 0, pitch: 0, name: 'chase' },
  { yaw: 45, pitch: 15, name: 'rear_3_4' },
  { yaw: 90, pitch: 0, name: 'side' },
  { yaw: 180, pitch: 0, name: 'front' }
];

for (const a of angles) {
  const svg = renderSvg(a.yaw, a.pitch, 128, 96, 50);
  const outPath = `C:/Users/dell/Desktop/rover/tools/test_pbr_${a.name}.png`;
  await sharp(Buffer.from(svg)).png().toFile(outPath);
  console.log(`Saved ${outPath}`);
}

import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';
import { execSync } from 'child_process';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
console.log('Loading GLB source of truth:', glbPath);

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

console.log('Simplifying mesh to 22,000 triangles for crisp PBR details...');
let [sHigh] = MeshoptSimplifier.simplify(indices, flatPos, 3, 22000 * 3, 0.005);
console.log(`Simplified triangles: ${sHigh.length / 3}`);

const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

const W = 100, H = 76, scale = 40;

function renderSvg(yawDeg, pitchDeg) {
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

// 1. Render all 36 frames
console.log('Rendering 36 yaw frames...');
const rawBuffers = [];
const framePngDir = 'C:/Users/dell/Desktop/rover/tools/pbr_frames';
if (!fs.existsSync(framePngDir)) fs.mkdirSync(framePngDir, { recursive: true });

for (let y = 0; y < 36; y++) {
  const yaw = y * 10;
  const svg = renderSvg(yaw, 0);
  const pngPath = `${framePngDir}/frame_${y.toString().padStart(2, '0')}.png`;
  const buf = await sharp(Buffer.from(svg)).removeAlpha().raw().toBuffer();
  await sharp(Buffer.from(svg)).png().toFile(pngPath);
  rawBuffers.push(buf);
}
console.log('Saved 36 PNG frames to', framePngDir);

// 2. Stitch into a 6x6 grid image for global palette generation
const gridW = W * 6, gridH = H * 6;
const gridBuf = new Uint8Array(gridW * gridH * 3);
for (let f = 0; f < 36; f++) {
  const gx = (f % 6) * W, gy = Math.floor(f / 6) * H;
  const fBuf = rawBuffers[f];
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const srcIdx = (y * W + x) * 3;
      const dstIdx = ((gy + y) * gridW + (gx + x)) * 3;
      gridBuf[dstIdx] = fBuf[srcIdx];
      gridBuf[dstIdx+1] = fBuf[srcIdx+1];
      gridBuf[dstIdx+2] = fBuf[srcIdx+2];
    }
  }
}

const gridPngPath = 'C:/Users/dell/Desktop/rover/tools/pbr_frames_grid.png';
await sharp(Buffer.from(gridBuf), { raw: { width: gridW, height: gridH, channels: 3 } })
  .png()
  .toFile(gridPngPath);
console.log('Saved grid image to', gridPngPath);

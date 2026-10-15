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

console.log('Simplifying to 20k triangles...');
let [sHigh] = MeshoptSimplifier.simplify(indices, flatPos, 3, 20000 * 3, 0.005);

const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderSvg(yawDeg, pitchDeg, W = 100, H = 76, scale = 40) {
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

// Generate 36 yaw frames (0 to 350 deg in 10 deg steps)
const W = 100, H = 76;
const rawFrames = [];
console.log('Rendering 36 yaw angles...');
for (let y = 0; y < 36; y++) {
  const yaw = y * 10;
  const svg = renderSvg(yaw, 0, W, H, 40);
  const buf = await sharp(Buffer.from(svg)).removeAlpha().raw().toBuffer();
  rawFrames.push(buf);
}
console.log('All 36 frames rendered!');

// Collect all unique colors (quantized to RGB565)
const colorFreq = new Map();
for (const buf of rawFrames) {
  for (let i = 0; i < buf.length; i += 3) {
    const r = buf[i], g = buf[i+1], b = buf[i+2];
    if (r < 15 && g < 15 && b < 25) continue; // transparent background
    const c565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    colorFreq.set(c565, (colorFreq.get(c565) || 0) + 1);
  }
}

console.log(`Unique RGB565 colors across all 36 frames: ${colorFreq.size}`);

// Pick top 255 colors (index 0 is reserved for transparent black)
const sortedColors = [...colorFreq.entries()].sort((a, b) => b[1] - a[1]);
const palette = [0x0000]; // index 0 = transparent
for (let i = 0; i < Math.min(255, sortedColors.length); i++) {
  palette.push(sortedColors[i][0]);
}
console.log(`Palette size: ${palette.length}`);

// Function to find nearest palette index for an RGB565 color
function findNearest(c565) {
  let bestIdx = 1, bestDist = 1e9;
  const r0 = (c565 >> 11) & 31, g0 = (c565 >> 5) & 63, b0 = c565 & 31;
  for (let i = 1; i < palette.length; i++) {
    const c = palette[i];
    const r1 = (c >> 11) & 31, g1 = (c >> 5) & 63, b1 = c & 31;
    const dr = r0 - r1, dg = (g0 - g1) >> 1, db = b0 - b1;
    const dist = dr*dr*2 + dg*dg*4 + db*db*2;
    if (dist < bestDist) {
      bestDist = dist;
      bestIdx = i;
      if (dist === 0) break;
    }
  }
  return bestIdx;
}

// Encode all 36 frames into row-spans
// Each row: uint8_t span_count
// Each span: uint8_t x_start, uint8_t len, uint8_t pixel_indices[len]
const frameData = [];
let totalBytes = 0;

for (let f = 0; f < 36; f++) {
  const buf = rawFrames[f];
  const bytes = [];
  for (let y = 0; y < H; y++) {
    // Find spans in this row
    const spans = [];
    let x = 0;
    while (x < W) {
      while (x < W) {
        const pIdx = (y * W + x) * 3;
        const r = buf[pIdx], g = buf[pIdx+1], b = buf[pIdx+2];
        if (r >= 15 || g >= 15 || b >= 25) break;
        x++;
      }
      if (x >= W) break;
      const xStart = x;
      const indices = [];
      while (x < W) {
        const pIdx = (y * W + x) * 3;
        const r = buf[pIdx], g = buf[pIdx+1], b = buf[pIdx+2];
        if (r < 15 && g < 15 && b < 25) break;
        const c565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
        indices.push(findNearest(c565));
        x++;
      }
      spans.push({ xStart, indices });
    }
    // Write spans for row y
    bytes.push(spans.length);
    for (const sp of spans) {
      bytes.push(sp.xStart);
      bytes.push(sp.indices.length);
      bytes.push(...sp.indices);
    }
  }
  frameData.push(bytes);
  totalBytes += bytes.length;
}

console.log(`Total encoded size for ALL 36 frames: ${totalBytes} bytes (${(totalBytes/1024).toFixed(1)} KB)`);

// Test decode 4 frames to verify visual accuracy
function decodeFrame(fIdx) {
  const bytes = frameData[fIdx];
  const rgb = new Uint8Array(W * H * 3);
  let p = 0;
  for (let y = 0; y < H; y++) {
    const spanCount = bytes[p++];
    for (let s = 0; s < spanCount; s++) {
      const xStart = bytes[p++];
      const len = bytes[p++];
      for (let i = 0; i < len; i++) {
        const palIdx = bytes[p++];
        const c565 = palette[palIdx];
        const r = ((c565 >> 11) & 31) * 255 / 31;
        const g = ((c565 >> 5) & 63) * 255 / 63;
        const b = (c565 & 31) * 255 / 31;
        const outIdx = (y * W + (xStart + i)) * 3;
        rgb[outIdx] = r; rgb[outIdx+1] = g; rgb[outIdx+2] = b;
      }
    }
  }
  return rgb;
}

// Combine 4 reconstructed test frames into a single image
const compositeW = W * 4, compositeH = H;
const compBuf = new Uint8Array(compositeW * compositeH * 3);

const testIndices = [0, 5, 9, 18]; // 0 deg (chase), 50 deg, 90 deg (side), 180 deg (front)
for (let i = 0; i < 4; i++) {
  const fIdx = testIndices[i];
  const dec = decodeFrame(fIdx);
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const srcIdx = (y * W + x) * 3;
      const dstIdx = (y * compositeW + (i * W + x)) * 3;
      compBuf[dstIdx] = dec[srcIdx];
      compBuf[dstIdx+1] = dec[srcIdx+1];
      compBuf[dstIdx+2] = dec[srcIdx+2];
    }
  }
}

await sharp(Buffer.from(compBuf), { raw: { width: compositeW, height: compositeH, channels: 3 } })
  .png()
  .toFile('C:/Users/dell/Desktop/rover/tools/test_decoded_pbr_frames.png');

console.log('Saved test_decoded_pbr_frames.png successfully!');

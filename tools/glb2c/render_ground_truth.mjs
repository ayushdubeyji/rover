import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt (1).glb';
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

console.log('Simplifying to 12000 triangles for high-res reference render...');
let [sHigh] = MeshoptSimplifier.simplify(indices, flatPos, 3, 12000 * 3, 0.01);
console.log('High-res triangles:', sHigh.length / 3);

const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderScene(yawDeg, pitchDeg, title) {
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
    const r = texData[pIdx], g = texData[pIdx+1], b = texData[pIdx+2];

    const depth = (pts[0][2] + pts[1][2] + pts[2][2]) / 3;
    tris.push({ pts, depth, r, g, b });
  }

  tris.sort((a, b) => b.depth - a.depth);

  const cx = 150, cy = 130, scale = 110;
  let polys = '';
  for (const t of tris) {
    const p1 = (cx + t.pts[0][0] * scale).toFixed(1) + ',' + (cy - t.pts[0][1] * scale).toFixed(1);
    const p2 = (cx + t.pts[1][0] * scale).toFixed(1) + ',' + (cy - t.pts[1][1] * scale).toFixed(1);
    const p3 = (cx + t.pts[2][0] * scale).toFixed(1) + ',' + (cy - t.pts[2][1] * scale).toFixed(1);
    polys += '<polygon points="' + p1 + ' ' + p2 + ' ' + p3 + '" fill="rgb(' + t.r + ',' + t.g + ',' + t.b + ')" />';
  }
  return '<g><text x="10" y="20" fill="#38bdf8" font-size="12" font-family="monospace">' + title + '</text>' + polys + '</g>';
}

const svg = '<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="300" style="background:#070a13;">' +
  '<g transform="translate(0,0)">' + renderScene(0, 0, "Chase Cam (Yaw 0 deg)") + '</g>' +
  '<g transform="translate(300,0)">' + renderScene(45, 15, "Rear 3/4 (Yaw 45 deg, Pitch 15)") + '</g>' +
  '<g transform="translate(600,0)">' + renderScene(90, 0, "Side View (Yaw 90 deg)") + '</g>' +
  '<g transform="translate(900,0)">' + renderScene(180, 0, "Front View (Yaw 180 deg)") + '</g>' +
'</svg>';

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/ground_truth_glb_render.png');
console.log('Saved ground_truth_glb_render.png successfully');

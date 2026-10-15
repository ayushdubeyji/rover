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

console.log('Pre-simplifying full mesh to 20k triangles first...');
let [s20k] = MeshoptSimplifier.simplify(indices, flatPos, 3, 20000 * 3, 0.01);
console.log('s20k triangles:', s20k.length / 3);

// Wheel centers
const wheelCenters = [
  { name: 'FL', c: [-0.352, -0.069, 0.294] },
  { name: 'FR', c: [-0.353, -0.075, -0.296] },
  { name: 'RL', c: [0.353, -0.055, 0.294] },
  { name: 'RR', c: [0.351, -0.061, -0.296] }
];

function getCategory(x, y, z) {
  for (let w = 0; w < 4; w++) {
    const [cx, cy, cz] = wheelCenters[w].c;
    const dx = x - cx, dy = y - cy, dz = z - cz;
    if (Math.sqrt(dx*dx + dy*dy) < 0.17 && Math.abs(dz) < 0.11) {
      return 'wheel_' + w;
    }
  }
  if (y > 0.01 && Math.abs(x) < 0.22 && Math.abs(z) < 0.18) {
    return 'battery';
  }
  return 'chassis';
}

const groups = {
  wheel_0: [],
  wheel_1: [],
  wheel_2: [],
  wheel_3: [],
  battery: [],
  chassis: []
};

for (let i = 0; i < s20k.length; i += 3) {
  const i0 = s20k[i], i1 = s20k[i+1], i2 = s20k[i+2];
  const p0 = [flatPos[i0*3], flatPos[i0*3+1], flatPos[i0*3+2]];
  const p1 = [flatPos[i1*3], flatPos[i1*3+1], flatPos[i1*3+2]];
  const p2 = [flatPos[i2*3], flatPos[i2*3+1], flatPos[i2*3+2]];
  const avg = [(p0[0]+p1[0]+p2[0])/3, (p0[1]+p1[1]+p2[1])/3, (p0[2]+p1[2]+p2[2])/3];
  
  const cat = getCategory(avg[0], avg[1], avg[2]);
  groups[cat].push(i0, i1, i2);
}

for (const k in groups) {
  console.log(`Group ${k}: ${groups[k].length / 3} triangles`);
}

// Simplify each group independently:
const budgets = {
  wheel_0: 90,
  wheel_1: 90,
  wheel_2: 90,
  wheel_3: 90,
  battery: 110,
  chassis: 160
};

const combinedTris = [];
for (const k in groups) {
  const tris = groups[k];
  if (tris.length === 0) continue;
  const target = budgets[k];
  let [s] = MeshoptSimplifier.simplifySloppy(new Uint32Array(tris), flatPos, 3, null, target * 3, 0.25);
  console.log(`Simplified ${k} to ${s.length / 3} triangles`);
  for (let j = 0; j < s.length; j++) combinedTris.push(s[j]);
}

console.log('Total combined triangles:', combinedTris.length / 3);

// Render verification
const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderMesh(yawDeg, pitchDeg, title) {
  const radY = yawDeg * Math.PI / 180;
  const radP = pitchDeg * Math.PI / 180;
  const cosY = Math.cos(radY), sinY = Math.sin(radY);
  const cosP = Math.cos(radP), sinP = Math.sin(radP);

  const tris = [];
  for (let i = 0; i < combinedTris.length; i += 3) {
    const i0 = combinedTris[i], i1 = combinedTris[i+1], i2 = combinedTris[i+2];
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

  const cx = 150, cy = 130, scale = 110;
  let polys = '';
  for (const t of tris) {
    const p1 = (cx + t.pts[0][0] * scale).toFixed(1) + ',' + (cy - t.pts[0][1] * scale).toFixed(1);
    const p2 = (cx + t.pts[1][0] * scale).toFixed(1) + ',' + (cy - t.pts[1][1] * scale).toFixed(1);
    const p3 = (cx + t.pts[2][0] * scale).toFixed(1) + ',' + (cy - t.pts[2][1] * scale).toFixed(1);
    polys += '<polygon points="' + p1 + ' ' + p2 + ' ' + p3 + '" fill="rgb(' + t.r + ',' + t.g + ',' + t.b + ')" stroke="rgba(0,0,0,0.15)" stroke-width="0.3"/>';
  }
  return '<g><text x="10" y="20" fill="#38bdf8" font-size="12" font-family="monospace">' + title + ' (' + (combinedTris.length/3) + ' tris)</text>' + polys + '</g>';
}

const svg = '<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="300" style="background:#070a13;">' +
  '<g transform="translate(0,0)">' + renderMesh(0, 0, "Chase Cam (Yaw 0)") + '</g>' +
  '<g transform="translate(300,0)">' + renderMesh(45, 15, "Rear 3/4 (Yaw 45, Pitch 15)") + '</g>' +
  '<g transform="translate(600,0)">' + renderMesh(90, 0, "Side View (Yaw 90)") + '</g>' +
  '<g transform="translate(900,0)">' + renderMesh(180, 0, "Front View (Yaw 180)") + '</g>' +
'</svg>';

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/segmented_decimate_render.png');
console.log('Saved segmented_decimate_render.png');

import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('C:/Users/dell/Desktop/rover/tools/rover_model_optimized.glb');
const prim = doc.getRoot().listMeshes()[0].listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const uva = prim.getAttribute('TEXCOORD_0');
const idx = prim.getIndices();

const wheelCenters = [
  [-0.352, -0.069, 0.294],
  [-0.353, -0.075, -0.296],
  [0.353, -0.055, 0.294],
  [0.351, -0.061, -0.296]
];

function isWheelVertex(x, y, z) {
  for (const [cx, cy, cz] of wheelCenters) {
    const dx = x - cx, dy = y - cy, dz = z - cz;
    const dist2D = Math.sqrt(dx*dx + dy*dy);
    if (dist2D < 0.155 && Math.abs(dz) < 0.105) return true;
  }
  return false;
}

// Collect non-wheel triangles
const chassisIndices = [];
for (let i = 0; i < idx.getCount(); i += 3) {
  const i0 = idx.getScalar(i), i1 = idx.getScalar(i+1), i2 = idx.getScalar(i+2);
  const p0 = pos.getElement(i0, []);
  const p1 = pos.getElement(i1, []);
  const p2 = pos.getElement(i2, []);
  // Keep triangle if at least 2 vertices are not wheel
  const wCount = (isWheelVertex(p0[0], p0[1], p0[2]) ? 1 : 0) +
                (isWheelVertex(p1[0], p1[1], p1[2]) ? 1 : 0) +
                (isWheelVertex(p2[0], p2[1], p2[2]) ? 1 : 0);
  if (wCount < 2) {
    chassisIndices.push(i0, i1, i2);
  }
}
console.log('Chassis triangles before simplification:', chassisIndices.length / 3);

const flatPos = new Float32Array(pos.getArray());
// Simplify chassis
const targetChassisTri = 400;
let [sChassis, err] = MeshoptSimplifier.simplifySloppy(new Uint32Array(chassisIndices), flatPos, 3, null, targetChassisTri * 3, 0.15);
console.log('Simplified chassis triangles:', sChassis.length / 3, 'error:', err);

// Render SVG test
const cT = 0.8387, sT = 0.5446;
const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];

function renderView(yawDeg, pitchDeg, title) {
  const radY = yawDeg * Math.PI / 180, radP = pitchDeg * Math.PI / 180;
  const cy = Math.cos(radY), sy = Math.sin(radY);
  const cp = Math.cos(radP), sp = Math.sin(radP);
  
  let polys = [];
  for (let i = 0; i < sChassis.length; i += 3) {
    const pts = [sChassis[i], sChassis[i+1], sChassis[i+2]].map(vi => {
      // glb: -X = fwd, +Z = left, +Y = up
      // body: X = -x_glb, Y = +z_glb, Z = +y_glb
      const gx = flatPos[vi*3], gy = flatPos[vi*3+1], gz = flatPos[vi*3+2];
      const bx = -gx, by = gz, bz = gy;
      
      const x1 = bx * cp - bz * sp;
      const z1 = bx * sp + bz * cp;
      const x2 = x1 * cy - by * sy;
      const y2 = x1 * sy + by * cy;
      const [vx, vy, vd] = toView(x2, y2, z1);
      return { sx: 150 + vx * 180, sy: 130 - vy * 180, vd };
    });
    const depth = (pts[0].vd + pts[1].vd + pts[2].vd) / 3;
    polys.push({ pts, depth });
  }
  polys.sort((a, b) => a.depth - b.depth);
  let svgPolys = polys.map(p => 
    `<polygon points="${p.pts[0].sx.toFixed(1)},${p.pts[0].sy.toFixed(1)} ${p.pts[1].sx.toFixed(1)},${p.pts[1].sy.toFixed(1)} ${p.pts[2].sx.toFixed(1)},${p.pts[2].sy.toFixed(1)}" fill="#2d6a4f" stroke="#52b788" stroke-width="0.5"/>`
  ).join('');
  return `<g><text x="10" y="20" fill="#fff" font-size="12" font-family="sans-serif">${title}</text>${svgPolys}</g>`;
}

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="900" height="300" style="background:#070a13;">
  <g transform="translate(0,0)">${renderView(30, 15, "Chase 3/4 View")}</g>
  <g transform="translate(300,0)">${renderView(90, 0, "Side View")}</g>
  <g transform="translate(600,0)">${renderView(180, 0, "Front View")}</g>
</svg>`;

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/test_chassis_preview.png');
console.log('Saved test_chassis_preview.png');

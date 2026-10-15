import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb');

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const idx = prim.getIndices();
const uva = prim.getAttribute('TEXCOORD_0');

const flatPos = new Float32Array(pos.getArray());
const indices = new Uint32Array(idx.getArray());

// Simplify to 500 triangles
let [s1] = MeshoptSimplifier.simplify(indices, flatPos, 3, 5000 * 3, 0.05);
let [sLow] = MeshoptSimplifier.simplifySloppy(s1, flatPos, 3, null, 400 * 3, 0.15);

// Build SVG side view (X vs Y) and top view (X vs Z)
let sidePolys = '';
let topPolys = '';

for (let i = 0; i < sLow.length; i += 3) {
  const p0 = pos.getElement(sLow[i], []);
  const p1 = pos.getElement(sLow[i+1], []);
  const p2 = pos.getElement(sLow[i+2], []);
  
  // Side view: X (-0.5..0.5) mapped to 50..350, Y (-0.25..0.25) mapped to 250..50
  const sx0 = 200 + p0[0] * 300, sy0 = 150 - p0[1] * 300;
  const sx1 = 200 + p1[0] * 300, sy1 = 150 - p1[1] * 300;
  const sx2 = 200 + p2[0] * 300, sy2 = 150 - p2[1] * 300;
  sidePolys += `<polygon points="${sx0.toFixed(1)},${sy0.toFixed(1)} ${sx1.toFixed(1)},${sy1.toFixed(1)} ${sx2.toFixed(1)},${sy2.toFixed(1)}" fill="#38bdf8" stroke="#0f172a" stroke-width="0.5"/>`;

  // Top view: X (-0.5..0.5) mapped to 50..350, Z (-0.4..0.4) mapped to 50..350
  const tx0 = 200 + p0[0] * 300, ty0 = 150 + p0[2] * 300;
  const tx1 = 200 + p1[0] * 300, ty1 = 150 + p1[2] * 300;
  const tx2 = 200 + p2[0] * 300, ty2 = 150 + p2[2] * 300;
  topPolys += `<polygon points="${tx0.toFixed(1)},${ty0.toFixed(1)} ${tx1.toFixed(1)},${ty1.toFixed(1)} ${tx2.toFixed(1)},${ty2.toFixed(1)}" fill="#10b981" stroke="#0f172a" stroke-width="0.5"/>`;
}

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="800" height="300" style="background:#1e293b;">
  <g transform="translate(0,0)"><text x="20" y="30" fill="#fff" font-family="sans-serif">Side View: +X is right, -X is left</text>${sidePolys}</g>
  <g transform="translate(400,0)"><text x="20" y="30" fill="#fff" font-family="sans-serif">Top View: +X is right, +Z is down</text>${topPolys}</g>
</svg>`;

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/car_silhouette.png');
console.log('Saved car_silhouette.png');

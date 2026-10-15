import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';
import sharp from 'sharp';

await MeshoptDecoder.ready;
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
const uva = prim.getAttribute('TEXCOORD_0');

// Sample points along -X (negative X) vs +X (positive X)
let negPts = [], posPts = [];
const count = pos.getCount();
for (let i = 0; i < count; i += 20) {
  let p = pos.getElement(i, []);
  let uv = uva.getElement(i, []);
  let u = ((uv[0] % 1) + 1) % 1;
  let v = ((uv[1] % 1) + 1) % 1;
  let px = Math.min(texW - 1, Math.max(0, Math.floor(u * texW)));
  let py = Math.min(texH - 1, Math.max(0, Math.floor(v * texH)));
  let idx = (py * texW + px) * 3;
  let r = texData[idx], g = texData[idx+1], b = texData[idx+2];
  
  if (p[0] < -0.2) negPts.push({ p, color: `rgb(${r},${g},${b})` });
  if (p[0] > 0.2) posPts.push({ p, color: `rgb(${r},${g},${b})` });
}

console.log('Sampled negPts:', negPts.length, 'posPts:', posPts.length);

// Render colored front view (-X) and rear view (+X)
const renderNeg = negPts.map(pt => {
  let cx = 200 + pt.p[2] * 400; // Z width
  let cy = 200 - pt.p[1] * 400; // Y height
  return `<circle cx="${cx.toFixed(1)}" cy="${cy.toFixed(1)}" r="2" fill="${pt.color}" />`;
}).join('');

const renderPos = posPts.map(pt => {
  let cx = 200 + pt.p[2] * 400; // Z width
  let cy = 200 - pt.p[1] * 400; // Y height
  return `<circle cx="${cx.toFixed(1)}" cy="${cy.toFixed(1)}" r="2" fill="${pt.color}" />`;
}).join('');

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="800" height="400" style="background:#070a13;">
  <g transform="translate(0,0)"><text x="20" y="30" fill="#fff" font-family="sans-serif">Negative X End (Looking at X=-0.48)</text>${renderNeg}</g>
  <g transform="translate(400,0)"><text x="20" y="30" fill="#fff" font-family="sans-serif">Positive X End (Looking at X=+0.48)</text>${renderPos}</g>
</svg>`;

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/front_rear_colored.png');
console.log('Saved front_rear_colored.png');

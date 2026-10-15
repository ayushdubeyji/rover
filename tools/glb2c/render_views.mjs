import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';
import sharp from 'sharp';

await MeshoptDecoder.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb');

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
console.log('Position count:', pos.getCount());

const pts = [];
const step = Math.floor(pos.getCount() / 15000);
for (let i = 0; i < pos.getCount(); i += step) {
  pts.push(pos.getElement(i, []));
}

const renderPts = (pxIdx, pyIdx, sX, sY, offX, offY) => {
  let s = '';
  for (const p of pts) {
    const px = offX + p[pxIdx] * sX;
    const py = offY - p[pyIdx] * sY;
    s += `<circle cx="${px.toFixed(1)}" cy="${py.toFixed(1)}" r="1" fill="#00ffcc" opacity="0.6"/>`;
  }
  return s;
};

const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="400" style="background:#111;">
  <g transform="translate(0,0)"><text x="20" y="30" fill="#fff" font-family="sans-serif">Top View (X len vs Z width)</text>${renderPts(0, 2, 350, 350, 200, 200)}</g>
  <g transform="translate(400,0)"><text x="20" y="30" fill="#fff" font-family="sans-serif">Side View (X len vs Y height)</text>${renderPts(0, 1, 350, 350, 200, 250)}</g>
  <g transform="translate(800,0)"><text x="20" y="30" fill="#fff" font-family="sans-serif">Front/Back (Z width vs Y height)</text>${renderPts(2, 1, 350, 350, 200, 250)}</g>
</svg>`;

await sharp(Buffer.from(svg)).png().toFile('C:/Users/dell/Desktop/rover/tools/car_views.png');
console.log('Saved car_views.png successfully');

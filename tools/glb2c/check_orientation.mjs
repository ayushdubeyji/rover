import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';
import sharp from 'sharp';

await MeshoptDecoder.ready;
const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(glbPath);

const mat = doc.getRoot().listMaterials()[0];
const tex = mat.getBaseColorTexture();
const imgBuf = Buffer.from(tex.getImage());
const { data, info } = await sharp(imgBuf).removeAlpha().raw().toBuffer({ resolveWithObject: true });
console.log('Texture:', info.width, 'x', info.height);

// Let's check some vertices near +X (x > 0.4) and -X (x < -0.4)
const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const uva = prim.getAttribute('TEXCOORD_0');

let posColors = [];
let negColors = [];

for (let i = 0; i < pos.getCount(); i += 50) {
  const p = pos.getElement(i, []);
  const uv = uva ? uva.getElement(i, []) : [0,0];
  let u = ((uv[0] % 1) + 1) % 1;
  let v = ((uv[1] % 1) + 1) % 1;
  let px = Math.min(info.width - 1, Math.floor(u * info.width));
  let py = Math.min(info.height - 1, Math.floor(v * info.height));
  let idx = (py * info.width + px) * 3;
  let r = data[idx], g = data[idx+1], b = data[idx+2];
  if (p[0] > 0.42) posColors.push([r, g, b]);
  if (p[0] < -0.42) negColors.push([r, g, b]);
}

const avg = arr => arr.reduce((acc, c) => [acc[0]+c[0]/arr.length, acc[1]+c[1]/arr.length, acc[2]+c[2]/arr.length], [0,0,0]);
console.log('+X end avg RGB:', avg(posColors).map(Math.round));
console.log('-X end avg RGB:', avg(negColors).map(Math.round));

// Also let's inspect the general car color
let allColors = [];
for (let i = 0; i < pos.getCount(); i += 200) {
  const uv = uva.getElement(i, []);
  let u = ((uv[0] % 1) + 1) % 1;
  let v = ((uv[1] % 1) + 1) % 1;
  let px = Math.min(info.width - 1, Math.floor(u * info.width));
  let py = Math.min(info.height - 1, Math.floor(v * info.height));
  let idx = (py * info.width + px) * 3;
  allColors.push([data[idx], data[idx+1], data[idx+2]]);
}
console.log('Overall car avg RGB:', avg(allColors).map(Math.round));

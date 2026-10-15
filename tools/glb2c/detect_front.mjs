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
const { data, info } = await sharp(imgBuf).removeAlpha().raw().toBuffer({ resolveWithObject: true });
console.log('Texture:', info.width, 'x', info.height);

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const uva = prim.getAttribute('TEXCOORD_0');

let posRed = 0, posWhite = 0, negRed = 0, negWhite = 0;
for (let i = 0; i < pos.getCount(); i += 10) {
  const p = pos.getElement(i, []);
  const uv = uva.getElement(i, []);
  let u = ((uv[0] % 1) + 1) % 1;
  let v = ((uv[1] % 1) + 1) % 1;
  let px = Math.min(info.width - 1, Math.floor(u * info.width));
  let py = Math.min(info.height - 1, Math.floor(v * info.height));
  let idx = (py * info.width + px) * 3;
  let r = data[idx], g = data[idx+1], b = data[idx+2];
  
  // Check red (taillight): high R, low G, low B
  if (r > 160 && g < 80 && b < 80) {
    if (p[0] > 0.35) posRed++;
    if (p[0] < -0.35) negRed++;
  }
  // Check bright white/yellow (headlight): high R, high G
  if (r > 200 && g > 200 && b > 150) {
    if (p[0] > 0.35) posWhite++;
    if (p[0] < -0.35) negWhite++;
  }
}

console.log('+X end: red count =', posRed, 'white count =', posWhite);
console.log('-X end: red count =', negRed, 'white count =', negWhite);

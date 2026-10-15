import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';

await MeshoptDecoder.ready;
const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(glbPath);

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');

// Split X into 10 bins from -0.489 to +0.489 and find max Y in each bin
let bins = new Array(10).fill(-1e9);
for (let i = 0; i < pos.getCount(); i++) {
  const p = pos.getElement(i, []);
  let b = Math.min(9, Math.max(0, Math.floor((p[0] + 0.489) / 0.979 * 10)));
  bins[b] = Math.max(bins[b], p[1]);
}

console.log('Max Y profile from -X to +X:');
bins.forEach((y, i) => {
  let x = -0.489 + (i + 0.5) * 0.0979;
  console.log(`Bin ${i} (x = ${x.toFixed(2)}): max Y = ${y.toFixed(3)}`);
});

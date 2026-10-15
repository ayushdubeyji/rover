import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb');

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const idx = prim.getIndices();

const flatPos = new Float32Array(pos.getArray());
const indices = new Uint32Array(idx.getArray());

let [s1] = MeshoptSimplifier.simplify(indices, flatPos, 3, 6000 * 3, 0.03);

for (const err of [0.06, 0.07, 0.08, 0.09, 0.10]) {
  for (const target of [350, 400, 450, 500]) {
    let [s] = MeshoptSimplifier.simplifySloppy(s1, flatPos, 3, null, target * 3, err);
    console.log(`err: ${err}, target: ${target} -> tris: ${s.length / 3}`);
  }
}

import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(glbPath);

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const idx = prim.getIndices();

const flatPos = new Float32Array(pos.getArray());
const indices = new Uint32Array(idx.getArray());

console.log('Simplifying to 10k with strict topology...');
let [s1] = MeshoptSimplifier.simplify(indices, flatPos, 3, 10000 * 3, 0.05);
console.log('Strict simplified triangles:', s1.length / 3);

// Now run sloppy from 4k down to 240
console.log('Running simplifySloppy on the clean low-poly mesh...');
let [sLow] = MeshoptSimplifier.simplifySloppy(s1, flatPos, 3, null, 240 * 3, 0.15);
console.log('Low poly triangles:', sLow.length / 3);

let [sLow2] = MeshoptSimplifier.simplifySloppy(s1, flatPos, 3, null, 180 * 3, 0.15);
console.log('Low poly 2 triangles:', sLow2.length / 3);

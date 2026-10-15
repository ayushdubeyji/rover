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

console.log('Original pos count:', pos.getCount(), 'indices:', indices.length, 'triangles:', indices.length / 3);

// Test stage 1: simplify to 50k
console.log('Simplifying to 50k...');
let [s1, err1] = MeshoptSimplifier.simplify(indices, flatPos, 3, 50000 * 3, 0.05);
console.log('Stage 1 triangles:', s1.length / 3, 'err:', err1);

// Test stage 2: simplify to 5k
console.log('Simplifying to 5k...');
let [s2, err2] = MeshoptSimplifier.simplify(s1, flatPos, 3, 5000 * 3, 0.05);
console.log('Stage 2 triangles:', s2.length / 3, 'err:', err2);

// Test stage 3: simplify to 500
console.log('Simplifying to 500...');
let [s3, err3] = MeshoptSimplifier.simplify(s2, flatPos, 3, 500 * 3, 0.1);
console.log('Stage 3 triangles:', s3.length / 3, 'err:', err3);

// Test stage 4: simplify to 250
let [s4, err4] = MeshoptSimplifier.simplify(s3, flatPos, 3, 250 * 3, 0.15);
console.log('Stage 4 triangles:', s4.length / 3, 'err:', err4);

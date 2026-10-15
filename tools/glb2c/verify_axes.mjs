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

let min = [1e9, 1e9, 1e9], max = [-1e9, -1e9, -1e9];
for (let i = 0; i < pos.getCount(); i++) {
  const p = pos.getElement(i, []);
  for (let a = 0; a < 3; a++) {
    min[a] = Math.min(min[a], p[a]);
    max[a] = Math.max(max[a], p[a]);
  }
}
console.log('Model Bounding Box:');
console.log('X:', min[0].toFixed(3), 'to', max[0].toFixed(3), 'len =', (max[0]-min[0]).toFixed(3));
console.log('Y:', min[1].toFixed(3), 'to', max[1].toFixed(3), 'len =', (max[1]-min[1]).toFixed(3));
console.log('Z:', min[2].toFixed(3), 'to', max[2].toFixed(3), 'len =', (max[2]-min[2]).toFixed(3));

// Check vertex density at extremes
// In cars: Y is usually height (ext ~0.424). Let's check Y min vs Y max:
// Y min is ground (tires touch ground), Y max is roof.
// Between X (0.979) and Z (0.747):
// Length of a car is longer than its width.
// Car length = 0.979 (X axis). Car width = 0.747 (Z axis).
console.log('Length is X (0.979), Height is Y (0.424), Width is Z (0.747).');

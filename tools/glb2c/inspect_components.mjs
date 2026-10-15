import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';

await MeshoptDecoder.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb');

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const idx = prim.getIndices();
const count = pos.getCount();

const parent = new Int32Array(count);
for (let i = 0; i < count; i++) parent[i] = i;
function find(i) {
  let root = i;
  while (root !== parent[root]) root = parent[root];
  let curr = i;
  while (curr !== root) { let nxt = parent[curr]; parent[curr] = root; curr = nxt; }
  return root;
}
function union(i, j) {
  let r1 = find(i), r2 = find(j);
  if (r1 !== r2) parent[r1] = r2;
}
for (let i = 0; i < idx.getCount(); i += 3) {
  union(idx.getScalar(i), idx.getScalar(i+1));
  union(idx.getScalar(i+1), idx.getScalar(i+2));
}

const comps = new Map();
for (let i = 0; i < count; i++) {
  let r = find(i);
  let p = pos.getElement(i, []);
  let c = comps.get(r);
  if (!c) {
    c = { r, count: 0, mn: [1e9, 1e9, 1e9], mx: [-1e9, -1e9, -1e9], sum: [0, 0, 0] };
    comps.set(r, c);
  }
  c.count++;
  for (let a = 0; a < 3; a++) {
    if (p[a] < c.mn[a]) c.mn[a] = p[a];
    if (p[a] > c.mx[a]) c.mx[a] = p[a];
    c.sum[a] += p[a];
  }
}

const sorted = [...comps.values()].sort((a, b) => b.count - a.count);
console.log('Top 20 components:');
sorted.slice(0, 20).forEach((c, idx) => {
  let center = c.sum.map(s => (s / c.count).toFixed(3)).join(', ');
  let size = c.mx.map((m, a) => (m - c.mn[a]).toFixed(3)).join(', ');
  console.log('Comp ' + idx + ': verts=' + c.count + ' center=[' + center + '] size=[' + size + ']');
});

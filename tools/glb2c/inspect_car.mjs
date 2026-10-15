import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';

await MeshoptDecoder.ready;
await MeshoptSimplifier.ready;

const glbPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(glbPath);

const mulMat = (a, b) => {
  const o = new Array(16).fill(0);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) for (let k = 0; k < 4; k++) o[c*4+r] += a[k*4+r] * b[c*4+k];
  return o;
};

const verts = [], uvs = [], tris = [];
const mats = [];
const triMat = [];

async function walk(node, parent) {
  const m = mulMat(parent, node.getMatrix());
  const mesh = node.getMesh();
  if (mesh) {
    for (const prim of mesh.listPrimitives()) {
      const pos = prim.getAttribute('POSITION');
      const uv = prim.getAttribute('TEXCOORD_0');
      const idx = prim.getIndices();
      const base = verts.length;
      for (let i = 0; i < pos.getCount(); i++) {
        const p = pos.getElement(i, []);
        verts.push([
          m[0]*p[0] + m[4]*p[1] + m[8]*p[2] + m[12],
          m[1]*p[0] + m[5]*p[1] + m[9]*p[2] + m[13],
          m[2]*p[0] + m[6]*p[1] + m[10]*p[2] + m[14]
        ]);
        uvs.push(uv ? uv.getElement(i, []) : [0, 0]);
      }
      const mat = prim.getMaterial();
      let mi = mats.indexOf(mat);
      if (mi < 0) { mats.push(mat); mi = mats.length - 1; }
      const count = idx ? idx.getCount() : pos.getCount();
      for (let i = 0; i < count; i += 3) {
        tris.push([
          base + (idx ? idx.getScalar(i) : i),
          base + (idx ? idx.getScalar(i+1) : i+1),
          base + (idx ? idx.getScalar(i+2) : i+2)
        ]);
        triMat.push(mi);
      }
    }
  }
  for (const c of node.listChildren()) await walk(c, m);
}

const I = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
for (const n of doc.getRoot().getDefaultScene().listChildren()) await walk(n, I);
console.log('Source verts:', verts.length, 'tris:', tris.length);

let mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
verts.forEach(v => {
  for (let a = 0; a < 3; a++) {
    if (v[a] < mn[a]) mn[a] = v[a];
    if (v[a] > mx[a]) mx[a] = v[a];
  }
});
const ext = mx.map((x, a) => x - mn[a]);
console.log('Extents X,Y,Z:', ext.map(x => x.toFixed(3)));
console.log('Min:', mn.map(x => x.toFixed(3)), 'Max:', mx.map(x => x.toFixed(3)));

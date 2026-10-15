import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { prune } from '@gltf-transform/functions';
import { MeshoptDecoder, MeshoptEncoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';
import fs from 'fs';

await MeshoptDecoder.ready;
await MeshoptEncoder.ready;
await MeshoptSimplifier.ready;

const inputPath = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
const outputPath = 'C:/Users/dell/Desktop/rover/tools/rover_model_optimized.glb';

console.log('Loading high-poly GLB:', inputPath);
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({
  'meshopt.decoder': MeshoptDecoder,
  'meshopt.encoder': MeshoptEncoder
});
const doc = await io.read(inputPath);

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const norm = prim.getAttribute('NORMAL');
const uva = prim.getAttribute('TEXCOORD_0');
const idx = prim.getIndices();

const flatPos = new Float32Array(pos.getArray());
const indices = new Uint32Array(idx.getArray());
console.log(`Original triangles: ${indices.length / 3}`);

// Simplify to 30,000 triangles with strict topology preservation
console.log('Simplifying to ~30k triangles...');
let [sIdx, err] = MeshoptSimplifier.simplify(indices, flatPos, 3, 30000 * 3, 0.005);
console.log(`Simplified to: ${sIdx.length / 3} triangles (error: ${err.toFixed(5)})`);

// Remap and compact vertices
const used = new Map();
const newPos = [];
const newNorm = [];
const newUV = [];
const newIndices = new Uint32Array(sIdx.length);

for (let i = 0; i < sIdx.length; i++) {
  const vi = sIdx[i];
  if (!used.has(vi)) {
    const newIdx = used.size;
    used.set(vi, newIdx);
    newPos.push(...pos.getElement(vi, []));
    if (norm) newNorm.push(...norm.getElement(vi, []));
    if (uva) newUV.push(...uva.getElement(vi, []));
  }
  newIndices[i] = used.get(vi);
}
console.log(`Compacted vertices: ${used.size}`);

// Create new accessors
const buffer = doc.getRoot().listBuffers()[0];
const newPosAcc = doc.createAccessor().setType('VEC3').setArray(new Float32Array(newPos)).setBuffer(buffer);
const newIdxAcc = doc.createAccessor().setType('SCALAR').setArray(newIndices).setBuffer(buffer);
prim.setAttribute('POSITION', newPosAcc);
prim.setIndices(newIdxAcc);

if (norm) {
  const newNormAcc = doc.createAccessor().setType('VEC3').setArray(new Float32Array(newNorm)).setBuffer(buffer);
  prim.setAttribute('NORMAL', newNormAcc);
}
if (uva) {
  const newUvAcc = doc.createAccessor().setType('VEC2').setArray(new Float32Array(newUV)).setBuffer(buffer);
  prim.setAttribute('TEXCOORD_0', newUvAcc);
}

// Optimize textures: resize base color to 1024x1024 JPEG with 85% quality
const textures = doc.getRoot().listTextures();
console.log(`Optimizing ${textures.length} textures...`);
for (const tex of textures) {
  const rawImg = tex.getImage();
  if (rawImg && rawImg.length > 500000) {
    console.log(`Resizing texture: ${tex.getName() || 'unnamed'} (original size: ${(rawImg.length / 1024 / 1024).toFixed(2)} MB)`);
    const optimizedBuf = await sharp(Buffer.from(rawImg))
      .resize(1024, 1024, { fit: 'inside' })
      .jpeg({ quality: 85 })
      .toBuffer();
    tex.setImage(optimizedBuf);
    tex.setMimeType('image/jpeg');
    console.log(`Optimized to: ${(optimizedBuf.length / 1024).toFixed(1)} KB`);
  }
}

console.log('Pruning unreferenced buffers...');
await doc.transform(prune());

console.log('Writing optimized GLB to:', outputPath);
const glbBuffer = await io.writeBinary(doc);
fs.writeFileSync(outputPath, Buffer.from(glbBuffer));
console.log(`Successfully generated optimized GLB! File size: ${(glbBuffer.byteLength / 1024 / 1024).toFixed(2)} MB`);

import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';
import fs from 'fs';

await MeshoptDecoder.ready;
const src = 'C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb';
const dst = 'C:/Users/dell/Desktop/rover/tools/rover_model_standard.glb';

console.log('Reading and decompressing GLB...');
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read(src);

// Strip meshopt extension requirement so standard three.js GLTFLoader can load it without any decoder!
const root = doc.getRoot();
for (const ext of doc.getRoot().listExtensionsUsed()) {
  if (ext.extensionName === 'KHR_meshopt_compression') {
    ext.dispose();
  }
}

const glbBytes = await io.writeBinary(doc);
fs.writeFileSync(dst, Buffer.from(glbBytes));
console.log(`Saved uncompressed standard GLB to ${dst} (${(glbBytes.byteLength/1024/1024).toFixed(1)} MB)`);

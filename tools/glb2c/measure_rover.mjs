import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder } from 'meshoptimizer';

await MeshoptDecoder.ready;
const io = new NodeIO().registerExtensions(ALL_EXTENSIONS).registerDependencies({ 'meshopt.decoder': MeshoptDecoder });
const doc = await io.read('C:/Users/dell/Downloads/tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb');

const mesh = doc.getRoot().listMeshes()[0];
const prim = mesh.listPrimitives()[0];
const pos = prim.getAttribute('POSITION');
const count = pos.getCount();

// Find wheel clusters
// In glTF:
// Front is -X (around -0.35), Rear is +X (around +0.35)
// Left is +Z (around +0.28), Right is -Z (around -0.28)
// Ground is -Y (around -0.15 to -0.21)

const flPts = [], frPts = [], rlPts = [], rrPts = [];
const chassisPts = [];

for (let i = 0; i < count; i += 5) {
  let [x, y, z] = pos.getElement(i, []);
  // Wheels are located at |x| > 0.22 and |z| > 0.20
  if (x < -0.22 && z > 0.20) flPts.push([x, y, z]);
  else if (x < -0.22 && z < -0.20) frPts.push([x, y, z]);
  else if (x > 0.22 && z > 0.20) rlPts.push([x, y, z]);
  else if (x > 0.22 && z < -0.20) rrPts.push([x, y, z]);
  else chassisPts.push([x, y, z]);
}

function analyzeWheel(name, pts) {
  let minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9, maxZ = -1e9;
  let sumX = 0, sumY = 0, sumZ = 0;
  for (let [x, y, z] of pts) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    sumX += x; sumY += y; sumZ += z;
  }
  let c = [sumX / pts.length, sumY / pts.length, sumZ / pts.length];
  let radX = (maxX - minX) / 2;
  let radY = (maxY - minY) / 2;
  let widthZ = (maxZ - minZ);
  console.log(`${name}: Center=[${c.map(v=>v.toFixed(4))}], RadiusX=${radX.toFixed(4)}, RadiusY=${radY.toFixed(4)}, WidthZ=${widthZ.toFixed(4)}`);
  return { center: c, rad: (radX + radY)/2, width: widthZ, minZ, maxZ };
}

console.log('--- GROUND TRUTH WHEEL MEASUREMENTS ---');
const fl = analyzeWheel('Front-Left  (FL)', flPts);
const fr = analyzeWheel('Front-Right (FR)', frPts);
const rl = analyzeWheel('Rear-Left   (RL)', rlPts);
const rr = analyzeWheel('Rear-Right  (RR)', rrPts);

function analyzeChassis(pts) {
  let minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9, maxZ = -1e9;
  for (let [x, y, z] of pts) {
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  console.log(`Chassis BBox: X=[${minX.toFixed(4)}, ${maxX.toFixed(4)}] Y=[${minY.toFixed(4)}, ${maxY.toFixed(4)}] Z=[${minZ.toFixed(4)}, ${maxZ.toFixed(4)}]`);
}
analyzeChassis(chassisPts);

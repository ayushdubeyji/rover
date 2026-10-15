import { NodeIO } from '@gltf-transform/core';
import { ALL_EXTENSIONS } from '@gltf-transform/extensions';
import { MeshoptDecoder, MeshoptSimplifier } from 'meshoptimizer';
import sharp from 'sharp';

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

// Pre-simplify to 30k
let [s30k] = MeshoptSimplifier.simplify(indices, flatPos, 3, 30000 * 3, 0.01);

const testCounts = [500, 800, 1200, 2000];
for (const targetTri of testCounts) {
  let [sIdx] = MeshoptSimplifier.simplifySloppy(s30k, flatPos, 3, null, targetTri * 3, 0.2);
  let triCount = sIdx.length / 3;
  console.log('Target ' + targetTri + ' -> Actual ' + triCount + ' triangles');

  // Render SVG point cloud / wireframe
  const cT = 0.8387, sT = 0.5446;
  const toView = (wx, wy, wz) => [-wy, wz * cT + wx * sT, wx * cT - wz * sT];
  
  // Render rear 3/4 view (yaw 45, pitch 15)
  const radY = 45 * Math.PI / 180, radP = 15 * Math.PI / 180;
  const cy = Math.cos(radY), sy = Math.sin(radY);
  const cp = Math.cos(radP), sp = Math.sin(radP);

  let lines = '';
  for (let i = 0; i < sIdx.length; i += 3) {
    let pts = [sIdx[i], sIdx[i+1], sIdx[i+2]].map(vi => {
      let x = flatPos[vi*3], y = flatPos[vi*3+1], z = flatPos[vi*3+2];
      // X = fwd, Y = up, Z = width
      let x1 = x * cp - y * sp;
      let y1 = x * sp + y * cp;
      let x2 = x1 * cy - z * sy;
      let z2 = x1 * sy + z * cy;
      let v = toView(x2, z2, y1);
      return [150 + v[0] * 220, 150 - v[1] * 220];
    });
    lines += `<polygon points="${pts[0][0].toFixed(1)},${pts[0][1].toFixed(1)} ${pts[1][0].toFixed(1)},${pts[1][1].toFixed(1)} ${pts[2][0].toFixed(1)},${pts[2][1].toFixed(1)}" fill="rgba(46,120,50,0.6)" stroke="#00ffcc" stroke-width="0.5"/>`;
  }

  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="300" height="300" style="background:#0b0f19;">
    <text x="10" y="20" fill="#fff" font-family="sans-serif" font-size="12">Triangles: ${triCount}</text>
    ${lines}
  </svg>`;
  await sharp(Buffer.from(svg)).png().toFile(`C:/Users/dell/Desktop/rover/tools/test_decimate_${targetTri}.png`);
}
console.log('Finished rendering test decimation views');

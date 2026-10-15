import sharp from 'sharp';

const gt = await sharp('C:/Users/dell/Desktop/rover/tools/ground_truth_glb_render.png').resize(1200, 300).toBuffer();
const s3 = await sharp('C:/Users/dell/Desktop/rover/tools/accurate_mecanum_preview.png').resize(1200, 300).toBuffer();

const titleBar = `<svg width="1200" height="60" xmlns="http://www.w3.org/2000/svg">
  <rect width="1200" height="60" fill="#0b0f19"/>
  <text x="20" y="38" fill="#38bdf8" font-family="sans-serif" font-size="20" font-weight="bold">Tripo GLB Ground Truth vs S3 Remote 3D Mecanum Rover</text>
  <text x="750" y="38" fill="#94a3b8" font-family="sans-serif" font-size="14">Top: Original GLB (12,000 Tris) | Bottom: S3 Controller (581 Tris, 40 FPS)</text>
</svg>`;

const titleBuf = await sharp(Buffer.from(titleBar)).toBuffer();

const composite = await sharp({
  create: {
    width: 1200,
    height: 660,
    channels: 4,
    background: { r: 7, g: 10, b: 19, alpha: 1 }
  }
})
.composite([
  { input: titleBuf, top: 0, left: 0 },
  { input: gt, top: 60, left: 0 },
  { input: s3, top: 360, left: 0 }
])
.png()
.toFile('C:/Users/dell/Desktop/rover/tools/glb_vs_s3_comparison.png');

console.log('Saved glb_vs_s3_comparison.png successfully');

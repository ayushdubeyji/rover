import sharp from 'sharp';

const { data, info } = await sharp('C:/Users/dell/Desktop/rover/tools/test_pbr_rear_3_4.png').raw().toBuffer({ resolveWithObject: true });
const w = info.width, h = info.height;

// Convert to RGB565 array
const rgb565 = new Uint16Array(w * h);
for (let i = 0; i < w * h; i++) {
  const r = data[i * 4], g = data[i * 4 + 1], b = data[i * 4 + 2];
  if (r < 15 && g < 15 && b < 25) {
    rgb565[i] = 0; // Transparent background
  } else {
    let c = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    if (c === 0) c = 0x0821;
    rgb565[i] = c;
  }
}

let totalSpanBytes = 0;
for (let y = 0; y < h; y++) {
  let x = 0;
  while (x < w) {
    while (x < w && rgb565[y * w + x] === 0) x++;
    if (x >= w) break;
    let start = x;
    while (x < w && rgb565[y * w + x] !== 0) x++;
    let len = x - start;
    totalSpanBytes += 2 + len * 2;
  }
}
console.log('128x96 Span bytes:', totalSpanBytes, 'KB:', (totalSpanBytes / 1024).toFixed(1));

// Test 100x75
const res = await sharp('C:/Users/dell/Desktop/rover/tools/test_pbr_rear_3_4.png').resize(100, 75).raw().toBuffer({ resolveWithObject: true });
const d2 = res.data;
const w2 = res.info.width, h2 = res.info.height;
let span2 = 0;
for (let y = 0; y < h2; y++) {
  let x = 0;
  while (x < w2) {
    while (x < w2 && (d2[(y*w2+x)*4] < 15 && d2[(y*w2+x)*4+1] < 15 && d2[(y*w2+x)*4+2] < 25)) x++;
    if (x >= w2) break;
    let start = x;
    while (x < w2 && !(d2[(y*w2+x)*4] < 15 && d2[(y*w2+x)*4+1] < 15 && d2[(y*w2+x)*4+2] < 25)) x++;
    let len = x - start;
    span2 += 2 + len * 2;
  }
}
console.log('100x75 Span bytes:', span2, 'KB:', (span2 / 1024).toFixed(1));

// Test 8-bit indexed palette (256 colors) on 100x75:
// With 8-bit palette: len*1 instead of len*2!
let span8bit = 0;
for (let y = 0; y < h2; y++) {
  let x = 0;
  while (x < w2) {
    while (x < w2 && (d2[(y*w2+x)*4] < 15 && d2[(y*w2+x)*4+1] < 15 && d2[(y*w2+x)*4+2] < 25)) x++;
    if (x >= w2) break;
    let start = x;
    while (x < w2 && !(d2[(y*w2+x)*4] < 15 && d2[(y*w2+x)*4+1] < 15 && d2[(y*w2+x)*4+2] < 25)) x++;
    let len = x - start;
    span8bit += 2 + len * 1;
  }
}
console.log('100x75 8-bit Span bytes:', span8bit, 'KB:', (span8bit / 1024).toFixed(1));

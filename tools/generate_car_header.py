import os
from PIL import Image
import numpy as np

grid_path = r'C:\Users\dell\Desktop\rover\tools\pbr_frames_grid.png'
frame_dir = r'C:\Users\dell\Desktop\rover\tools\pbr_frames'
output_h = r'C:\Users\dell\Desktop\rover\s3_remote\car_pbr_frames.h'

print("Loading grid image for global palette generation...")
grid_img = Image.open(grid_path).convert('RGB')

# Quantize to 255 colors (saving 1 for transparent black)
q_grid = grid_img.quantize(colors=255, method=Image.Quantize.MEDIANCUT)
raw_palette = q_grid.getpalette()[:255*3]

# Palette 0 is transparent black (0, 0, 0) -> RGB565 0x0000
rgb888_palette = [(0, 0, 0)]
rgb565_palette = [0x0000]

for i in range(255):
    r = raw_palette[i*3]
    g = raw_palette[i*3+1]
    b = raw_palette[i*3+2]
    rgb888_palette.append((r, g, b))
    c565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    if c565 == 0 and (r > 0 or g > 0 or b > 0):
        c565 = 0x0821 # avoid pure 0 for visible dark pixels
    rgb565_palette.append(c565)

print(f"Generated global 256-color palette! (256 RGB565 entries)")

# Create an Image object with this exact palette for quantizing individual frames
pal_img = Image.new('P', (1, 1))
flat_pal = []
for r, g, b in rgb888_palette:
    flat_pal.extend([r, g, b])
pal_img.putpalette(flat_pal)

# Process all 36 frames
W, H = 100, 76
encoded_frames = []
total_bytes = 0

for f in range(36):
    frame_path = os.path.join(frame_dir, f'frame_{f:02d}.png')
    im = Image.open(frame_path).convert('RGB')
    
    # Quantize using our global palette
    q_frame = im.quantize(palette=pal_img, dither=0)
    arr = np.array(q_frame)
    
    # Mask out background (pixels in original image that are very dark)
    im_arr = np.array(im)
    is_bg = (im_arr[:, :, 0] < 15) & (im_arr[:, :, 1] < 15) & (im_arr[:, :, 2] < 25)
    arr[is_bg] = 0
    
    # Encode as row spans
    # For each row y:
    #   uint8_t span_count
    #   For each span:
    #     uint8_t x_start, len, pal_indices[len]
    frame_bytes = []
    for y in range(H):
        row = arr[y]
        spans = []
        x = 0
        while x < W:
            while x < W and row[x] == 0:
                x += 1
            if x >= W:
                break
            start = x
            while x < W and row[x] != 0:
                x += 1
            spans.append((start, list(row[start:x])))
        
        frame_bytes.append(len(spans))
        for start, indices in spans:
            frame_bytes.append(start)
            frame_bytes.append(len(indices))
            frame_bytes.extend(indices)
            
    encoded_frames.append(frame_bytes)
    total_bytes += len(frame_bytes)

print(f"Total encoded size for all 36 frames: {total_bytes} bytes ({total_bytes/1024:.1f} KB)")

# Generate car_pbr_frames.h
with open(output_h, 'w') as out:
    out.write("""#pragma once
#include <Arduino.h>

#define CAR_PBR_FRAME_COUNT 36
#define CAR_PBR_FRAME_W 100
#define CAR_PBR_FRAME_H 76

// 256-color global palette in RGB565 (entry 0 is transparent 0x0000)
static const uint16_t CAR_PBR_PALETTE[256] PROGMEM = {
""")
    # Write palette in 8 entries per line
    for i in range(0, 256, 8):
        chunk = [f"0x{rgb565_palette[j]:04X}" for j in range(i, min(i+8, 256))]
        comma = "," if i + 8 < 256 else ""
        out.write(f"  {', '.join(chunk)}{comma}\n")
    out.write("};\n\n")

    # Write each frame
    for f in range(36):
        data = encoded_frames[f]
        out.write(f"// Frame {f}: Yaw {f*10} deg ({len(data)} bytes)\n")
        out.write(f"static const uint8_t CAR_PBR_FRAME_{f}[{len(data)}] PROGMEM = {{\n")
        for i in range(0, len(data), 16):
            chunk = [f"0x{b:02X}" for b in data[i:min(i+16, len(data))]]
            comma = "," if i + 16 < len(data) else ""
            out.write(f"  {', '.join(chunk)}{comma}\n")
        out.write("};\n\n")

    # Write frame pointer array
    out.write("static const uint8_t* const CAR_PBR_FRAMES[CAR_PBR_FRAME_COUNT] PROGMEM = {\n")
    for f in range(36):
        comma = "," if f < 35 else ""
        out.write(f"  CAR_PBR_FRAME_{f}{comma}\n")
    out.write("};\n")

print(f"Generated {output_h} successfully!")

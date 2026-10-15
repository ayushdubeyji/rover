import math
import os
import re
from PIL import Image, ImageDraw, ImageFont

W, H = 240, 240
STRIP_Y = 156

pbr_h_path = r'C:\Users\dell\Desktop\rover\s3_remote\car_pbr_frames.h'
with open(pbr_h_path, 'r') as f:
    code = f.read()

# Parse palette
pal_match = re.search(r'CAR_PBR_PALETTE\[256\] PROGMEM = \{([^}]+)\};', code, re.DOTALL)
pal_vals = [int(x.strip(), 16) for x in pal_match.group(1).replace('\n', '').split(',') if x.strip()]

def rgb565_to_rgb888(c):
    r = ((c >> 11) & 31) * 255 // 31
    g = ((c >> 5) & 63) * 255 // 63
    b = (c & 31) * 255 // 31
    return (r, g, b)

palette = [rgb565_to_rgb888(c) for c in pal_vals]

FW = int(re.search(r'CAR_PBR_FRAME_W\s+(\d+)', code).group(1))
FH = int(re.search(r'CAR_PBR_FRAME_H\s+(\d+)', code).group(1))
COUNT = int(re.search(r'CAR_PBR_FRAME_COUNT\s+(\d+)', code).group(1))

frames = []
for f in range(COUNT):
    m = re.search(rf'CAR_PBR_FRAME_(?:{f:02d}|{f})\s*\[\s*\d*\s*\]\s*PROGMEM\s*=\s*\{{([^}}]+)\}};', code, re.DOTALL)
    vals = [int(x.strip(), 16) for x in m.group(1).replace('\n', '').split(',') if x.strip()]
    frames.append(vals)

print(f"Loaded {len(frames)} PBR frames ({FW}x{FH}) and {len(palette)} palette entries!")

def render_display_frame(yaw_deg, roll_deg=0.0, pitch_deg=0.0):
    im = Image.new('RGB', (W, H), (7, 10, 19))
    draw = ImageDraw.Draw(im)

    # 1. Header (y=0..14)
    draw.line([(0, 14), (W, 14)], fill=(49, 134, 134))
    draw.text((4, 2), "ATTITUDE", fill=(0, 255, 255))
    draw.ellipse((147, 5, 153, 11), fill=(0, 255, 0))
    draw.text((157, 2), "LINK", fill=(0, 255, 0))
    draw.text((194, 2), "4.1V", fill=(255, 255, 255))

    # 2. PBR Rover Frame
    cx, cy = 120, 76
    py_off = -(int)(pitch_deg * 0.45)
    px_off = (int)(roll_deg * 0.45)

    # Drop shadow
    draw.ellipse((cx + px_off - 46, cy + 30 + py_off - 9, cx + px_off + 46, cy + 30 + py_off + 9), fill=(24, 28, 32))

    fIdx = int(round(yaw_deg / 10.0)) % COUNT
    data = frames[fIdx]

    top_x = cx - FW // 2 + px_off
    top_y = cy - FH // 2 + py_off

    p = 0
    for y in range(FH):
        span_count = data[p]
        p += 1
        screenY = top_y + y
        if screenY < 15 or screenY >= 155:
            for s in range(span_count):
                p += 2 + data[p+1]
            continue
        for s in range(span_count):
            x_start = data[p]
            len_s = data[p+1]
            p += 2
            screenX = top_x + x_start
            for i in range(len_s):
                pal_idx = data[p]
                p += 1
                col = palette[pal_idx]
                px = screenX + i
                if 0 <= px < W:
                    im.putpixel((px, screenY), col)

    # 3. Gizmo
    gx, gy, glen = 206, 42, 16
    draw.line([(gx, gy), (gx + glen, gy)], fill=(255, 0, 0), width=2)
    draw.text((gx + glen + 2, gy - 4), "X", fill=(255, 0, 0))
    draw.line([(gx, gy), (gx, gy - glen)], fill=(0, 255, 0), width=2)
    draw.text((gx - 4, gy - glen - 10), "Y", fill=(0, 255, 0))
    draw.line([(gx, gy), (gx - 10, gy + 10)], fill=(0, 100, 255), width=2)
    draw.text((gx - 16, gy + 10), "Z", fill=(0, 100, 255))

    # 4. Telemetry Text
    draw.text((4, 22), f"R {roll_deg:+5.1f}", fill=(123, 190, 239))
    draw.text((4, 32), f"P {pitch_deg:+5.1f}", fill=(123, 190, 239))
    draw.text((4, 42), f"Y {yaw_deg:+5.1f}", fill=(123, 190, 239))

    draw.text((178, 92), "G    +0", fill=(123, 190, 239))
    draw.text((178, 102), "A 1.00g", fill=(123, 190, 239))

    draw.text((46, 144), "[PBR 3D DIGITAL TWIN]", fill=(0, 255, 0))

    # 5. Bottom Info Strip
    draw.line([(0, STRIP_Y), (W, STRIP_Y)], fill=(0, 255, 0))
    draw.text((12, STRIP_Y + 10), "SPD  0.00   BAT 4.1V", fill=(255, 255, 255))
    draw.text((12, STRIP_Y + 34), "HDG    0   NORMAL", fill=(255, 255, 255))
    draw.text((12, STRIP_Y + 58), "TOF  45cm  US  60cm", fill=(255, 255, 255))

    return im

print("Rendering 4 simulation angles...")
f1 = render_display_frame(0.0, 0.0, 0.0)
f2 = render_display_frame(45.0, 0.0, 10.0)
f3 = render_display_frame(110.0, 15.0, 0.0)
f4 = render_display_frame(180.0, 0.0, 0.0)

comp = Image.new('RGB', (W * 4, H))
comp.paste(f1, (0, 0))
comp.paste(f2, (W, 0))
comp.paste(f3, (W * 2, 0))
comp.paste(f4, (W * 3, 0))

out_path = r'C:\Users\dell\Desktop\rover\tools\screen_pbr_preview.png'
comp.save(out_path)
print(f"Saved {out_path} successfully!")

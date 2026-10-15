from PIL import Image
import numpy as np

img = Image.open(r"C:\Users\dell\Desktop\rover\tools\tex_0.png")
w, h = img.size
print(f"Texture size: {w}x{h}")

# Let's inspect car_model.h
with open(r"C:\Users\dell\Desktop\rover\s3_remote\car_model.h", "r") as f:
    text = f.read()

import re
colors = re.findall(r'0x[0-9A-Fa-f]{4}', text)
print(f"Total colors in car_model.h: {len(colors)}")

rgb_list = []
for c_str in colors:
    val = int(c_str, 16)
    r = ((val >> 11) & 31) * 8
    g = ((val >> 5) & 63) * 4
    b = (val & 31) * 8
    rgb_list.append((r, g, b))

avg_r = np.mean([c[0] for c in rgb_list])
avg_g = np.mean([c[1] for c in rgb_list])
avg_b = np.mean([c[2] for c in rgb_list])
print(f"Avg RGB of car_model.h colors: ({avg_r:.1f}, {avg_g:.1f}, {avg_b:.1f})")

# Let's see some top brightest colors
bright = sorted(rgb_list, key=lambda c: c[0]+c[1]+c[2], reverse=True)[:10]
print("10 brightest colors:", bright)

# Let's see how many colors are dark (sum < 100)
dark_count = sum(1 for c in rgb_list if c[0]+c[1]+c[2] < 100)
print(f"Dark colors (<100 total intensity): {dark_count} / {len(rgb_list)}")

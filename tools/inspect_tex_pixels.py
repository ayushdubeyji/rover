import struct
import json
import numpy as np
from PIL import Image

glb_path = r"C:\Users\dell\Downloads\tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb"

with open(glb_path, 'rb') as f:
    magic = f.read(4)
    version, length = struct.unpack('<II', f.read(8))
    chunk0_len, chunk0_type = struct.unpack('<II', f.read(8))
    gltf = json.loads(f.read(chunk0_len).decode('utf-8'))
    chunk1_len, chunk1_type = struct.unpack('<II', f.read(8))
    bin_data = f.read(chunk1_len)

# Let's inspect where the texture is located and what the colors are
img = Image.open(r"C:\Users\dell\Desktop\rover\tools\tex_0.png")
arr = np.array(img)
print("Texture shape:", arr.shape)
print("Min RGB:", arr.min(axis=(0,1)), "Max RGB:", arr.max(axis=(0,1)), "Mean RGB:", arr.mean(axis=(0,1)))

# How many pixels are bright green?
# Green: G > 100 and G > R*1.3 and G > B*1.3
green_mask = (arr[:,:,1] > 100) & (arr[:,:,1] > arr[:,:,0] * 1.2) & (arr[:,:,1] > arr[:,:,2] * 1.2)
print("Green pixel percentage:", np.sum(green_mask) / (arr.shape[0]*arr.shape[1]) * 100, "%")

# How many pixels are dark/black?
dark_mask = (arr[:,:,0] < 50) & (arr[:,:,1] < 50) & (arr[:,:,2] < 50)
print("Dark pixel percentage:", np.sum(dark_mask) / (arr.shape[0]*arr.shape[1]) * 100, "%")

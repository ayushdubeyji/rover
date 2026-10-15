import struct
import json
from PIL import Image
import numpy as np

glb_path = r"C:\Users\dell\Downloads\tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb"

with open(glb_path, 'rb') as f:
    magic = f.read(4)
    version, length = struct.unpack('<II', f.read(8))
    chunk0_len, chunk0_type = struct.unpack('<II', f.read(8))
    gltf = json.loads(f.read(chunk0_len).decode('utf-8'))
    chunk1_len, chunk1_type = struct.unpack('<II', f.read(8))
    bin_data = f.read(chunk1_len)

# Extract UV accessor
prim = gltf['meshes'][0]['primitives'][0]
pos_acc = gltf['accessors'][prim['attributes']['POSITION']]
uv_acc = gltf['accessors'][prim['attributes']['TEXCOORD_0']]

pos_bv = gltf['bufferViews'][pos_acc['bufferView']]
uv_bv = gltf['bufferViews'][uv_acc['bufferView']]

pos_bytes = bin_data[pos_bv.get('byteOffset', 0): pos_bv.get('byteOffset', 0) + pos_bv['byteLength']]
uv_bytes = bin_data[uv_bv.get('byteOffset', 0): uv_bv.get('byteOffset', 0) + uv_bv['byteLength']]

# Load texture
tex = Image.open(r"C:\Users\dell\Desktop\rover\tools\tex_0.png")
tw, th = tex.size
pixels = np.array(tex)

# Sample 100 points with V vs (1-V)
# In tripo: what is the color of the hood / roof?
# Roof is at max Y (p[1] > 0.18)
print("Texture size:", tw, "x", th)
print("Testing UV mapping...")

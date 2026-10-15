import struct
import json
import os
from PIL import Image
import io

glb_path = r"C:\Users\dell\Downloads\tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb"

with open(glb_path, 'rb') as f:
    magic = f.read(4)
    version, length = struct.unpack('<II', f.read(8))
    chunk0_len, chunk0_type = struct.unpack('<II', f.read(8))
    gltf = json.loads(f.read(chunk0_len).decode('utf-8'))
    chunk1_len, chunk1_type = struct.unpack('<II', f.read(8))
    bin_data = f.read(chunk1_len)

print("GLTF keys:", gltf.keys())
print("Buffer views:", len(gltf.get('bufferViews', [])))
print("Accessors:", len(gltf.get('accessors', [])))

# Extract images
images = gltf.get('images', [])
for i, img in enumerate(images):
    bv_idx = img['bufferView']
    bv = gltf['bufferViews'][bv_idx]
    offset = bv.get('byteOffset', 0)
    length = bv['byteLength']
    img_bytes = bin_data[offset:offset+length]
    im = Image.open(io.BytesIO(img_bytes))
    print(f"Image {i}: {img.get('name')} format={im.format} size={im.size} mode={im.mode}")
    out_name = f"C:\\Users\\dell\\Desktop\\rover\\tools\\tex_{i}.png"
    im.save(out_name)
    print("Saved texture to", out_name)

# Inspect mesh attributes
mesh = gltf['meshes'][0]
prim = mesh['primitives'][0]
print("Primitive attributes:", prim['attributes'].keys())
print("Indices accessor:", prim.get('indices'))

def get_accessor_data(acc_idx):
    acc = gltf['accessors'][acc_idx]
    bv = gltf['bufferViews'][acc['bufferView']]
    offset = bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
    count = acc['count']
    ctype = acc['componentType']
    atype = acc['type']
    print(f"Accessor {acc_idx}: count={count}, ctype={ctype}, atype={atype}, min={acc.get('min')}, max={acc.get('max')}")

for name, acc_idx in prim['attributes'].items():
    print(f"Attr {name}:")
    get_accessor_data(acc_idx)
if 'indices' in prim:
    print("Indices:")
    get_accessor_data(prim['indices'])

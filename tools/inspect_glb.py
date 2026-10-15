import struct
import json

glb_path = r"C:\Users\dell\Downloads\tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb"

with open(glb_path, 'rb') as f:
    magic = f.read(4)
    version, length = struct.unpack('<II', f.read(8))
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    chunk_data = f.read(chunk_len)
    gltf = json.loads(chunk_data.decode('utf-8'))

print("Meshes:", len(gltf.get('meshes', [])))
print("Materials:", len(gltf.get('materials', [])))
print("Textures:", len(gltf.get('textures', [])))
print("Images:", len(gltf.get('images', [])))
if 'materials' in gltf:
    for i, m in enumerate(gltf['materials']):
        print(f"Material {i}:", m)
if 'images' in gltf:
    for i, img in enumerate(gltf['images']):
        print(f"Image {i}:", img.get('name'), img.get('mimeType'), img.get('bufferView'))

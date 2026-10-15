import struct, json

glb_path = r"C:\Users\dell\Downloads\tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb"
with open(glb_path, 'rb') as f:
    f.seek(12)
    chunk_len, chunk_type = struct.unpack('<II', f.read(8))
    gltf = json.loads(f.read(chunk_len).decode('utf-8'))

for m in gltf.get('materials', []):
    print("Material:", json.dumps(m, indent=2))

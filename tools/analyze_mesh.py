import trimesh
import numpy as np
from PIL import Image

# Load GLB
scene = trimesh.load(r"C:\Users\dell\Downloads\tripo_pbr_model_4dc5e0b1-5d1e-4e35-adc5-3f2d39eaed06_meshopt.glb")
mesh = scene.dump(concatenate=True) if hasattr(scene, 'dump') else scene

print("Mesh vertices:", len(mesh.vertices))
print("Mesh faces:", len(mesh.faces))
print("Mesh bounds:\n", mesh.bounds)
print("Mesh extents (dx, dy, dz):", mesh.extents)

# Check colors in texture
img = Image.open(r"C:\Users\dell\Desktop\rover\tools\tex_0.png")
print("Texture loaded, size:", img.size)

# Sample some UVs
if hasattr(mesh.visual, 'uv') and mesh.visual.uv is not None:
    print("UV min/max:", mesh.visual.uv.min(axis=0), mesh.visual.uv.max(axis=0))

import http.server
import socketserver
import threading
import subprocess
import os
import sys
import json
import urllib.parse
from PIL import Image
import numpy as np

PORT = 8898
BASE_DIR = r"C:\Users\dell\Desktop\rover\tools"
GLB_PATH = os.path.join(BASE_DIR, "model.glb")

received_data = {}
done_event = threading.Event()

class MeshHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        length = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(length)

        if parsed.path == "/save_positions":
            received_data["positions"] = data
            print(f"Received positions: {len(data)} bytes ({len(data)//12} vertices)")
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/save_uvs":
            received_data["uvs"] = data
            print(f"Received UVs: {len(data)} bytes ({len(data)//8} UV pairs)")
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/save_indices":
            received_data["indices"] = data
            print(f"Received indices: {len(data)} bytes ({len(data)//12} triangles)")
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/save_texture":
            tex_path = os.path.join(BASE_DIR, "rover_texture_4k.jpg")
            with open(tex_path, "wb") as f:
                f.write(data)
            received_data["texture_path"] = tex_path
            print(f"Received texture: {len(data)} bytes -> {tex_path}")
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/done":
            print("Received DONE signal from extractor!")
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            done_event.set()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

html_content = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<script src="js/three.min.js"></script>
<script src="js/meshopt_decoder.js"></script>
<script src="js/GLTFLoader.js"></script>
</head>
<body>
<h1>Extracting Decoded Mesh...</h1>
<script>
async function run() {
  const loader = new THREE.GLTFLoader();
  if (typeof MeshoptDecoder !== 'undefined') {
    loader.setMeshoptDecoder(MeshoptDecoder);
  }

  loader.load('model.glb', async (gltf) => {
    let targetMesh = null;
    gltf.scene.traverse((child) => {
      if (child.isMesh && !targetMesh) {
        targetMesh = child;
      }
    });

    if (!targetMesh) {
      console.error("No mesh found in GLTF!");
      return;
    }

    const geo = targetMesh.geometry;
    const pos = geo.attributes.position.array;
    const uv = geo.attributes.uv.array;
    let idx = geo.index ? geo.index.array : null;

    if (!idx) {
      // non-indexed, build indices
      const n = pos.length / 3;
      idx = new Uint32Array(n);
      for (let i = 0; i < n; i++) idx[i] = i;
    } else if (!(idx instanceof Uint32Array)) {
      idx = new Uint32Array(idx);
    }

    console.log("Extracted positions:", pos.length, "UVs:", uv.length, "Indices:", idx.length);

    // Send binary buffers
    await fetch('/save_positions', { method: 'POST', body: pos.buffer });
    await fetch('/save_uvs', { method: 'POST', body: uv.buffer });
    await fetch('/save_indices', { method: 'POST', body: idx.buffer });

    // Extract texture image if present
    if (targetMesh.material && targetMesh.material.map && targetMesh.material.map.image) {
      const img = targetMesh.material.map.image;
      const canvas = document.createElement('canvas');
      canvas.width = img.width;
      canvas.height = img.height;
      const ctx = canvas.getContext('2d');
      ctx.drawImage(img, 0, 0);
      canvas.toBlob(async (blob) => {
        await fetch('/save_texture', { method: 'POST', body: blob });
        await fetch('/done', { method: 'POST' });
      }, 'image/jpeg', 0.95);
    } else {
      await fetch('/done', { method: 'POST' });
    }
  }, undefined, (err) => {
    console.error("Error loading GLTF:", err);
  });
}
window.onload = run;
</script>
</body>
</html>
"""

html_path = os.path.join(BASE_DIR, "extract_decoded_glb.html")
with open(html_path, "w", encoding="utf-8") as f:
    f.write(html_content)

server = socketserver.ThreadingTCPServer(("127.0.0.1", PORT), MeshHandler)
server_thread = threading.Thread(target=server.serve_forever, daemon=True)
server_thread.start()
print(f"Extraction server running on http://127.0.0.1:{PORT}")

chrome_exe = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
cmd = [
    chrome_exe,
    "--headless=new",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--allow-file-access-from-files",
    f"http://127.0.0.1:{PORT}/extract_decoded_glb.html"
]

print("Launching Chrome to decode meshopt geometry...")
proc = subprocess.Popen(cmd)

success = done_event.wait(timeout=120)
try:
    proc.terminate()
except Exception:
    pass
server.shutdown()

if not success or "positions" not in received_data or "indices" not in received_data:
    print("Failed to extract mesh!")
    sys.exit(1)

print("Decoded mesh geometry successfully extracted from GLTF!")
pos = np.frombuffer(received_data["positions"], dtype=np.float32).reshape(-1, 3)
uv = np.frombuffer(received_data["uvs"], dtype=np.float32).reshape(-1, 2)
idx = np.frombuffer(received_data["indices"], dtype=np.uint32).reshape(-1, 3)

print(f"Decoded Mesh: {len(pos)} vertices, {len(idx)} triangles, {len(uv)} UVs")
np.save(os.path.join(BASE_DIR, "decoded_positions.npy"), pos)
np.save(os.path.join(BASE_DIR, "decoded_uvs.npy"), uv)
np.save(os.path.join(BASE_DIR, "decoded_indices.npy"), idx)
print("Saved decoded numpy arrays!")

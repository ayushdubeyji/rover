import http.server
import socketserver
import threading
import subprocess
import os
import sys
import time
import urllib.parse
from PIL import Image
import numpy as np

PORT = 8899
BASE_DIR = r"C:\Users\dell\Desktop\rover\tools"
FRAMES_DIR = os.path.join(BASE_DIR, "pbr_web_frames")
os.makedirs(FRAMES_DIR, exist_ok=True)

received_frames = {}
done_event = threading.Event()

class FrameHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/save_frame":
            qs = urllib.parse.parse_qs(parsed.query)
            idx = int(qs.get("idx", [0])[0])
            length = int(self.headers.get("Content-Length", 0))
            data = self.rfile.read(length)
            frame_path = os.path.join(FRAMES_DIR, f"frame_{idx:02d}.png")
            with open(frame_path, "wb") as f:
                f.write(data)
            received_frames[idx] = frame_path
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/done":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            print("Received DONE signal from renderer!")
            done_event.set()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

W, H = 140, 96
html_content = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<script src="js/three.min.js"></script>
<script src="js/meshopt_decoder.js"></script>
<script src="js/GLTFLoader.js"></script>
<style>
  body {{ margin:0; padding:0; background: transparent; overflow: hidden; }}
  canvas {{ display: block; }}
</style>
</head>
<body>
<canvas id="c" width="{W}" height="{H}"></canvas>
<script>
async function run() {{
  const canvas = document.getElementById('c');
  const renderer = new THREE.WebGLRenderer({{
    canvas: canvas,
    alpha: true,
    antialias: true,
    preserveDrawingBuffer: true
  }});
  renderer.setSize({W}, {H});
  renderer.setPixelRatio(1);
  if (THREE.sRGBEncoding) renderer.outputEncoding = THREE.sRGBEncoding;

  const scene = new THREE.Scene();

  // Camera at 33 degree elevation, distance 1.42
  const camera = new THREE.PerspectiveCamera(30, {W}/{H}, 0.1, 50);
  camera.position.set(1.42, 0.92, 0.0);
  camera.lookAt(0, -0.05, 0);

  // Exact Web Dashboard Lighting Rig
  const amb = new THREE.AmbientLight(0xffffff, 1.4);
  scene.add(amb);

  const dir = new THREE.DirectionalLight(0xffffff, 2.2);
  dir.position.set(2.5, 4.0, 1.5);
  scene.add(dir);

  const fill = new THREE.DirectionalLight(0x38bdf8, 0.9);
  fill.position.set(-2.5, 2.0, -1.5);
  scene.add(fill);

  const loader = new THREE.GLTFLoader();
  if (typeof MeshoptDecoder !== 'undefined') {{
    loader.setMeshoptDecoder(MeshoptDecoder);
  }}

  loader.load('model.glb', async (gltf) => {{
    const root = gltf.scene;
    const pivot = new THREE.Group();
    scene.add(pivot);

    const box = new THREE.Box3().setFromObject(root);
    const center = box.getCenter(new THREE.Vector3());
    root.position.sub(center);
    pivot.add(root);

    root.traverse((child) => {{
      if (child.isMesh && child.material) {{
        child.material.roughness = 0.55;
        child.material.metalness = 0.35;
      }}
    }});

    // 36 frames at 10 deg steps
    for (let f = 0; f < 36; f++) {{
      const yawDeg = f * 10;
      pivot.rotation.y = -(yawDeg * Math.PI / 180.0);

      renderer.render(scene, camera);

      await new Promise((resolve) => {{
        canvas.toBlob(async (blob) => {{
          await fetch('/save_frame?idx=' + f, {{ method: 'POST', body: blob }});
          resolve();
        }}, 'image/png');
      }});
    }}

    await fetch('/done', {{ method: 'POST' }});
  }}, undefined, (err) => {{
    console.error('GLTF load error:', err);
  }});
}}
window.onload = run;
</script>
</body>
</html>
"""

html_path = os.path.join(BASE_DIR, "render_pbr_threejs.html")
with open(html_path, "w", encoding="utf-8") as f:
    f.write(html_content)

server = socketserver.ThreadingTCPServer(("127.0.0.1", PORT), FrameHandler)
server_thread = threading.Thread(target=server.serve_forever, daemon=True)
server_thread.start()
print(f"HTTP server running on http://127.0.0.1:{PORT}")

chrome_exe = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
cmd = [
    chrome_exe,
    "--headless=new",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--allow-file-access-from-files",
    f"http://127.0.0.1:{PORT}/render_pbr_threejs.html"
]

print("Launching Chrome headless to render all 36 Three.js PBR views...")
proc = subprocess.Popen(cmd)

success = done_event.wait(timeout=180)
try:
    proc.terminate()
except Exception:
    pass
server.shutdown()

if not success or len(received_frames) < 36:
    print(f"Failed to receive all frames! Received: {len(received_frames)}")
    sys.exit(1)

print("All 36 Three.js PBR frames successfully captured!")

# Check bounding boxes
min_x_all, min_y_all, max_x_all, max_y_all = 999, 999, 0, 0
for f in range(36):
    img = Image.open(received_frames[f])
    bbox = img.getbbox()
    min_x_all = min(min_x_all, bbox[0])
    min_y_all = min(min_y_all, bbox[1])
    max_x_all = max(max_x_all, bbox[2])
    max_y_all = max(max_y_all, bbox[3])

print(f"\nOverall bbox across all 36 frames: x=[{min_x_all}, {max_x_all}] / {W}, y=[{min_y_all}, {max_y_all}] / {H}")

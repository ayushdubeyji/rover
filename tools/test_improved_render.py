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
FRAMES_DIR = os.path.join(BASE_DIR, "test_render_out")
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
            name = qs.get("name", ["frame"])[0]
            length = int(self.headers.get("Content-Length", 0))
            data = self.rfile.read(length)
            frame_path = os.path.join(FRAMES_DIR, f"{name}.png")
            with open(frame_path, "wb") as f:
                f.write(data)
            received_frames[name] = frame_path
            print(f"Captured {name} ({len(data)} bytes)", flush=True)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/done":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            print("DONE!", flush=True)
            done_event.set()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

# Render at 2x resolution: 280 x 192, then downscale to 140 x 96
W2, H2 = 280, 192

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
<canvas id="c" width="{W2}" height="{H2}"></canvas>
<script>
async function run() {{
  const canvas = document.getElementById('c');
  const renderer = new THREE.WebGLRenderer({{
    canvas: canvas,
    alpha: true,
    antialias: true,
    preserveDrawingBuffer: true
  }});
  renderer.setSize({W2}, {H2});
  renderer.setPixelRatio(1);
  if (THREE.sRGBEncoding) renderer.outputEncoding = THREE.sRGBEncoding;

  const scene = new THREE.Scene();

  // Camera placed behind the rover (-X) elevated at 33 degrees looking forward along +X:
  // Rear chase view: camera is at -X. Battery is in rear facing camera, front electronics point forward.
  const camera = new THREE.PerspectiveCamera(30, {W2}/{H2}, 0.1, 50);
  camera.position.set(-1.48, 0.96, 0.0);
  camera.lookAt(0, -0.04, 0);

  // Balanced studio lighting (authentic diffuse colors, no cyan tint):
  const amb = new THREE.AmbientLight(0xffffff, 1.1);
  scene.add(amb);

  const dir1 = new THREE.DirectionalLight(0xfffdfa, 1.8);
  dir1.position.set(-2.0, 3.5, 2.0);
  scene.add(dir1);

  const dir2 = new THREE.DirectionalLight(0xf2f6fa, 1.2);
  dir2.position.set(2.0, 2.5, -2.0);
  scene.add(dir2);

  const dir3 = new THREE.DirectionalLight(0xffffff, 0.7);
  dir3.position.set(-2.5, 1.5, -2.0);
  scene.add(dir3);

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

    // Keep original PBR maps! Do NOT overwrite metalness/roughness with constant values!

    const testAngles = [
      {{ name: 'chase_yaw0', yaw: 0 }},
      {{ name: 'angle_yaw45', yaw: 45 }},
      {{ name: 'side_yaw90', yaw: 90 }},
      {{ name: 'front_yaw180', yaw: 180 }}
    ];

    for (const item of testAngles) {{
      pivot.rotation.y = -(item.yaw * Math.PI / 180.0);
      renderer.render(scene, camera);

      await new Promise((resolve) => {{
        canvas.toBlob(async (blob) => {{
          await fetch('/save_frame?name=' + item.name, {{ method: 'POST', body: blob }});
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

html_path = os.path.join(BASE_DIR, "test_render.html")
with open(html_path, "w", encoding="utf-8") as f:
    f.write(html_content)

socketserver.TCPServer.allow_reuse_address = True
server = socketserver.ThreadingTCPServer(("127.0.0.1", PORT), FrameHandler)
server_thread = threading.Thread(target=server.serve_forever, daemon=True)
server_thread.start()
print(f"HTTP server running on http://127.0.0.1:{PORT}", flush=True)

chrome_exe = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
cmd = [
    chrome_exe,
    "--headless=new",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--allow-file-access-from-files",
    f"http://127.0.0.1:{PORT}/test_render.html"
]

print("Launching Chrome headless...", flush=True)
proc = subprocess.Popen(cmd)
success = done_event.wait(timeout=60)
try:
    proc.terminate()
except Exception:
    pass
server.shutdown()

print("Downscaling test renders with Lanczos...")
for name in ['chase_yaw0', 'angle_yaw45', 'side_yaw90', 'front_yaw180']:
    p = os.path.join(FRAMES_DIR, f"{name}.png")
    if os.path.exists(p):
        im = Image.open(p)
        im_down = im.resize((140, 96), Image.Resampling.LANCZOS)
        out_p = os.path.join(FRAMES_DIR, f"{name}_140x96.png")
        im_down.save(out_p)
        print(f"Saved {out_p}")

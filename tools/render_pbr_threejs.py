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

PORT = 8898
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
            print(f"Captured frame {idx:02d}/35 ({len(data)} bytes)", flush=True)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/done":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            print("Received DONE signal from renderer!", flush=True)
            done_event.set()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass

W, H = 140, 96
FRAME_COUNT = 36

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

  // 33 degree elevation, distance 1.48 to fit 140x96 with margin
  const camera = new THREE.PerspectiveCamera(30, {W}/{H}, 0.1, 50);
  camera.position.set(1.48, 0.96, 0.0);
  camera.lookAt(0, -0.04, 0);

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

    for (let f = 0; f < {FRAME_COUNT}; f++) {{
      const yawDeg = f * 10;
      // Clockwise vehicle rotation for positive yaw heading
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
    f"http://127.0.0.1:{PORT}/render_pbr_threejs.html"
]

print("Launching Chrome headless to render all 36 Three.js PBR views...", flush=True)
proc = subprocess.Popen(cmd)

success = done_event.wait(timeout=240)
try:
    proc.terminate()
except Exception:
    pass
server.shutdown()

if not success or len(received_frames) < FRAME_COUNT:
    print(f"Failed to receive all frames! Received: {len(received_frames)}/{FRAME_COUNT}", flush=True)
    sys.exit(1)

print("All 36 Three.js PBR frames successfully captured!", flush=True)

# Step 2: Global Palette Quantization and Row-span RLE Compression
print(f"Generating global 256-color palette from actual rover pixels (Frame size {W}x{H})...", flush=True)

car_pixels = []
frame_images = []
min_x_all, min_y_all, max_x_all, max_y_all = 999, 999, 0, 0

for f in range(FRAME_COUNT):
    img = Image.open(received_frames[f]).convert("RGBA")
    frame_images.append(img)
    bbox = img.getbbox()
    min_x_all = min(min_x_all, bbox[0])
    min_y_all = min(min_y_all, bbox[1])
    max_x_all = max(max_x_all, bbox[2])
    max_y_all = max(max_y_all, bbox[3])
    
    arr = np.array(img)
    alpha = arr[:, :, 3]
    visible = arr[alpha > 40][:, :3]
    if len(visible) > 0:
        car_pixels.append(visible)

print(f"Rover Bounding Box across all 36 frames: x=[{min_x_all}, {max_x_all}] / {W}, y=[{min_y_all}, {max_y_all}] / {H}", flush=True)

all_visible = np.vstack(car_pixels)
print(f"Total visible rover pixel samples: {len(all_visible)}", flush=True)

# Quantize to 255 colors (entry 0 is transparent 0x0000)
vis_img = Image.fromarray(all_visible.reshape(-1, 1, 3))
vis_quant = vis_img.quantize(colors=255, method=Image.Quantize.MEDIANCUT)
raw_palette = vis_quant.getpalette()[:255*3]

rgb565_palette = [0x0000]
rgb888_palette = [(0, 0, 0)]

for i in range(255):
    r = raw_palette[i*3]
    g = raw_palette[i*3+1]
    b = raw_palette[i*3+2]
    rgb888_palette.append((r, g, b))
    c565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    if c565 == 0 and (r > 0 or g > 0 or b > 0):
        c565 = 0x0821 # avoid pure 0 for visible dark pixels
    rgb565_palette.append(c565)

# Build PIL palette image
pal_img = Image.new('P', (1, 1))
flat_pal = []
for r, g, b in rgb888_palette:
    flat_pal.extend([r, g, b])
pal_img.putpalette(flat_pal)

# Step 3: Encode each frame into row spans
encoded_frames = []
total_bytes = 0

for f in range(FRAME_COUNT):
    img = frame_images[f]
    arr_rgba = np.array(img)
    alpha = arr_rgba[:, :, 3]
    rgb = img.convert('RGB')
    
    q_frame = rgb.quantize(palette=pal_img, dither=Image.Dither.NONE)
    arr_q = np.array(q_frame)
    arr_q[alpha <= 40] = 0
    
    frame_bytes = []
    for y in range(H):
        row = arr_q[y]
        spans = []
        x = 0
        while x < W:
            while x < W and row[x] == 0:
                x += 1
            if x >= W:
                break
            start = x
            while x < W and row[x] != 0:
                x += 1
            spans.append((start, list(row[start:x])))
        
        frame_bytes.append(len(spans))
        for start, indices in spans:
            frame_bytes.append(start)
            frame_bytes.append(len(indices))
            frame_bytes.extend(indices)
            
    encoded_frames.append(frame_bytes)
    total_bytes += len(frame_bytes)

print(f"Total encoded size for all {FRAME_COUNT} frames: {total_bytes} bytes ({total_bytes/1024:.1f} KB)", flush=True)

# Step 4: Write car_pbr_frames.h
header_path = r"C:\Users\dell\Desktop\rover\s3_remote\car_pbr_frames.h"
with open(header_path, "w") as out:
    out.write(f"""#pragma once
#include <Arduino.h>

#define CAR_PBR_FRAME_COUNT {FRAME_COUNT}
#define CAR_PBR_FRAME_W {W}
#define CAR_PBR_FRAME_H {H}

// 256-color global palette in RGB565 (entry 0 is transparent 0x0000)
static const uint16_t CAR_PBR_PALETTE[256] PROGMEM = {{
""")
    for i in range(0, 256, 8):
        line = "  " + ", ".join(f"0x{rgb565_palette[j]:04X}" for j in range(i, i+8)) + ",\n"
        out.write(line)
    out.write("};\n\n")

    for f in range(FRAME_COUNT):
        out.write(f"// Frame {f:02d}: Yaw {f*10} deg ({len(encoded_frames[f])} bytes)\n")
        out.write(f"static const uint8_t CAR_PBR_FRAME_{f:02d}[] PROGMEM = {{\n")
        fbytes = encoded_frames[f]
        for i in range(0, len(fbytes), 16):
            chunk = fbytes[i:i+16]
            out.write("  " + ", ".join(f"0x{b:02X}" for b in chunk) + ",\n")
        out.write("};\n\n")

    out.write(f"static const uint8_t * const CAR_PBR_FRAMES[{FRAME_COUNT}] PROGMEM = {{\n")
    for f in range(FRAME_COUNT):
        out.write(f"  CAR_PBR_FRAME_{f:02d},\n")
    out.write("};\n")

print(f"Wrote {header_path} successfully!", flush=True)

# Step 5: Generate 6x6 sprite grid preview
grid_img = Image.new("RGBA", (W * 6, H * 6), (7, 10, 19, 255))
for f in range(FRAME_COUNT):
    gx = (f % 6) * W
    gy = (f // 6) * H
    grid_img.paste(frame_images[f], (gx, gy), frame_images[f])

grid_path = os.path.join(BASE_DIR, "pbr_frames_grid.png")
grid_img.save(grid_path)
print(f"Saved 36-frame sprite grid to {grid_path}", flush=True)

print("Three.js 36-frame PBR pipeline completed successfully!", flush=True)

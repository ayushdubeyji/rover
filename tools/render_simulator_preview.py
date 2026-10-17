import http.server
import socketserver
import threading
import subprocess
import os
import sys
import time
import urllib.parse
from PIL import Image

PORT = 8896
BASE_DIR = r"C:\Users\dell\Desktop\rover\tools"
FRAMES_DIR = os.path.join(BASE_DIR, "sim_renders")
os.makedirs(FRAMES_DIR, exist_ok=True)

done_event = threading.Event()
saved_frames = {}

class SimMultiHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/save_frame":
            qs = urllib.parse.parse_qs(parsed.query)
            name = qs.get("name", ["frame"])[0]
            length = int(self.headers.get("Content-Length", 0))
            data = self.rfile.read(length)
            fpath = os.path.join(FRAMES_DIR, f"{name}.png")
            with open(fpath, "wb") as f:
                f.write(data)
            saved_frames[name] = fpath
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
        elif parsed.path == "/done":
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
<style>
  body { margin: 0; background: #060913; overflow: hidden; }
  #container { width: 960px; height: 540px; }
</style>
<script src="js/three.min.js"></script>
<script src="js/meshopt_decoder.js"></script>
<script src="js/GLTFLoader.js"></script>
</head>
<body>
<div id="container"></div>

<script>
window.onload = function() {
  const container = document.getElementById('container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x060913);
  scene.fog = new THREE.FogExp2(0x060913, 0.04);

  const camera = new THREE.PerspectiveCamera(42, 960 / 540, 0.1, 100);

  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  renderer.setSize(960, 540);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  if (THREE.sRGBEncoding) renderer.outputEncoding = THREE.sRGBEncoding;
  container.appendChild(renderer.domElement);

  // Lighting Rig
  const ambient = new THREE.AmbientLight(0xffffff, 1.4);
  scene.add(ambient);
  const sun = new THREE.DirectionalLight(0xffffff, 2.8);
  sun.position.set(4, 7, 3.5);
  sun.castShadow = true;
  scene.add(sun);
  const skyFill = new THREE.DirectionalLight(0x38bdf8, 1.2);
  skyFill.position.set(-4, 3, -3);
  scene.add(skyFill);

  // Arena Grid
  const grid = new THREE.GridHelper(16, 80, 0x0284c7, 0x1e293b);
  scene.add(grid);

  // Ground floor
  const floorGeo = new THREE.PlaneGeometry(24, 24);
  const floorMat = new THREE.MeshStandardMaterial({ color: 0x060913, roughness: 0.9, metalness: 0.1 });
  const floor = new THREE.Mesh(floorGeo, floorMat);
  floor.rotateX(-Math.PI / 2);
  floor.receiveShadow = true;
  scene.add(floor);

  // Origin Ring
  const originGeo = new THREE.RingGeometry(0.18, 0.22, 32);
  originGeo.rotateX(-Math.PI / 2);
  const originMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
  const originRing = new THREE.Mesh(originGeo, originMat);
  originRing.position.y = 0.002;
  scene.add(originRing);

  // Trajectory Motion Trail (Curved S-drive into arena)
  const pts = [];
  for (let t = 0; t <= 1; t += 0.02) {
    let px = Math.sin(t * Math.PI * 1.5) * 0.8 + t * 0.6;
    let pz = -t * 1.6;
    pts.push(new THREE.Vector3(px, 0.015, pz));
  }
  const trailGeo = new THREE.BufferGeometry().setFromPoints(pts);
  const trailMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2, transparent: true, opacity: 0.85 });
  scene.add(new THREE.Line(trailGeo, trailMat));

  // Rover entity rig
  const pivotGroup = new THREE.Group();
  const lastPt = pts[pts.length - 1];
  pivotGroup.position.copy(lastPt);
  scene.add(pivotGroup);

  const roverMesh = new THREE.Group();
  const roverHeading = -Math.PI * 0.22; // 39.6 deg heading
  roverMesh.rotation.y = roverHeading;
  pivotGroup.add(roverMesh);

  // Heading indicator arrow
  const dirArrowGeo = new THREE.ConeGeometry(0.08, 0.22, 16);
  dirArrowGeo.rotateX(Math.PI / 2);
  const dirArrowMat = new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.85 });
  const dirArrow = new THREE.Mesh(dirArrowGeo, dirArrowMat);
  dirArrow.position.set(0, 0.01, -0.28);
  roverMesh.add(dirArrow);

  // Compass Ring
  const compRingGeo = new THREE.RingGeometry(0.32, 0.35, 32);
  compRingGeo.rotateX(-Math.PI / 2);
  const compRingMat = new THREE.MeshBasicMaterial({ color: 0x0284c7, transparent: true, opacity: 0.45, side: THREE.DoubleSide });
  const compRing = new THREE.Mesh(compRingGeo, compRingMat);
  compRing.position.y = 0.008;
  roverMesh.add(compRing);

  // Ultrasonic Proximity Cone
  const usConeGeo = new THREE.ConeGeometry(0.18, 0.55, 16, 1, true);
  usConeGeo.rotateX(-Math.PI / 2);
  usConeGeo.translate(0, 0, -0.28);
  const usConeMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.28, wireframe: true });
  const usCone = new THREE.Mesh(usConeGeo, usConeMat);
  usCone.position.set(0, 0.12, -0.16);
  roverMesh.add(usCone);

  // ToF Left and Right Beams
  const tofGeo = new THREE.CylinderGeometry(0.005, 0.005, 0.45, 8);
  tofGeo.rotateX(-Math.PI / 2);
  tofGeo.translate(0, 0, -0.225);
  const tofMat = new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.8 });
  const tofL = new THREE.Mesh(tofGeo, tofMat);
  tofL.position.set(-0.12, 0.10, -0.15);
  tofL.rotation.y = Math.PI * 0.15;
  roverMesh.add(tofL);

  const tofR = new THREE.Mesh(tofGeo.clone(), tofMat);
  tofR.position.set(0.12, 0.10, -0.15);
  tofR.rotation.y = -Math.PI * 0.15;
  roverMesh.add(tofR);

  // Load Model
  const loader = new THREE.GLTFLoader();
  if (typeof MeshoptDecoder !== 'undefined') loader.setMeshoptDecoder(MeshoptDecoder);

  loader.load('rover_model_optimized.glb', async (gltf) => {
    const model = gltf.scene;
    const box = new THREE.Box3().setFromObject(model);
    const center = box.getCenter(new THREE.Vector3());
    model.position.x = -center.x;
    model.position.z = -center.z;
    model.position.y = -box.min.y;
    model.rotation.y = Math.PI / 2;

    model.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    roverMesh.add(model);

    // Frame 1: Isometric Arena Orbit View
    camera.position.set(pivotGroup.position.x + 1.8, pivotGroup.position.y + 1.3, pivotGroup.position.z + 1.9);
    camera.lookAt(pivotGroup.position.x, pivotGroup.position.y + 0.12, pivotGroup.position.z);
    renderer.render(scene, camera);

    await new Promise(r => setTimeout(r, 200));
    renderer.render(scene, camera);
    await new Promise((resolve) => {
      renderer.domElement.toBlob(async (blob) => {
        await fetch('/save_frame?name=arena_orbit', { method: 'POST', body: blob });
        resolve();
      }, 'image/png');
    });

    // Frame 2: Chase Camera View (Behind Rover looking forward)
    const forwardVec = new THREE.Vector3(0, 0, -1).applyAxisAngle(new THREE.Vector3(0, 1, 0), roverHeading);
    camera.position.copy(pivotGroup.position).addScaledVector(forwardVec, -0.95).add(new THREE.Vector3(0, 0.42, 0));
    const lookTarget = pivotGroup.position.clone().addScaledVector(forwardVec, 1.2).add(new THREE.Vector3(0, 0.10, 0));
    camera.lookAt(lookTarget);
    renderer.render(scene, camera);

    await new Promise(r => setTimeout(r, 200));
    renderer.render(scene, camera);
    await new Promise((resolve) => {
      renderer.domElement.toBlob(async (blob) => {
        await fetch('/save_frame?name=chase_cam', { method: 'POST', body: blob });
        resolve();
      }, 'image/png');
    });

    await fetch('/done', { method: 'POST' });
  });
};
</script>
</body>
</html>
"""

html_path = os.path.join(BASE_DIR, "render_multi_sim.html")
with open(html_path, "w", encoding="utf-8") as f:
    f.write(html_content)

socketserver.TCPServer.allow_reuse_address = True
server = socketserver.ThreadingTCPServer(("127.0.0.1", PORT), SimMultiHandler)
server_thread = threading.Thread(target=server.serve_forever, daemon=True)
server_thread.start()

chrome_exe = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
cmd = [
    chrome_exe,
    "--headless=new",
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--window-size=960,540",
    f"http://127.0.0.1:{PORT}/render_multi_sim.html"
]

proc = subprocess.Popen(cmd)
success = done_event.wait(timeout=30)
try:
    proc.terminate()
except Exception:
    pass
server.shutdown()

f1 = os.path.join(FRAMES_DIR, "arena_orbit.png")
f2 = os.path.join(FRAMES_DIR, "chase_cam.png")
showcase_path = os.path.join(BASE_DIR, "simulator_showcase.png")

if os.path.exists(f1) and os.path.exists(f2):
    im1 = Image.open(f1)
    im2 = Image.open(f2)
    # Side-by-side composite: 1920 x 540
    comp = Image.new("RGB", (1920, 540), (6, 9, 19))
    comp.paste(im1, (0, 0))
    comp.paste(im2, (960, 0))
    comp.save(showcase_path)
    print(f"[SUCCESS] Generated simulator showcase composite: {showcase_path}")
else:
    print(f"Error: Missing frames {f1} or {f2}")

const http = require('http');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');

const server = http.createServer((req, res) => {
  const filePath = path.join(__dirname, req.url.replace(/^\//, ''));
  if (req.url === '/render.html') {
    res.writeHead(200, { 'Content-Type': 'text/html' });
    res.end(`
      <!DOCTYPE html><html><head>
      <script src="js/three.min.js"></script>
      <script src="js/meshopt_decoder.js"></script>
      <script src="js/GLTFLoader.js"></script>
      </head><body><div id="c" style="width:800px;height:600px;"></div>
      <script>
        async function run() {
          await MeshoptDecoder.ready;
          const scene = new THREE.Scene();
          scene.background = new THREE.Color(0x1a202c);
          const camera = new THREE.PerspectiveCamera(40, 800/600, 0.1, 100);
          const renderer = new THREE.WebGLRenderer({ antialias: true });
          renderer.setSize(800, 600);
          document.getElementById('c').appendChild(renderer.domElement);

          const hemi = new THREE.HemisphereLight(0xffffff, 0x444444, 2.0);
          scene.add(hemi);
          const dir = new THREE.DirectionalLight(0xffffff, 1.5);
          dir.position.set(5, 10, 7);
          scene.add(dir);

          const loader = new THREE.GLTFLoader();
          loader.setMeshoptDecoder(MeshoptDecoder);
          loader.load('rover_model_optimized.glb', (gltf) => {
            const m = gltf.scene;
            const b = new THREE.Box3().setFromObject(m);
            const center = b.getCenter(new THREE.Vector3());
            m.position.sub(center);
            scene.add(m);

            // Add colored markers:
            // Red sphere at +X (length end 1)
            const redSphere = new THREE.Mesh(new THREE.SphereGeometry(0.08), new THREE.MeshBasicMaterial({color: 0xff0000}));
            redSphere.position.set(0.5, 0.1, 0);
            scene.add(redSphere);

            // Blue sphere at -X (length end 2)
            const blueSphere = new THREE.Mesh(new THREE.SphereGeometry(0.08), new THREE.MeshBasicMaterial({color: 0x0000ff}));
            blueSphere.position.set(-0.5, 0.1, 0);
            scene.add(blueSphere);

            // Camera looking from +Z (side view)
            camera.position.set(0, 1.0, 2.2);
            camera.lookAt(0, 0, 0);
            renderer.render(scene, camera);
            window.__READY__ = true;
          });
        }
        run();
      </script></body></html>
    `);
    return;
  }
  if (fs.existsSync(filePath)) {
    res.writeHead(200);
    fs.createReadStream(filePath).pipe(res);
  } else {
    res.writeHead(404);
    res.end();
  }
});

server.listen(9876, async () => {
  const chrome = spawn('C:/Program Files/Google/Chrome/Application/chrome.exe', [
    '--headless=new',
    '--remote-debugging-port=9227',
    'http://localhost:9876/render.html'
  ]);

  await new Promise(r => setTimeout(r, 2000));
  try {
    const res = await fetch('http://localhost:9227/json');
    const tabs = await res.json();
    const ws = new WebSocket(tabs[0].webSocketDebuggerUrl);
    ws.addEventListener('open', () => {
      setTimeout(() => {
        ws.send(JSON.stringify({ id: 1, method: 'Page.captureScreenshot' }));
      }, 1500);
    });
    ws.addEventListener('message', (evt) => {
      const msg = JSON.parse(evt.data);
      if (msg.id === 1 && msg.result?.data) {
        fs.writeFileSync('tools/side_view_markers.png', Buffer.from(msg.result.data, 'base64'));
        console.log('Saved tools/side_view_markers.png');
        ws.close();
        chrome.kill();
        server.close();
        process.exit(0);
      }
    });
  } catch (e) {
    console.error(e);
    chrome.kill();
    server.close();
    process.exit(1);
  }
});

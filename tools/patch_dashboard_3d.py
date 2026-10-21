import paramiko

host = '192.168.0.111'
user = 'ayush'
pw = os.environ.get('SSH_PASSWORD')
html_path = '/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html'

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(host, username=user, password=pw, timeout=10)

sftp = c.open_sftp()
with sftp.open(html_path, 'r') as f:
    html = f.read().decode('utf-8')

# 1. Add script tags in <head>
head_scripts = """
    <!-- Three.js + Local GLTFLoader + MeshoptDecoder -->
    <script src="/static/js/three.min.js"></script>
    <script src="/static/js/meshopt_decoder.js"></script>
    <script src="/static/js/GLTFLoader.js"></script>
    <script src="/static/js/OrbitControls.js"></script>
</head>"""
html = html.replace('</head>', head_scripts)

# 2. Add 3D Rover Card right after Card 3 (before Card 4)
card_3d = """
        <!-- Card 3D: Live 3D Rover Digital Twin -->
        <div class="card" style="grid-column: 1 / -1;">
            <div class="card-title" style="display: flex; justify-content: space-between; align-items: center;">
                <span>Rover 3D Digital Twin (PBR Textures)</span>
                <span id="model-status" style="font-size: 0.75rem; color: #10b981; font-weight: normal;">Initializing 3D Engine...</span>
            </div>
            <div id="rover-3d-container" style="width: 100%; height: 380px; border-radius: 8px; overflow: hidden; background: #070a13; position: relative;">
                <div style="position: absolute; bottom: 8px; left: 12px; font-size: 0.72rem; color: #64748b; font-family: monospace; pointer-events: none; z-index: 5;">
                    Orbit: Left-Click Drag · Pan: Right-Click Drag · Zoom: Scroll
                </div>
                <div id="telemetry-hud-3d" style="position: absolute; top: 10px; right: 12px; font-size: 0.75rem; color: #38bdf8; font-family: monospace; z-index: 5; background: rgba(15,23,42,0.8); padding: 4px 10px; border-radius: 4px; border: 1px solid #1e293b;">
                    Orientation: <span id="hud-yaw">0.0°</span>
                </div>
            </div>
        </div>

        <!-- Card 4: Real-time Live Charts -->"""
html = html.replace('<!-- Card 4: Real-time Live Charts -->', card_3d)

# 3. Add 3D initialization JS code right before </script>
js_code = """
        // --- 3D Digital Twin Viewport ---
        const container3D = document.getElementById('rover-3d-container');
        let roverMesh = null;
        let hudYawEl = document.getElementById('hud-yaw');
        let statusEl = document.getElementById('model-status');

        if (container3D && typeof THREE !== 'undefined') {
            const scene = new THREE.Scene();
            scene.background = new THREE.Color(0x070a13);

            const camera = new THREE.PerspectiveCamera(40, container3D.clientWidth / container3D.clientHeight, 0.1, 100);
            camera.position.set(1.4, 0.9, 1.8);

            const renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(container3D.clientWidth, container3D.clientHeight);
            renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
            renderer.shadowMap.enabled = true;
            renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            if (THREE.sRGBEncoding) renderer.outputEncoding = THREE.sRGBEncoding;
            container3D.appendChild(renderer.domElement);

            const controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.05;
            controls.maxPolarAngle = Math.PI / 2 + 0.1;

            const ambientLight = new THREE.AmbientLight(0xffffff, 1.3);
            scene.add(ambientLight);

            const dirLight = new THREE.DirectionalLight(0xffffff, 2.2);
            dirLight.position.set(3, 5, 2);
            dirLight.castShadow = true;
            scene.add(dirLight);

            const fillLight = new THREE.DirectionalLight(0x38bdf8, 0.9);
            fillLight.position.set(-3, 2, -2);
            scene.add(fillLight);

            const grid = new THREE.GridHelper(4, 20, 0x3b82f6, 0x1e293b);
            grid.position.y = -0.22;
            scene.add(grid);

            const loader = new THREE.GLTFLoader();
            if (typeof MeshoptDecoder !== 'undefined') {
                loader.setMeshoptDecoder(MeshoptDecoder);
            }

            statusEl.textContent = 'Loading 3D model & PBR textures...';
            loader.load('/static/rover_model.glb', (gltf) => {
                roverMesh = gltf.scene;
                const box = new THREE.Box3().setFromObject(roverMesh);
                const center = box.getCenter(new THREE.Vector3());
                roverMesh.position.sub(center);
                roverMesh.position.y += 0.05;
                
                roverMesh.traverse((child) => {
                    if (child.isMesh) {
                        child.castShadow = true;
                        child.receiveShadow = true;
                        if (child.material) {
                            child.material.roughness = 0.55;
                            child.material.metalness = 0.35;
                        }
                    }
                });

                scene.add(roverMesh);
                statusEl.textContent = '3D Digital Twin Active';
                statusEl.style.color = '#10b981';
            }, (xhr) => {
                if (xhr.lengthComputable) {
                    const pct = Math.round((xhr.loaded / xhr.total) * 100);
                    statusEl.textContent = `Loading ${pct}%`;
                }
            }, (err) => {
                console.error('Error loading 3D model:', err);
                statusEl.textContent = 'Model Load Error';
                statusEl.style.color = '#ef4444';
            });

            function animate3D() {
                requestAnimationFrame(animate3D);
                controls.update();
                renderer.render(scene, camera);
            }
            animate3D();

            window.addEventListener('resize', () => {
                if (!container3D || !renderer || !camera) return;
                camera.aspect = container3D.clientWidth / container3D.clientHeight;
                camera.updateProjectionMatrix();
                renderer.setSize(container3D.clientWidth, container3D.clientHeight);
            });
        }

        // Hook telemetry update into 3D rover mesh
        const origUpdateTelemetryUI = updateTelemetryUI;
        updateTelemetryUI = function(data) {
            origUpdateTelemetryUI(data);
            if (roverMesh && data.yaw !== undefined) {
                roverMesh.rotation.y = -data.yaw * Math.PI / 180;
                if (hudYawEl) hudYawEl.textContent = data.yaw.toFixed(1) + '°';
            }
        };
    </script>
"""
html = html.replace('</script>', js_code)

with sftp.open(html_path, 'w') as f:
    f.write(html.encode('utf-8'))

sftp.close()
c.close()
print("Updated index.html on host with 3D Rover Viewport successfully!")

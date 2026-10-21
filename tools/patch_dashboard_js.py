with open(r'C:\Users\dell\Desktop\rover\tools\index_host.html', 'r', encoding='utf-8') as f:
    content = f.read()

start_marker = "        // --- 3D Digital Twin Viewport ---"
end_marker = "    </script>"

start_idx = content.find(start_marker)
end_idx = content.find(end_marker, start_idx)

if start_idx == -1 or end_idx == -1:
    print("Error: Could not locate markers in index_host.html")
    exit(1)

js_simulator = """        // --- 3D Digital Twin & Arena Simulator Engine ---
        const container3D = document.getElementById('rover-3d-container');
        let roverMesh = null;
        let pivotGroup = null;
        let hudYawEl = document.getElementById('hud-yaw');
        let statusEl = document.getElementById('model-status');
        let btnCamMode = document.getElementById('btn-cam-mode');
        let btnCenterRover = document.getElementById('btn-center-rover');
        let btnClearTrail = document.getElementById('btn-clear-trail');

        let isChaseCam = false;
        let liveWheelSpeeds = [0, 0, 0, 0];

        // Trajectory trail buffer
        const MAX_TRAIL_PTS = 600;
        const trailPositions = new Float32Array(MAX_TRAIL_PTS * 3);
        let trailCount = 0;
        let trailLine = null;
        let lastTrailX = 9999, lastTrailZ = 9999;

        // Obstacle visualization meshes
        let usCone = null, usDisc = null;
        let tofLRay = null, tofLDisc = null;
        let tofRRay = null, tofRDisc = null;

        if (container3D && typeof THREE !== 'undefined') {
            const scene = new THREE.Scene();
            scene.background = new THREE.Color(0x060913);
            scene.fog = new THREE.FogExp2(0x060913, 0.045);

            const camera = new THREE.PerspectiveCamera(42, container3D.clientWidth / container3D.clientHeight, 0.08, 120);
            camera.position.set(2.2, 1.6, 2.5);

            const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
            renderer.setSize(container3D.clientWidth, container3D.clientHeight);
            renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
            renderer.shadowMap.enabled = true;
            renderer.shadowMap.type = THREE.PCFSoftShadowMap;
            if (THREE.sRGBEncoding) renderer.outputEncoding = THREE.sRGBEncoding;
            container3D.appendChild(renderer.domElement);

            const controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.06;
            controls.maxPolarAngle = Math.PI / 2 - 0.02; // Keep camera above ground arena
            controls.minDistance = 0.35;
            controls.maxDistance = 25.0;

            // --- Studio Lighting Rig for PBR Textures ---
            const ambientLight = new THREE.AmbientLight(0xffffff, 1.4);
            scene.add(ambientLight);

            const sunLight = new THREE.DirectionalLight(0xffffff, 2.6);
            sunLight.position.set(4.0, 7.0, 3.5);
            sunLight.castShadow = true;
            sunLight.shadow.mapSize.width = 1024;
            sunLight.shadow.mapSize.height = 1024;
            sunLight.shadow.camera.near = 0.5;
            sunLight.shadow.camera.far = 25;
            sunLight.shadow.camera.left = -3;
            sunLight.shadow.camera.right = 3;
            sunLight.shadow.camera.top = 3;
            sunLight.shadow.camera.bottom = -3;
            sunLight.shadow.bias = -0.0005;
            scene.add(sunLight);

            const skyFillLight = new THREE.DirectionalLight(0x38bdf8, 1.1);
            skyFillLight.position.set(-4.0, 3.0, -3.0);
            scene.add(skyFillLight);

            const rimLight = new THREE.DirectionalLight(0x10b981, 0.8);
            rimLight.position.set(0, -2.0, -4.0);
            scene.add(rimLight);

            // --- Arena Ground & Coordinate Grid ---
            const gridHelper = new THREE.GridHelper(16, 80, 0x0284c7, 0x1e293b);
            gridHelper.position.y = 0.0;
            scene.add(gridHelper);

            // Arena Origin Target Ring (0,0)
            const originRingGeo = new THREE.RingGeometry(0.18, 0.22, 32);
            originRingGeo.rotateX(-Math.PI / 2);
            const originRingMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
            const originRing = new THREE.Mesh(originRingGeo, originRingMat);
            originRing.position.y = 0.002;
            scene.add(originRing);

            // Ground Floor Plane (Shadow Receiver)
            const floorGeo = new THREE.PlaneGeometry(24, 24);
            const floorMat = new THREE.MeshStandardMaterial({
                color: 0x060913,
                roughness: 0.9,
                metalness: 0.1,
                depthWrite: true
            });
            const floorMesh = new THREE.Mesh(floorGeo, floorMat);
            floorMesh.rotateX(-Math.PI / 2);
            floorMesh.position.y = -0.002;
            floorMesh.receiveShadow = true;
            scene.add(floorMesh);

            // --- Trajectory Motion Trail ---
            const trailGeo = new THREE.BufferGeometry();
            trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
            trailGeo.setDrawRange(0, 0);
            const trailMat = new THREE.LineBasicMaterial({
                color: 0x38bdf8,
                linewidth: 2,
                transparent: true,
                opacity: 0.85
            });
            trailLine = new THREE.Line(trailGeo, trailMat);
            scene.add(trailLine);

            function addTrailPoint(x, z) {
                if (Math.hypot(x - lastTrailX, z - lastTrailZ) < 0.03) return; // 3cm threshold
                lastTrailX = x;
                lastTrailZ = z;

                if (trailCount < MAX_TRAIL_PTS) {
                    trailPositions[trailCount * 3 + 0] = x;
                    trailPositions[trailCount * 3 + 1] = 0.015;
                    trailPositions[trailCount * 3 + 2] = z;
                    trailCount++;
                } else {
                    // Shift buffer left
                    trailPositions.copyWithin(0, 3, MAX_TRAIL_PTS * 3);
                    trailPositions[(MAX_TRAIL_PTS - 1) * 3 + 0] = x;
                    trailPositions[(MAX_TRAIL_PTS - 1) * 3 + 1] = 0.015;
                    trailPositions[(MAX_TRAIL_PTS - 1) * 3 + 2] = z;
                }
                trailGeo.attributes.position.needsUpdate = true;
                trailGeo.setDrawRange(0, trailCount);
            }

            // --- Rover World Entity Rig ---
            pivotGroup = new THREE.Group();
            scene.add(pivotGroup);

            // Heading & Rotation Rig inside Pivot Group
            roverMesh = new THREE.Group();
            pivotGroup.add(roverMesh);

            // Ground Heading Projection Indicator
            const dirArrowGeo = new THREE.ConeGeometry(0.08, 0.22, 16);
            dirArrowGeo.rotateX(Math.PI / 2);
            const dirArrowMat = new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.75 });
            const dirArrow = new THREE.Mesh(dirArrowGeo, dirArrowMat);
            dirArrow.position.set(0, 0.01, -0.28); // points towards -Z (North)
            roverMesh.add(dirArrow);

            // Compass Dial Ring under Rover
            const compassRingGeo = new THREE.RingGeometry(0.32, 0.35, 32);
            compassRingGeo.rotateX(-Math.PI / 2);
            const compassRingMat = new THREE.MeshBasicMaterial({ color: 0x0284c7, transparent: true, opacity: 0.35, side: THREE.DoubleSide });
            const compassRing = new THREE.Mesh(compassRingGeo, compassRingMat);
            compassRing.position.y = 0.008;
            roverMesh.add(compassRing);

            // --- Obstacle Visualization Sensors ---
            // 1. Ultrasonic Sensor Cone (Front Center)
            const usConeGeo = new THREE.ConeGeometry(0.18, 0.6, 16, 1, true);
            usConeGeo.rotateX(-Math.PI / 2);
            usConeGeo.translate(0, 0, -0.3);
            const usConeMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.22, wireframe: true });
            usCone = new THREE.Mesh(usConeGeo, usConeMat);
            usCone.position.set(0, 0.12, -0.16);
            roverMesh.add(usCone);

            const usDiscGeo = new THREE.RingGeometry(0.02, 0.12, 16);
            const usDiscMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.65, side: THREE.DoubleSide });
            usDisc = new THREE.Mesh(usDiscGeo, usDiscMat);
            usDisc.position.set(0, 0.12, -0.6);
            roverMesh.add(usDisc);

            // 2. Dual ToF Beams (Left & Right, angled at ±33°, 10 cm from centre)
            const tofGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.5, 8);
            tofGeo.rotateX(-Math.PI / 2);
            tofGeo.translate(0, 0, -0.25);
            const tofMatL = new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.6 });
            tofLRay = new THREE.Mesh(tofGeo, tofMatL);
            tofLRay.position.set(-0.0545, 0.10, -0.0839);
            tofLRay.rotation.y = 0.57596; // +33 deg left
            roverMesh.add(tofLRay);

            const tofMatR = new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.6 });
            tofRRay = new THREE.Mesh(tofGeo.clone(), tofMatR);
            tofRRay.position.set(0.0545, 0.10, -0.0839);
            tofRRay.rotation.y = -0.57596; // -33 deg right
            roverMesh.add(tofRRay);

            // --- Load User's Authentic PBR 3D Rover Model ---
            const loader = new THREE.GLTFLoader();
            if (typeof MeshoptDecoder !== 'undefined') {
                loader.setMeshoptDecoder(MeshoptDecoder);
            }

            statusEl.textContent = 'Loading 3D Rover Twin...';
            let authenticRoverModel = null;

            loader.load('/static/rover_model.glb', (gltf) => {
                authenticRoverModel = gltf.scene;

                // Center bounding box accurately
                const box = new THREE.Box3().setFromObject(authenticRoverModel);
                const center = box.getCenter(new THREE.Vector3());
                const size = box.getSize(new THREE.Vector3());

                authenticRoverModel.position.x = -center.x;
                authenticRoverModel.position.z = -center.z;
                // Place bottom of wheels flush with ground plane Y = 0
                authenticRoverModel.position.y = -box.min.y;

                // Native model front is -X. Rotate 90 deg clockwise to align front with -Z (North):
                authenticRoverModel.rotation.y = Math.PI / 2;

                authenticRoverModel.traverse((child) => {
                    if (child.isMesh) {
                        child.castShadow = true;
                        child.receiveShadow = true;
                        if (child.material) {
                            child.material.roughness = Math.max(child.material.roughness || 0.5, 0.4);
                            child.material.metalness = Math.min(child.material.metalness || 0.3, 0.6);
                        }
                    }
                });

                roverMesh.add(authenticRoverModel);
                statusEl.textContent = '3D Digital Twin Active (Real PBR Rover)';
                statusEl.style.color = '#10b981';
            }, (xhr) => {
                if (xhr.lengthComputable) {
                    const pct = Math.round((xhr.loaded / xhr.total) * 100);
                    statusEl.textContent = `Loading ${pct}%`;
                }
            }, (err) => {
                console.error('GLTF loading error:', err);
                statusEl.textContent = 'Model Fallback Active';
                statusEl.style.color = '#f59e0b';
            });

            // Camera Mode Button Toggle
            if (btnCamMode) {
                btnCamMode.addEventListener('click', () => {
                    isChaseCam = !isChaseCam;
                    btnCamMode.textContent = isChaseCam ? '🎥 Chase Cam: ON' : '🎥 Chase Cam: OFF';
                    btnCamMode.style.color = isChaseCam ? '#10b981' : '#38bdf8';
                    btnCamMode.style.borderColor = isChaseCam ? '#10b981' : '#334155';
                });
            }

            if (btnCenterRover) {
                btnCenterRover.addEventListener('click', () => {
                    controls.target.copy(pivotGroup.position);
                    camera.position.set(pivotGroup.position.x + 1.8, pivotGroup.position.y + 1.4, pivotGroup.position.z + 2.0);
                });
            }

            if (btnClearTrail) {
                btnClearTrail.addEventListener('click', () => {
                    trailCount = 0;
                    trailGeo.setDrawRange(0, 0);
                    trailGeo.attributes.position.needsUpdate = true;
                });
            }

            // Disengage Chase Cam if user manually interacts with orbit controls
            controls.addEventListener('start', () => {
                if (isChaseCam) {
                    isChaseCam = false;
                    if (btnCamMode) {
                        btnCamMode.textContent = '🎥 Chase Cam: OFF';
                        btnCamMode.style.color = '#38bdf8';
                        btnCamMode.style.borderColor = '#334155';
                    }
                }
            });

            // Render & Simulation Animation Loop
            let lastAnimTime = performance.now();
            function animate3D() {
                requestAnimationFrame(animate3D);
                const now = performance.now();
                const dt = Math.min((now - lastAnimTime) / 1000.0, 0.1);
                lastAnimTime = now;

                // Chase Camera Follow Logic
                if (isChaseCam && pivotGroup && roverMesh) {
                    const roverPos = pivotGroup.position;
                    // Compute behind vector based on current rover yaw
                    const yawAngle = roverMesh.rotation.y;
                    const chaseDist = 1.6;
                    const chaseHeight = 0.85;

                    const desiredCamPos = new THREE.Vector3(
                        roverPos.x + Math.sin(yawAngle) * chaseDist,
                        roverPos.y + chaseHeight,
                        roverPos.z + Math.cos(yawAngle) * chaseDist
                    );

                    camera.position.lerp(desiredCamPos, 0.08);
                    controls.target.lerp(new THREE.Vector3(roverPos.x, roverPos.y + 0.15, roverPos.z), 0.1);
                }

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

        // --- Hook Telemetry Update into 3D Rover Digital Twin & Arena Simulator ---
        const origUpdateTelemetryUI = updateTelemetryUI;
        const hudPosEl = document.getElementById('hud-pos');
        const valPosEl = document.getElementById('val-pos');
        const valRangeEl = document.getElementById('val-range');
        const statusBadgeEl = document.getElementById('status-badge');

        updateTelemetryUI = function(data) {
            origUpdateTelemetryUI(data);

            const posX = Number(data.pos_x !== undefined ? data.pos_x : (data.pos_x_cm ? data.pos_x_cm / 100.0 : 0.0));
            const posY = Number(data.pos_y !== undefined ? data.pos_y : (data.pos_y_cm ? data.pos_y_cm / 100.0 : 0.0));

            // Update DOM telemetry indicators
            if (valPosEl) valPosEl.innerText = `X:${posX.toFixed(2)} Y:${posY.toFixed(2)}m`;
            if (hudPosEl) hudPosEl.innerText = `X: ${posX.toFixed(2)}m Y: ${posY.toFixed(2)}m`;

            // Update online link badge
            if (data.link_type && statusBadgeEl) {
                statusBadgeEl.className = "badge online";
                statusBadgeEl.innerText = `ONLINE (${data.link_type})`;
            }

            // Update range sensors in header
            const usMm = Number(data.ultra_mm || 0);
            const tofLMm = Number(data.tof_l_mm || (data.tof ? data.tof[0] : 0));
            const tofRMm = Number(data.tof_r_mm || (data.tof ? data.tof[1] : 0));

            if (valRangeEl) {
                valRangeEl.innerText = `US:${Math.round(usMm / 10)}cm ToF:${Math.round(tofLMm / 10)}cm`;
            }

            // 1. Live 3D World Position Translation (Odometry)
            if (pivotGroup) {
                // In Three.js: +X is East, +Z is South (-Y in ROS REP-103)
                pivotGroup.position.x = posX;
                pivotGroup.position.z = -posY;

                // Record dynamic trajectory trail
                addTrailPoint(posX, -posY);
            }

            // 2. Live 3D Orientation & Attitude Rotation
            if (roverMesh) {
                let yDeg = 0.0, pDeg = 0.0, rDeg = 0.0;
                if (data.pitch_deg !== undefined) pDeg = Number(data.pitch_deg);
                else if (data.p !== undefined) pDeg = Number(data.p);

                if (data.roll_deg !== undefined) rDeg = Number(data.roll_deg);
                else if (data.r !== undefined) rDeg = Number(data.r);

                if (data.yaw_deg !== undefined) yDeg = Number(data.yaw_deg);
                else if (data.hd !== undefined) yDeg = Number(data.hd);
                else if (data.yaw !== undefined) yDeg = Number(data.yaw) * 180 / Math.PI;

                const q = data.q || data.quaternion;
                if (q && Array.isArray(q) && q.length >= 4) {
                    // Strict Onboard IMU 9-DoF Madgwick AHRS Sensor Fusion Projection
                    // ROS REP-103 body frame to Three.js world frame:
                    // (qw, qx, qy, qz) -> (qx, -qz, -qy, qw)
                    roverMesh.quaternion.set(Number(q[1]), -Number(q[3]), -Number(q[2]), Number(q[0])).normalize();
                } else {
                    // Mathematical Euler Frame Transformation fallback:
                    // Clockwise heading rotation around vertical Y axis, pitch around lateral X, roll around Z
                    roverMesh.rotation.order = 'YXZ';
                    roverMesh.rotation.y = -yDeg * Math.PI / 180;
                    roverMesh.rotation.x = pDeg * Math.PI / 180;
                    roverMesh.rotation.z = -rDeg * Math.PI / 180;
                }

                // Project Live Rover IMU Coordinates onto HUD
                if (hudYawEl) {
                    const dispPitch = -pDeg;
                    let hudStr = `Y:${yDeg.toFixed(1)}° P:${dispPitch >= 0 ? '+' : ''}${dispPitch.toFixed(1)}° R:${rDeg >= 0 ? '+' : ''}${rDeg.toFixed(1)}°`;
                    const a = data.a || data.accel;
                    const g = data.g || data.gyro;
                    if (a && a.length >= 3) {
                        hudStr += ` | A:[${Number(a[0]).toFixed(1)},${Number(a[1]).toFixed(1)},${Number(a[2]).toFixed(1)}]`;
                    }
                    if (g && g.length >= 3) {
                        hudStr += ` | G:[${Number(g[0]).toFixed(2)},${Number(g[1]).toFixed(2)},${Number(g[2]).toFixed(2)}]`;
                    }
                    hudYawEl.textContent = hudStr;
                }
            }

            // 3. Dynamic Obstacle Sensor Visualization Cones
            if (usCone && usDisc) {
                const distM = Math.max(0.06, Math.min(3.0, (usMm > 0 ? usMm : 2500) / 1000.0));
                usCone.scale.set(distM, distM, distM);
                usDisc.position.z = -0.16 - distM * 0.6;
                const isWarn = (usMm > 0 && usMm < 350);
                const isCrit = (usMm > 0 && usMm < 180);
                const col = isCrit ? 0xef4444 : (isWarn ? 0xf59e0b : 0x38bdf8);
                usCone.material.color.setHex(col);
                usDisc.material.color.setHex(col);
            }

            if (tofLRay) {
                const distL = Math.max(0.05, Math.min(1.5, (tofLMm > 0 ? tofLMm : 1200) / 1000.0));
                tofLRay.scale.set(1, 1, distL / 0.5);
                const isWarn = (tofLMm > 0 && tofLMm < 250);
                tofLRay.material.color.setHex(isWarn ? 0xef4444 : 0x10b981);
            }

            if (tofRRay) {
                const distR = Math.max(0.05, Math.min(1.5, (tofRMm > 0 ? tofRMm : 1200) / 1000.0));
                tofRRay.scale.set(1, 1, distR / 0.5);
                const isWarn = (tofRMm > 0 && tofRMm < 250);
                tofRRay.material.color.setHex(isWarn ? 0xef4444 : 0x10b981);
            }
        };
"""

content = content[:start_idx] + js_simulator + content[end_idx:]

with open(r'C:\Users\dell\Desktop\rover\tools\index_host.html', 'w', encoding='utf-8') as f:
    f.write(content)

print("Successfully patched index_host.html with 3D Rover Twin & Arena Simulator Engine!")

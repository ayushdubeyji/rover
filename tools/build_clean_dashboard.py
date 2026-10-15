import paramiko
import re

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=10)

sftp = c.open_sftp()
html_path = '/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html'

with sftp.open(html_path, 'r') as f:
    orig = f.read().decode('utf-8')

# 1. Clean out the duplicate scripts from <head>
# Find </style> and </head>
style_end = orig.find('</style>')
head_end = orig.find('</head>')

clean_head_scripts = """</style>
    <!-- Three.js + Local GLTFLoader + MeshoptDecoder -->
    <script src="/static/js/three.min.js"></script>
    <script src="/static/js/meshopt_decoder.js"></script>
    <script src="/static/js/GLTFLoader.js"></script>
    <script src="/static/js/OrbitControls.js"></script>
</head>"""

orig = orig[:style_end] + clean_head_scripts + orig[head_end + 7:]

# 2. Add Dedicated Finer Controls Card after Card 1
finer_controls_card = """
        <!-- Card 1.5: Dedicated Precision & Finer Controls -->
        <div class="card">
            <div class="card-title" style="display: flex; justify-content: space-between; align-items: center;">
                <span>Precision & Finer Controls</span>
                <span id="trim-status" style="font-size: 0.72rem; color: var(--accent-green); font-family: monospace;">TRIM BALANCED</span>
            </div>
            
            <!-- Speed Limit Fine Adjustment -->
            <div class="control-row" style="margin-bottom: 10px;">
                <label style="display: flex; justify-content: space-between; font-size: 0.78rem;">
                    <span>Speed Ceiling (Fine 5% Steps):</span>
                    <span id="speed-limit-val" style="color: var(--accent-green); font-weight: bold;">60%</span>
                </label>
                <input type="range" id="speed-limit-slider" min="10" max="100" step="5" value="60" oninput="updateSpeedLimit(this.value); this.blur();">
            </div>

            <!-- Steering & Strafe Fine Trim -->
            <div style="background: rgba(15,23,42,0.6); padding: 8px 10px; border-radius: 6px; border: 1px solid var(--border); margin-bottom: 10px;">
                <div style="font-size: 0.72rem; color: var(--text-dim); margin-bottom: 6px; font-weight: 600; text-transform: uppercase;">Kinematic Trims (-20% .. +20%)</div>
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 8px;">
                    <div>
                        <label style="font-size: 0.72rem; display: flex; justify-content: space-between;">
                            <span>Fwd Trim</span><span id="trim-vx-val" style="color:#38bdf8;">0%</span>
                        </label>
                        <input type="range" id="trim-vx" min="-20" max="20" step="1" value="0" style="height:4px;" oninput="updateTrim('vx', this.value); this.blur();">
                    </div>
                    <div>
                        <label style="font-size: 0.72rem; display: flex; justify-content: space-between;">
                            <span>Strafe Trim</span><span id="trim-vy-val" style="color:#38bdf8;">0%</span>
                        </label>
                        <input type="range" id="trim-vy" min="-20" max="20" step="1" value="0" style="height:4px;" oninput="updateTrim('vy', this.value); this.blur();">
                    </div>
                </div>
                <div style="margin-top: 6px;">
                    <label style="font-size: 0.72rem; display: flex; justify-content: space-between;">
                        <span>Yaw Trim (Gyro Drift Compensation)</span><span id="trim-vz-val" style="color:#38bdf8;">0%</span>
                    </label>
                    <input type="range" id="trim-vz" min="-20" max="20" step="1" value="0" style="height:4px;" oninput="updateTrim('vz', this.value); this.blur();">
                </div>
                <div style="display: flex; justify-content: space-between; align-items: center; margin-top: 8px;">
                    <button onclick="resetTrims()" style="padding: 3px 8px; font-size: 0.7rem; background: #1e293b; color: #94a3b8; border: 1px solid #334155; border-radius: 4px; cursor: pointer;">Zero Trims</button>
                    <span style="font-size: 0.68rem; color: #64748b;">Keyboard: WASD / Q,E Rotate</span>
                </div>
            </div>

            <!-- Micro-Step Nudge Buttons -->
            <div style="display: flex; gap: 6px; align-items: center; justify-content: space-between; margin-bottom: 10px;">
                <div style="font-size: 0.72rem; color: var(--text-dim);">Micro-Docking:</div>
                <div style="display: flex; gap: 4px;">
                    <button class="mode-btn" style="padding: 4px 10px; font-size: 0.75rem;" onclick="microStep(0, 0.25, 0)"> L</button>
                    <button class="mode-btn" style="padding: 4px 10px; font-size: 0.75rem;" onclick="microStep(0.25, 0, 0)">? F</button>
                    <button class="mode-btn" style="padding: 4px 10px; font-size: 0.75rem;" onclick="microStep(-0.25, 0, 0)">? B</button>
                    <button class="mode-btn" style="padding: 4px 10px; font-size: 0.75rem;" onclick="microStep(0, -0.25, 0)">? R</button>
                </div>
            </div>

            <!-- Emergency Stop -->
            <button onclick="triggerEStop()" style="width: 100%; padding: 8px; font-weight: bold; font-size: 0.85rem; letter-spacing: 1px; color: #fff; background: linear-gradient(135deg, #ef4444, #991b1b); border: 1px solid #f87171; border-radius: 6px; cursor: pointer; text-shadow: 0 1px 2px rgba(0,0,0,0.5); box-shadow: 0 2px 8px rgba(239,68,68,0.3);">
                ? EMERGENCY STOP (SPACE)
            </button>
        </div>
"""

# Check if Card 1.5 already exists
if 'Card 1.5: Dedicated Precision & Finer Controls' not in orig:
    # Insert right before Card 2
    orig = orig.replace('<!-- Card 2: Dual Virtual Joysticks -->', finer_controls_card + '\n        <!-- Card 2: Dual Virtual Joysticks -->')

# 3. Add JS functions for fine controls, trims, keyboard teleop, and make sure all inputs blur()
js_enhancements = """
        // --- Precision & Finer Controls State ---
        let speedCeiling = 0.60;
        let trimVx = 0, trimVy = 0, trimVz = 0;

        function updateSpeedLimit(val) {
            speedCeiling = parseInt(val) / 100.0;
            document.getElementById('speed-limit-val').innerText = val + '%';
            sendControl();
        }

        function updateTrim(axis, val) {
            const v = parseInt(val) / 100.0;
            if (axis === 'vx') { trimVx = v; document.getElementById('trim-vx-val').innerText = (val > 0 ? '+' : '') + val + '%'; }
            if (axis === 'vy') { trimVy = v; document.getElementById('trim-vy-val').innerText = (val > 0 ? '+' : '') + val + '%'; }
            if (axis === 'vz') { trimVz = v; document.getElementById('trim-vz-val').innerText = (val > 0 ? '+' : '') + val + '%'; }
            
            const hasTrim = (trimVx !== 0 || trimVy !== 0 || trimVz !== 0);
            const statusEl = document.getElementById('trim-status');
            if (statusEl) {
                statusEl.innerText = hasTrim ? 'ACTIVE TRIMS' : 'TRIM BALANCED';
                statusEl.style.color = hasTrim ? 'var(--accent-amber)' : 'var(--accent-green)';
            }
            sendControl();
        }

        function resetTrims() {
            trimVx = 0; trimVy = 0; trimVz = 0;
            document.getElementById('trim-vx').value = 0;
            document.getElementById('trim-vy').value = 0;
            document.getElementById('trim-vz').value = 0;
            document.getElementById('trim-vx-val').innerText = '0%';
            document.getElementById('trim-vy-val').innerText = '0%';
            document.getElementById('trim-vz-val').innerText = '0%';
            const statusEl = document.getElementById('trim-status');
            if (statusEl) { statusEl.innerText = 'TRIM BALANCED'; statusEl.style.color = 'var(--accent-green)'; }
            sendControl();
        }

        function microStep(dx, dy, dz) {
            if (!ws || ws.readyState !== WebSocket.OPEN) return;
            const payload = {
                type: "control",
                vx: dx * speedCeiling,
                vy: dy * speedCeiling,
                vz: dz * speedCeiling,
                pan: pan,
                tilt: tilt
            };
            ws.send(JSON.stringify(payload));
            setTimeout(() => {
                if (vx === 0 && vy === 0 && vz === 0) {
                    ws.send(JSON.stringify({ type: "control", vx: 0, vy: 0, vz: 0, pan: pan, tilt: tilt }));
                }
            }, 60);
        }

        function triggerEStop() {
            vx = 0; vy = 0; vz = 0;
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({ type: "control", vx: 0, vy: 0, vz: 0, pan: pan, tilt: tilt }));
            }
            if (typeof resetKnobs === 'function') resetKnobs();
            const badge = document.getElementById('status-badge');
            if (badge) {
                badge.innerText = "E-STOP ENGAGED";
                badge.className = "badge offline";
                setTimeout(() => {
                    badge.innerText = "ONLINE (20Hz)";
                    badge.className = "badge online";
                }, 1000);
            }
        }

        // Global Keyboard Teleop (WASD + Q/E)
        window.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
            let updated = false;
            if (e.code === 'KeyW') { vx = speedCeiling; updated = true; }
            else if (e.code === 'KeyS') { vx = -speedCeiling; updated = true; }
            if (e.code === 'KeyA') { vy = speedCeiling; updated = true; }
            else if (e.code === 'KeyD') { vy = -speedCeiling; updated = true; }
            if (e.code === 'KeyQ') { vz = speedCeiling; updated = true; }
            else if (e.code === 'KeyE') { vz = -speedCeiling; updated = true; }
            if (e.code === 'Space') { triggerEStop(); e.preventDefault(); return; }
            if (updated) {
                sendControl();
                e.preventDefault();
            }
        });

        window.addEventListener('keyup', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
            let updated = false;
            if (e.code === 'KeyW' || e.code === 'KeyS') { vx = 0; updated = true; }
            if (e.code === 'KeyA' || e.code === 'KeyD') { vy = 0; updated = true; }
            if (e.code === 'KeyQ' || e.code === 'KeyE') { vz = 0; updated = true; }
            if (updated) {
                sendControl();
                e.preventDefault();
            }
        });
"""

# Hook speedCeiling and trim into sendControl
# Let's check sendControl in orig
if 'function sendControl()' in orig:
    # Replace sendControl to include trims and speedCeiling
    old_send = """function sendControl() {
            if (!ws || ws.readyState !== WebSocket.OPEN) return;
            const payload = {
                type: "control",
                vx: vx,
                vy: vy,
                vz: vz,
                pan: pan,
                tilt: tilt
            };
            ws.send(JSON.stringify(payload));
        }"""
    new_send = """function sendControl() {
            if (!ws || ws.readyState !== WebSocket.OPEN) return;
            const finalVx = (vx !== 0 ? (vx * speedCeiling + trimVx) : trimVx);
            const finalVy = (vy !== 0 ? (vy * speedCeiling + trimVy) : trimVy);
            const finalVz = (vz !== 0 ? (vz * speedCeiling + trimVz) : trimVz);
            const payload = {
                type: "control",
                vx: parseFloat(finalVx.toFixed(3)),
                vy: parseFloat(finalVy.toFixed(3)),
                vz: parseFloat(finalVz.toFixed(3)),
                pan: pan,
                tilt: tilt
            };
            ws.send(JSON.stringify(payload));
        }"""
    orig = orig.replace(old_send, new_send)

# Add js_enhancements before </script>
if '// --- Precision & Finer Controls State ---' not in orig:
    orig = orig.replace('// Auto-connect', js_enhancements + '\n        // Auto-connect')

with sftp.open(html_path, 'w') as f:
    f.write(orig.encode('utf-8'))

print("Updated index.html successfully with finer controls and clean head scripts!")
sftp.close()
c.close()

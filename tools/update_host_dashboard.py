import subprocess, base64

script = """import pathlib

srv_path = pathlib.Path('/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py')
code = srv_path.read_text()

poll_code = '''
import math
import urllib.request

async def poll_esp32_rover_loop():
    loop = asyncio.get_event_loop()
    def fetch_telem():
        try:
            with urllib.request.urlopen('http://192.168.0.101/telem', timeout=0.35) as r:
                return json.loads(r.read().decode())
        except Exception:
            return None

    while True:
        t = await loop.run_in_executor(None, fetch_telem)
        if t:
            hd_deg = float(t.get('hd', 0.0))
            if hd_deg < 0:
                hd_deg += 360.0
            p_deg = float(t.get('p', 0.0))
            r_deg = float(t.get('r', 0.0))
            latest_telemetry['yaw'] = hd_deg * math.pi / 180.0
            latest_telemetry['mag_heading'] = hd_deg * math.pi / 180.0
            latest_telemetry['yaw_deg'] = hd_deg
            latest_telemetry['pitch_deg'] = p_deg
            latest_telemetry['roll_deg'] = r_deg
            w = t.get('w', [0, 0, 0, 0])
            if len(w) >= 4:
                latest_telemetry['pwm_fl'] = int(w[0])
                latest_telemetry['pwm_fr'] = int(w[1])
                latest_telemetry['pwm_rl'] = int(w[2])
                latest_telemetry['pwm_rr'] = int(w[3])
        await asyncio.sleep(0.06)
'''

code = code.replace('@app.on_event("startup")\\n\\nimport math', 'import math')
code = code.replace('async def on_startup():', '@app.on_event("startup")\\nasync def on_startup():')
code = code.replace('@app.on_event("startup")\\n@app.on_event("startup")', '@app.on_event("startup")')
srv_path.write_text(code)
build_srv = pathlib.Path('/home/ayush/mecanum_robot/mecanum_host_ws/build/mecanum_robot/mecanum_robot/web_telemetry_server.py')
if build_srv.exists():
    build_srv.write_text(code)

for idx_str in [
    '/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html',
    '/home/ayush/mecanum_robot/mecanum_host_ws/build/mecanum_robot/web/static/index.html'
]:
    p = pathlib.Path(idx_str)
    if p.exists():
        html = p.read_text()
        old_hook = '''        // Hook telemetry update into 3D rover mesh
        const origUpdateTelemetryUI = updateTelemetryUI;
        updateTelemetryUI = function(data) {
            origUpdateTelemetryUI(data);
            if (roverMesh && data.yaw !== undefined) {
                roverMesh.rotation.y = -data.yaw * Math.PI / 180;
                if (hudYawEl) hudYawEl.textContent = data.yaw.toFixed(1) + '°';
            }
        };'''
        new_hook = '''        // Hook telemetry update into 3D rover mesh
        const origUpdateTelemetryUI = updateTelemetryUI;
        updateTelemetryUI = function(data) {
            origUpdateTelemetryUI(data);
            if (roverMesh && data.yaw !== undefined) {
                const yDeg = (data.yaw_deg !== undefined) ? data.yaw_deg : (data.yaw * 180 / Math.PI);
                const pDeg = data.pitch_deg || 0.0;
                const rDeg = data.roll_deg || 0.0;
                roverMesh.rotation.order = 'YXZ';
                roverMesh.rotation.y = -yDeg * Math.PI / 180;
                roverMesh.rotation.x = pDeg * Math.PI / 180;
                roverMesh.rotation.z = -rDeg * Math.PI / 180;
                if (hudYawEl) hudYawEl.textContent = `Y:${yDeg.toFixed(1)}° P:${pDeg.toFixed(1)}° R:${rDeg.toFixed(1)}°`;
            }
        };'''
        if old_hook in html:
            html = html.replace(old_hook, new_hook)
            p.write_text(html)
print('HOST_DASHBOARD_UPDATED_OK')
"""

b64 = base64.b64encode(script.encode()).decode()
cmd = [
    r"C:\Windows\System32\OpenSSH\ssh.exe",
    "-n",
    "-o",
    "BatchMode=yes",
    "ayush@192.168.0.111",
    f"echo {b64} | base64 -d | python3 && pkill -f web_telemetry_server.py; sleep 0.5; bash /home/ayush/mecanum_robot/launch_ros2_dashboard.sh",
]
res = subprocess.run(cmd, capture_output=True, text=True, stdin=subprocess.DEVNULL)
print(res.stdout, res.stderr)

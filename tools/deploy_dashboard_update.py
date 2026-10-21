import paramiko
import os
import time

host = '192.168.0.111'
user = 'ayush'
pw = os.environ.get('SSH_PASSWORD')
key_path = os.path.expanduser('~/.ssh/id_ed25519')

local_html = r"C:\Users\dell\Desktop\rover\index.html"
remote_html = "/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html"

local_py = r"C:\Users\dell\Desktop\rover\web_telemetry_server.py"
remote_py = "/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py"

local_mp4 = r"C:\Users\dell\Desktop\rover\rover_crawl.mp4"
remote_mp4 = "/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/rover_crawl.mp4"

local_sh = r"C:\Users\dell\Desktop\rover\run_telemetry.sh"
remote_sh = "/home/ayush/mecanum_robot/run_telemetry.sh"

print("[DEPLOY] Connecting to host", host)
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
try:
    if os.path.exists(key_path):
        c.connect(host, username=user, key_filename=key_path, timeout=10)
    else:
        c.connect(host, username=user, password=pw, timeout=10)
except Exception:
    if pw:
        c.connect(host, username=user, password=pw, timeout=10)
    else:
        raise

sftp = c.open_sftp()
print("[DEPLOY] Uploading index.html...")
sftp.put(local_html, remote_html)
print(f"[DEPLOY] index.html uploaded ({os.path.getsize(local_html)} bytes)")

if os.path.exists(local_mp4):
    print("[DEPLOY] Uploading rover_crawl.mp4...")
    sftp.put(local_mp4, remote_mp4)
    print(f"[DEPLOY] rover_crawl.mp4 uploaded ({os.path.getsize(local_mp4)} bytes)")

if os.path.exists(local_sh):
    print("[DEPLOY] Uploading run_telemetry.sh...")
    sftp.put(local_sh, remote_sh)
    print(f"[DEPLOY] run_telemetry.sh uploaded ({os.path.getsize(local_sh)} bytes)")

print("[DEPLOY] Uploading web_telemetry_server.py...")
sftp.put(local_py, remote_py)
print(f"[DEPLOY] web_telemetry_server.py uploaded ({os.path.getsize(local_py)} bytes)")
sftp.close()

c.exec_command("chmod +x /home/ayush/mecanum_robot/run_telemetry.sh")

# Restart web_telemetry_server process
print("[DEPLOY] Restarting rover-telemetry service...")
stdin, stdout, stderr = c.exec_command("sudo systemctl restart rover-telemetry.service 2>/dev/null || (pkill -f web_telemetry_server.py || true && nohup bash /home/ayush/mecanum_robot/run_telemetry.sh > /home/ayush/mecanum_robot/telemetry_server.log 2>&1 &)")
stdout.channel.recv_exit_status()
time.sleep(2)

stdin, stdout, stderr = c.exec_command("ps aux | grep web_telemetry_server.py | grep -v grep")
status = stdout.read().decode().strip()
print("[DEPLOY] Service status:\n", status)

c.close()
print("[DEPLOY] Deployment finished successfully.")

import paramiko
import os
import time

host = '192.168.0.111'
user = 'ayush'
pw = 'REMOVED'

local_html = r"C:\Users\dell\Desktop\rover\index.html"
remote_html = "/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html"

local_py = r"C:\Users\dell\Desktop\rover\web_telemetry_server.py"
remote_py = "/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py"

print("[DEPLOY] Connecting to host", host)
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(host, username=user, password=pw, timeout=10)

sftp = c.open_sftp()
print("[DEPLOY] Uploading index.html...")
sftp.put(local_html, remote_html)
print(f"[DEPLOY] index.html uploaded ({os.path.getsize(local_html)} bytes)")

print("[DEPLOY] Uploading web_telemetry_server.py...")
sftp.put(local_py, remote_py)
print(f"[DEPLOY] web_telemetry_server.py uploaded ({os.path.getsize(local_py)} bytes)")
sftp.close()

# Restart web_telemetry_server process
print("[DEPLOY] Stopping existing web_telemetry_server process...")
stdin, stdout, stderr = c.exec_command("pkill -f web_telemetry_server.py || true")
stdout.channel.recv_exit_status()
time.sleep(1)

print("[DEPLOY] Starting updated web_telemetry_server...")
stdin, stdout, stderr = c.exec_command("nohup bash /home/ayush/mecanum_robot/run_telemetry.sh > /home/ayush/mecanum_robot/telemetry_server.log 2>&1 &")
stdout.channel.recv_exit_status()
time.sleep(2)

stdin, stdout, stderr = c.exec_command("ps aux | grep web_telemetry_server.py | grep -v grep")
status = stdout.read().decode().strip()
print("[DEPLOY] Service status:\n", status)

c.close()
print("[DEPLOY] Deployment finished successfully.")

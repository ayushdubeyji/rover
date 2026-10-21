import paramiko
import os

host = '192.168.0.111'
user = 'ayush'
pw = os.environ.get('SSH_PASSWORD')
src = r"C:\Users\dell\Desktop\rover\tools\rover_model_optimized.glb"
dst = "/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/rover_model.glb"

print(f"Connecting to {host}...")
c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(host, username=user, password=pw, timeout=10)

sftp = c.open_sftp()
print(f"Uploading {src} ({os.path.getsize(src)/1024/1024:.1f} MB) to {dst}...")
sftp.put(src, dst)
sftp.close()
c.close()
print("Upload complete!")

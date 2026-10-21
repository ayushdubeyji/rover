import paramiko, pathlib

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', key_filename=os.path.expanduser('~/.ssh/id_ed25519') if os.path.exists(os.path.expanduser('~/.ssh/id_ed25519')) else None, password=os.environ.get('SSH_PASSWORD'), timeout=15)

sftp = c.open_sftp()
local_path = r'C:\Users\dell\Desktop\rover\index.html'

targets = [
    '/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html',
    '/home/ayush/mecanum_robot/mecanum_host_ws/install/mecanum_robot/share/mecanum_robot/web/static/index.html',
    '/home/ayush/mecanum_robot/mecanum_host_ws/build/mecanum_robot/web/static/index.html'
]

for t in targets:
    try:
        sftp.put(local_path, t)
        print(f"Uploaded successfully to {t}")
    except Exception as e:
        print(f"Skipping {t}: {e}")

sftp.close()
c.close()

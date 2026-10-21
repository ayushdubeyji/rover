import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', key_filename=os.path.expanduser('~/.ssh/id_ed25519') if os.path.exists(os.path.expanduser('~/.ssh/id_ed25519')) else None, password=os.environ.get('SSH_PASSWORD'), timeout=10)

cmd = "sed -n '330,390p' /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html"
stdin, stdout, stderr = c.exec_command(cmd)
print(stdout.read().decode())
c.close()

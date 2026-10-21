import paramiko
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', key_filename=os.path.expanduser('~/.ssh/id_ed25519') if os.path.exists(os.path.expanduser('~/.ssh/id_ed25519')) else None, password=os.environ.get('SSH_PASSWORD'))
_, o, _ = c.exec_command('which firefox google-chrome chromium chromium-browser; who; env | grep -E "DISPLAY|XDG"')
print(o.read().decode())
c.close()

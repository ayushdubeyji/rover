import paramiko
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED')
_, o, _ = c.exec_command('which firefox google-chrome chromium chromium-browser; who; env | grep -E "DISPLAY|XDG"')
print(o.read().decode())
c.close()

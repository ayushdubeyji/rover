import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=5)

cmd = """
w
who
ps aux | grep -E "Xorg|wayland|lxqt|lightdm|openbox" | grep -v grep
"""
stdin, stdout, stderr = c.exec_command(cmd)
print(stdout.read().decode())
c.close()

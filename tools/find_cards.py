import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=10)

cmd = "wc -l /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html; grep -n 'class=\"card\"' /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/index.html"
stdin, stdout, stderr = c.exec_command(cmd)
print(stdout.read().decode())
c.close()

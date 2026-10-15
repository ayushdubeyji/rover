import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=5)

cmd = """
ls -la /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/
grep -rn "ws://" /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/ || true
grep -rn "8080" /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/ || true
"""
stdin, stdout, stderr = c.exec_command(cmd)
print(stdout.read().decode())
c.close()

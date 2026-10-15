import paramiko
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED')
_, o, _ = c.exec_command('head -n 60 /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/mecanum_base_node.py')
print(o.read().decode())
c.close()

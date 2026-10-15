import paramiko
c = paramiko.SSHClient(); c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=15)
s = c.open_sftp(); s.put(r'C:\Users\dell\Desktop\rover\tools\install_ros2.sh', '/home/ayush/install_ros2.sh'); s.close()
c.exec_command("mkdir -p /home/ayush/logs; echo REMOVED | sudo -S -p '' nohup bash /home/ayush/install_ros2.sh > /home/ayush/logs/ros_install.log 2>&1 &")
print('started')

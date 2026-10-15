import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=5)

cmd = """
python3 -c "
with open('/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py', 'r') as f:
    content = f.read()
new_content = content.replace('port=8080,', 'port=int(os.environ.get(\\\"PORT\\\", 8090)),')
with open('/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py', 'w') as f:
    f.write(new_content)
print('Patched successfully!')
"
sed -n '198,206p' /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py
"""
stdin, stdout, stderr = c.exec_command(cmd)
print(stdout.read().decode())
c.close()

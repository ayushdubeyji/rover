import paramiko

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED')

cmd = """python3 -c "
import struct, json
data = open('/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static/rover_model.glb', 'rb').read()
magic, ver, length = struct.unpack('<3I', data[:12])
chunk_len, chunk_type = struct.unpack('<2I', data[12:20])
j = json.loads(data[20:20+chunk_len].decode())
print('Nodes:', [n.get('name') for n in j.get('nodes', [])])
print('Meshes:', [m.get('name') for m in j.get('meshes', [])])
"
"""

stdin, stdout, stderr = c.exec_command(cmd)
print(stdout.read().decode())
print(stderr.read().decode())
c.close()

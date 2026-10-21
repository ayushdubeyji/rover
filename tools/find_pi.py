import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import host_cmd as h

cmd = """python3 -c "
import socket
for i in range(1, 255):
    ip = f'192.168.0.{i}'
    for port in [22, 8081]:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.08)
        if s.connect_ex((ip, port)) == 0:
            print(f'OPEN: {ip}:{port}')
        s.close()
" """

out, err = h.run(cmd)
print("OUT:\n", out)
if err: print("ERR:\n", err)

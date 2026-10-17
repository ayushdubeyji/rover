import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import host_cmd as h

cmd = """python3 -c '
import socket
for ip in ["192.168.0.101", "192.168.0.105", "192.168.0.112", "192.168.0.113"]:
    for port in [80, 8080, 8888, 3232, 22]:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        r = s.connect_ex((ip, port))
        s.close()
        if r == 0:
            print(f"OPEN: {ip}:{port}")
print("Done scan")
'"""

out, err = h.run(cmd)
print("OUT:\n", out)
if err: print("ERR:\n", err)

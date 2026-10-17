import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import host_cmd as h

cmd = """python3 -c '
import socket
for p in [80, 8888, 3232, 8080]:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(1.0)
    res = s.connect_ex(("192.168.0.101", p))
    s.close()
    print(f"Port {p}: {res}")
'"""

out, err = h.run(cmd)
print("OUT:", out)
print("ERR:", err)

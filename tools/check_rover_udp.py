import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.bind(('0.0.0.0', 8888))
except Exception as e:
    print(f"Bind error: {e}")
    exit(1)

s.settimeout(3.0)
try:
    data, addr = s.recvfrom(2048)
    print(f"SUCCESS: received {len(data)} bytes from {addr}: {data.decode('utf-8', errors='replace')}")
except socket.timeout:
    print("TIMEOUT: no UDP packet received in 3s")
except Exception as e:
    print(f"ERROR: {e}")
finally:
    s.close()

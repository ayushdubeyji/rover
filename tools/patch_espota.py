import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import host_cmd as h

cmd = """python3 -c '
path = "/home/ayush/espota_modern.py"
with open(path, "r") as f:
    c = f.read()
if "res = connection.recv(10)" in c:
    c = c.replace("res = connection.recv(10)", "res = connection.recv(1024)")
    with open(path, "w") as f:
        f.write(c)
    print("Successfully patched recv(10) -> recv(1024)")
else:
    print("Already patched or pattern not found")
'"""

out, err = h.run(cmd)
print("OUT:\n", out)
if err: print("ERR:\n", err)

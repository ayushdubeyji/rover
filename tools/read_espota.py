import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import host_cmd as h

cmd = """python3 -c '
with open("/home/ayush/espota_modern.py") as f:
    lines = f.readlines()
for i, line in enumerate(lines[390:460], start=391):
    print(f"{i}: {line}", end="")
'"""

out, err = h.run(cmd)
print("OUT:\n", out)
if err: print("ERR:\n", err)

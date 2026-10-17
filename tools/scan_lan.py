import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import host_cmd as h

cmd = """bash -c '
for i in $(seq 1 254); do ping -c 1 -W 1 192.168.0.$i >/dev/null 2>&1 & done
wait
ip neigh show
'"""

out, err = h.run(cmd)
print("OUT:\n", out)
print("ERR:\n", err)

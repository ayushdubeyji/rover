import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import host_cmd as h

cmd = """bash -c '
ps aux | grep -v "grep" | grep -E "python|ros|telemetry|rover"
'"""

out, err = h.run(cmd)
print("OUT:\n", out)
if err: print("ERR:\n", err)

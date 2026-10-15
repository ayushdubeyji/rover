import subprocess, sys

cli = r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
fqbn = "esp32:esp32:esp32"
sketch = r"C:\Users\dell\Desktop\rover\rover esp\rover"

print("--- Compiling rover.ino ---")
cmd = [cli, "compile", "--fqbn", fqbn, "--export-binaries", "--no-color", sketch]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
for line in proc.stdout:
    sys.stdout.write(line)
proc.wait()
print(f"Compile exit code: {proc.returncode}")
sys.exit(proc.returncode)

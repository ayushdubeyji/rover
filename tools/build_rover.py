import subprocess, sys, os, shutil

cli = r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
fqbn = "esp32:esp32:esp32"
sketch = r"C:\Users\dell\Desktop\rover\rover esp\rover"
build_dir = r"C:\Users\dell\Desktop\rover\rover esp\rover\build_temp"
dest_bin = r"C:\Users\dell\Desktop\rover\rover esp\rover\build\esp32.esp32.esp32\rover.ino.bin"

os.makedirs(build_dir, exist_ok=True)
os.makedirs(os.path.dirname(dest_bin), exist_ok=True)

print("--- Compiling rover.ino ---")
cmd = [cli, "compile", "--fqbn", fqbn, "--build-path", build_dir, "--no-color", sketch]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
for line in proc.stdout:
    sys.stdout.write(line)
proc.wait()
print(f"Compile exit code: {proc.returncode}")

if proc.returncode == 0:
    src_bin = os.path.join(build_dir, "rover.ino.bin")
    if os.path.exists(src_bin):
        shutil.copy2(src_bin, dest_bin)
        print(f"Successfully copied {src_bin} -> {dest_bin} ({os.path.getsize(dest_bin)} bytes)")
    else:
        print(f"ERROR: {src_bin} not found!")
        sys.exit(1)

sys.exit(proc.returncode)

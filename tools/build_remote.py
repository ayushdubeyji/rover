import subprocess, sys

cli = r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
fqbn = "esp32:esp32:esp32s3:PSRAM=opi,FlashMode=qio,FlashSize=16M,CDCOnBoot=cdc"
sketch = r"C:\Users\dell\Desktop\rover\s3_remote"

print("--- Compiling s3_remote.ino ---")
cmd = [cli, "compile", "--fqbn", fqbn, "--export-binaries", "--no-color", sketch]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
for line in proc.stdout:
    sys.stdout.write(line)
proc.wait()
print(f"Compile exit code: {proc.returncode}")
sys.exit(proc.returncode)

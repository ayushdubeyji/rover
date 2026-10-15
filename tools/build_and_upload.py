import subprocess, sys

cli = r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
fqbn = "esp32:esp32:esp32s3:PSRAM=opi,FlashMode=qio,FlashSize=16M,CDCOnBoot=cdc"
sketch = r"C:\Users\dell\Desktop\rover\s3_remote"

print("--- Step 1: Compiling s3_remote ---")
cmd = [cli, "compile", "--fqbn", fqbn, "--no-color", sketch]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
for line in proc.stdout:
    sys.stdout.write(line)
proc.wait()
print(f"Compile exit code: {proc.returncode}")
if proc.returncode != 0:
    sys.exit(proc.returncode)

print("\n--- Step 2: Uploading s3_remote to COM14 ---")
cmd_up = [cli, "upload", "-p", "COM14", "--fqbn", fqbn, "--no-color", sketch]
proc_up = subprocess.Popen(cmd_up, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True)
for line in proc_up.stdout:
    sys.stdout.write(line)
proc_up.wait()
print(f"Upload exit code: {proc_up.returncode}")
sys.exit(proc_up.returncode)

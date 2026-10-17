import time, subprocess, sys
import serial.tools.list_ports

cli = r"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
fqbn = "esp32:esp32:esp32s3:PSRAM=opi,FlashMode=qio,FlashSize=16M,CDCOnBoot=cdc"
sketch = r"C:\Users\dell\Desktop\rover\s3_remote"

print("Waiting for ESP32-S3 remote (COM14 / Espressif USB)...")
for _ in range(30):
    ports = [p.device for p in serial.tools.list_ports.comports() if '303A' in getattr(p, 'hwid', '') or p.device == 'COM14']
    if ports:
        port = ports[0]
        print(f"Found ESP32-S3 on {port}! Uploading precompiled binary now...")
        cmd = [cli, "upload", "-p", port, "--fqbn", fqbn, "--no-color", sketch]
        res = subprocess.run(cmd)
        if res.returncode == 0:
            print(f"SUCCESS: Uploaded to {port}!")
            sys.exit(0)
        else:
            print("Upload failed, retrying...")
    time.sleep(1)

print("Timeout waiting for ESP32-S3 USB port.")
sys.exit(1)

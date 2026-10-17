import paramiko
import sys
import time
import serial

HOST = "192.168.0.111"
USER = "ayush"
PASS = "REMOVED"

bin_path = r"C:\Users\dell\Desktop\rover\rover esp\rover\build\esp32.esp32.esp32\rover.ino.bin"
remote_bin = "/home/ayush/rover.ino.bin"

ser = None
try:
    ser = serial.Serial('COM14', 115200)
    ser.dtr = True
    ser.rts = True
    print("Holding S3 Remote (COM14) in reset to ensure 100% WiFi quietness during OTA...")
    time.sleep(0.5)
except Exception as e:
    print(f"Notice: COM14 reset hold skipped: {e}")

try:
    print("Connecting to host server via SFTP...")
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PASS, timeout=10)

    sftp = ssh.open_sftp()
    print(f"Uploading {bin_path} -> {remote_bin}...")
    sftp.put(bin_path, remote_bin)
    sftp.close()
    print("Uploaded binary successfully!")

    print("Executing espota.py from host server to 192.168.0.101...")
    cmd = (
        "python3 /home/ayush/espota_modern.py "
        "-d -i 192.168.0.101 -I 192.168.0.111 -p 3232 -f /home/ayush/rover.ino.bin"
    )
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=90)
    for line in iter(stdout.readline, ""):
        print(line, end="")
    err = stderr.read().decode()
    if err:
        print("STDERR:", err)

    ssh.close()
finally:
    if ser:
        ser.dtr = False
        ser.rts = False
        ser.close()
        print("Released S3 Remote from reset.")

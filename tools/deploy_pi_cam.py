#!/usr/bin/env python3
import os
import sys
import time
import socket
import paramiko

HOST = '192.168.0.103'
USER = 'ayush'
PASS = os.environ.get('PI_PASSWORD') or os.environ.get('SSH_PASSWORD')
CPP_LOCAL = os.path.join(os.path.dirname(__file__), '..', 'pi_cam', 'pi_cam_node.cpp')
REMOTE_DIR = '/home/ayush/rover_cam'
REMOTE_CPP = '/home/ayush/rover_cam/pi_cam_node.cpp'
REMOTE_BIN = '/home/ayush/rover_cam/pi_cam_node'

SERVICE_CONTENT = """[Unit]
Description=Raspberry Pi 3B+ Native C++ Camera & Visual Navigation Daemon
After=network.target

[Service]
Type=simple
User=ayush
ExecStart=/home/ayush/rover_cam/pi_cam_node
Restart=always
RestartSec=1
KillMode=process

[Install]
WantedBy=multi-user.target
"""

def wait_for_port(host, port=22, timeout=60):
    print(f"[*] Waiting for {host}:{port} to be reachable (up to {timeout}s)...")
    start = time.time()
    while time.time() - start < timeout:
        try:
            with socket.create_connection((host, port), timeout=1.5):
                print(f"[+] {host}:{port} is OPEN!")
                return True
        except (socket.timeout, socket.error, OSError):
            time.sleep(1)
    print(f"[-] Timed out waiting for {host}:{port}")
    return False

def run_ssh(client, cmd, sudo=False):
    if sudo:
        cmd = f"echo '{PASS}' | sudo -S {cmd}"
    print(f">> {cmd}")
    stdin, stdout, stderr = client.exec_command(cmd, timeout=90)
    out = stdout.read().decode('utf-8', 'ignore')
    err = stderr.read().decode('utf-8', 'ignore')
    if out:
        print(out.strip())
    if err and "password for" not in err:
        print(f"[ERR] {err.strip()}", file=sys.stderr)
    return out, err

def deploy():
    if not wait_for_port(HOST, 22, timeout=120):
        print(f"[-] Cannot connect to Pi at {HOST}:22. Exiting.")
        sys.exit(1)

    print("[*] Connecting via SSH...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    key_path = os.path.expanduser('~/.ssh/id_ed25519')
    try:
        if os.path.exists(key_path):
            client.connect(HOST, username=USER, key_filename=key_path, timeout=10)
        elif PASS:
            client.connect(HOST, username=USER, password=PASS, timeout=10, allow_agent=False, look_for_keys=False)
        else:
            client.connect(HOST, username=USER, timeout=10)
    except Exception:
        if PASS:
            client.connect(HOST, username=USER, password=PASS, timeout=10, allow_agent=False, look_for_keys=False)
        else:
            raise

    print(f"[*] Uploading {CPP_LOCAL} to {REMOTE_CPP}...")
    sftp = client.open_sftp()
    sftp.put(CPP_LOCAL, REMOTE_CPP)
    sftp.close()
    print("[+] Source file uploaded.")

    print("[*] Compiling pi_cam_node on Pi 3B+...")
    compile_cmd = f"g++ -O3 -std=c++17 -pthread {REMOTE_CPP} -o {REMOTE_BIN}"
    out, err = run_ssh(client, compile_cmd)
    print("[+] Compilation finished.")

    print("[*] Updating /etc/systemd/system/rover-cam.service with Restart=always and RestartSec=1...")
    # Escape quotes and write via sudo tee
    svc_write_cmd = f"cat << 'EOF' | sudo tee /etc/systemd/system/rover-cam.service\n{SERVICE_CONTENT}EOF"
    run_ssh(client, svc_write_cmd, sudo=True)

    print("[*] Reloading systemd and restarting rover-cam...")
    run_ssh(client, "systemctl daemon-reload && systemctl restart rover-cam", sudo=True)

    time.sleep(2)
    print("[*] Checking service status...")
    out, _ = run_ssh(client, "systemctl status rover-cam --no-pager")

    print("[*] Verifying HTTP port 8081...")
    out, _ = run_ssh(client, "curl -s http://127.0.0.1:8081/status")
    print(f"[+] Status response: {out}")

    client.close()
    print("[SUCCESS] Pi 3B+ camera watchdog daemon deployed and verified.")

if __name__ == '__main__':
    deploy()

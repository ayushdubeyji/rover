import paramiko
import sys
import time
import serial

import argparse
import json
import urllib.request
import urllib.error

import os

HOST = "192.168.0.111"
USER = "ayush"
PASS = os.environ.get("SSH_PASSWORD")
KEY_PATH = os.path.expanduser("~/.ssh/id_ed25519")

bin_path = r"C:\Users\dell\Desktop\rover\rover esp\rover\build\esp32.esp32.esp32\rover.ino.bin"
remote_bin = "/home/ayush/rover.ino.bin"

def check_target_safety(ssh, target_ip):
    # Query mDNS for any record matching this IP
    stdin, stdout, _ = ssh.exec_command("avahi-browse -r -t _arduino._tcp 2>/dev/null", timeout=5)
    lines = stdout.read().decode('utf-8', errors='ignore').splitlines()
    curr = {}
    candidates = []
    for line in lines:
        line = line.strip()
        if line.startswith("="):
            curr = {"raw": line}
            candidates.append(curr)
        elif curr:
            if "hostname =" in line:
                curr["hostname"] = line.split("=")[-1].strip(" []")
            elif "address =" in line:
                curr["address"] = line.split("=")[-1].strip(" []")
            elif "txt =" in line:
                curr["txt"] = line.split("=", 1)[-1].strip()

    for c in candidates:
        if c.get("address") == target_ip:
            h = c.get("hostname", "").lower()
            t = c.get("txt", "").lower()
            if any(k in h or k in t for k in ["c3", "lolin_c3", "riscv"]):
                print(f"[FATAL] Safety Guard: {target_ip} is an ESP32-C3 / non-Xtensa device ({c.get('hostname')}, {c.get('txt')})!")
                print("[FATAL] Refusing to flash. Aborting to protect non-Xtensa hardware.")
                return False

    # Also check if target responds to HTTP /status but doesn't match rover
    check_status_cmd = f"""python3 -c "
import urllib.request, json
try:
    req = urllib.request.Request('http://{target_ip}/status', headers={{'User-Agent': 'safety'}})
    with urllib.request.urlopen(req, timeout=1.0) as r:
        d = json.loads(r.read().decode('utf-8'))
        if 'service' in d and d['service'] != 'rover_core':
            print('NON_ROVER_SERVICE')
except:
    pass
" """
    _, stdout, _ = ssh.exec_command(check_status_cmd, timeout=5)
    out = stdout.read().decode('utf-8', errors='ignore').strip()
    if "NON_ROVER_SERVICE" in out:
        print(f"[FATAL] Safety Guard: {target_ip} returned a non-rover HTTP service. Refusing to flash.")
        return False

    return True

def discover_rover(ssh):
    print("[DISCOVERY] Scanning LAN for Xtensa Rover ESP32...")
    # 1. Check mDNS via avahi-browse on host
    stdin, stdout, stderr = ssh.exec_command("avahi-browse -r -t _arduino._tcp 2>/dev/null", timeout=10)
    lines = stdout.read().decode('utf-8', errors='ignore').splitlines()
    candidates = []
    curr = {}
    for line in lines:
        line = line.strip()
        if line.startswith("="):
            curr = {"raw": line}
            candidates.append(curr)
        elif curr:
            if "hostname =" in line:
                curr["hostname"] = line.split("=")[-1].strip(" []")
            elif "address =" in line:
                curr["address"] = line.split("=")[-1].strip(" []")
            elif "port =" in line:
                curr["port"] = line.split("=")[-1].strip(" []")
            elif "txt =" in line:
                curr["txt"] = line.split("=", 1)[-1].strip()

    for c in candidates:
        ip = c.get("address")
        host = c.get("hostname", "")
        txt = c.get("txt", "")
        print(f"[DISCOVERY] Found mDNS candidate: {ip} ({host}) txt={txt}")
        # Reject ESP32-C3 / RISC-V devices
        if any(k in host.lower() or k in txt.lower() for k in ["c3", "lolin_c3", "riscv"]):
            print(f"[DISCOVERY] Skipping {ip}: detected ESP32-C3 / non-Xtensa board ({txt})")
            continue
        if "rover" in host.lower() or "mecanum" in host.lower():
            print(f"[DISCOVERY] Validated rover by mDNS: {ip} ({host})")
            return ip

    # 2. Dynamic ARP / Neighbor query on host to find all active LAN IPs
    stdin, stdout, _ = ssh.exec_command("ip neigh show | grep -v FAILED | awk '{print $1}'", timeout=5)
    active_ips = [l.strip() for l in stdout.read().decode('utf-8', errors='ignore').splitlines() if l.strip()]
    priority_ips = ["192.168.0.101"] + [f"192.168.0.{i}" for i in range(100, 125)]
    all_candidates = list(dict.fromkeys(active_ips + priority_ips))

    # 3. Probe candidate IPs via HTTP and port 3232 on host
    probe_script = f"""python3 -c "
import urllib.request, json, socket
candidates = {all_candidates!r}
for ip in candidates:
    p3232 = False
    p80 = False
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)
        p3232 = (s.connect_ex((ip, 3232)) == 0)
        s.close()
    except:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)
        p80 = (s.connect_ex((ip, 80)) == 0)
        s.close()
    except:
        pass

    if not (p3232 or p80):
        continue

    # Check /health
    try:
        req = urllib.request.Request(f'http://{{ip}}/health', headers={{'User-Agent': 'ota_disc'}})
        with urllib.request.urlopen(req, timeout=0.5) as r:
            data = json.loads(r.read().decode('utf-8'))
            if data.get('service') == 'rover_core':
                print(f'FOUND:{{ip}}:health')
                break
    except:
        pass
    # Check /status
    try:
        req = urllib.request.Request(f'http://{{ip}}/status', headers={{'User-Agent': 'ota_disc'}})
        with urllib.request.urlopen(req, timeout=0.5) as r:
            data = json.loads(r.read().decode('utf-8'))
            if 'link' in data and 'w' in data and 'mode' in data and 'hd' in data:
                print(f'FOUND:{{ip}}:status')
                break
    except:
        pass
" """
    stdin, stdout, stderr = ssh.exec_command(probe_script, timeout=15)
    for line in stdout.read().decode('utf-8', errors='ignore').splitlines():
        if line.startswith("FOUND:"):
            parts = line.split(":")
            found_ip = parts[1]
            print(f"[DISCOVERY] Validated rover by HTTP {parts[2]}: {found_ip}")
            return found_ip

    return None

def verify_rover_status(ssh, target_ip):
    print(f"[VERIFY] Querying http://{target_ip}/status after flash...")
    cmd = f"""python3 -c "
import urllib.request, json, time
for attempt in range(15):
    try:
        time.sleep(1.0)
        req = urllib.request.Request('http://{target_ip}/status', headers={{'User-Agent': 'verify'}})
        with urllib.request.urlopen(req, timeout=1.5) as r:
            data = json.loads(r.read().decode('utf-8'))
            print('VERIFY_OK:', json.dumps(data))
            break
    except Exception as e:
        print(f'Wait (attempt {{attempt+1}}): {{e}}')
" """
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=30)
    out = stdout.read().decode('utf-8', errors='ignore')
    print(out)
    for line in out.splitlines():
        if line.startswith("VERIFY_OK:"):
            data = json.loads(line[10:])
            wheels = data.get("w", [999, 999, 999, 999])
            idle_zero = all(abs(v) == 0 for v in wheels)
            print(f"[VERIFY] Wheels: {wheels} -> Idle zero output: {idle_zero}")
            return True, data
    return False, None

parser = argparse.ArgumentParser(description="Flash Rover ESP32 firmware via host server espota")
parser.add_argument("target_ip", nargs="?", default=None, help="Target Rover IP (auto-discovered if omitted)")
parser.add_argument("--ip", dest="ip_arg", default=None, help="Explicit Rover IP")
parser.add_argument("--no-verify", action="store_true", help="Skip post-flash /status verification")
args = parser.parse_args()

target_ip = args.ip_arg or args.target_ip

ser = None
try:
    ser = serial.Serial('COM14', 115200)
    ser.dtr = True
    ser.rts = True
    print("Holding S3 Remote (COM14) in reset to ensure 100% WiFi quietness during OTA...")
    time.sleep(0.5)
except Exception as e:
    print(f"Notice: COM14 reset hold skipped: {e}")

ssh = None
telemetry_paused = False
try:
    print("Connecting to host server via SFTP...")
    ssh = paramiko.SSHClient()
    try:
        if os.path.exists(KEY_PATH):
            ssh.connect(HOST, username=USER, key_filename=KEY_PATH, timeout=10)
        else:
            ssh.connect(HOST, username=USER, password=PASS, timeout=10)
    except Exception:
        if PASS:
            ssh.connect(HOST, username=USER, password=PASS, timeout=10)
        else:
            raise

    if not target_ip:
        target_ip = discover_rover(ssh)
        if not target_ip:
            print("[ERROR] Auto-discovery could not locate an online Xtensa Rover ESP32!")
            print("[ERROR] Refusing to flash. Check rover power, WiFi association, or specify IP manually with --ip.")
            sys.exit(1)

    # Universal Safety Guard: Check target IP to avoid flashing ESP32-C3 or non-Xtensa hardware
    if not check_target_safety(ssh, target_ip):
        sys.exit(2)

    print(f"[TARGET] Rover IP confirmed: {target_ip}")

    sftp = ssh.open_sftp()
    print(f"Uploading {bin_path} -> {remote_bin}...")
    sftp.put(bin_path, remote_bin)
    sftp.close()
    print("Uploaded binary successfully!")

    print("Pausing web_telemetry_server during OTA to guarantee 100% WiFi quietness...")
    ssh.exec_command("pkill -f web_telemetry_server.py")
    telemetry_paused = True
    time.sleep(1)

    print(f"Executing espota.py from host server to {target_ip}...")
    cmd = (
        f"python3 /home/ayush/espota_modern.py "
        f"-d -i {target_ip} -I 192.168.0.111 -p 3232 -f /home/ayush/rover.ino.bin"
    )
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=120)
    for line in iter(stdout.readline, ""):
        print(line, end="")
    err = stderr.read().decode()
    if err:
        print("STDERR:", err)

    if not args.no_verify:
        time.sleep(2)
        ok, telem = verify_rover_status(ssh, target_ip)
        if ok:
            print(f"[SUCCESS] Rover firmware verified online at {target_ip}!")
        else:
            print(f"[WARNING] Verification pending or device still rebooting at {target_ip}.")
finally:
    if ssh:
        if telemetry_paused:
            print("Restarting web_telemetry_server on host...")
            ssh.exec_command("nohup bash /home/ayush/mecanum_robot/run_telemetry.sh > /home/ayush/mecanum_robot/telemetry_server.log 2>&1 &")
            time.sleep(1)
        ssh.close()
    if ser:
        ser.dtr = False
        ser.rts = False
        ser.close()
        print("Released S3 Remote from reset.")

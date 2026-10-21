# Host Server (192.168.0.111) Bootstrap & Recovery Runbook

This runbook documents the complete setup, architecture, and one-command recovery procedure for the host compute server at `192.168.0.111` (`ayush-i3`).

---

## 1. Host Identity & Network Specification

- **Host Name**: `ayush-i3`
- **User**: `ayush`
- **OS**: Ubuntu Linux 26.04.1 LTS (kernel `7.0.0-38-generic`, x86_64)
- **RAM**: 4 GB (3.3 GB usable, Swap 3.8 GB)
- **Disk**: 457 GB root partition (`/dev/sda2`), ~12 GB used (<4%)
- **Network Interfaces**:
  - Ethernet `enp7s0`: `192.168.0.111/24` (MAC: `74:e6:e2:34:95:b6`)
  - Wi-Fi `wlp6s0`: `192.168.0.102/24`
- **DHCP Recommendation**: Configure a static DHCP lease / IP reservation on the LAN router binding `192.168.0.111` to MAC address `74:e6:e2:34:95:b6`.

---

## 2. Security & Authentication Architecture

- **Passwordless SSH Key Authentication**:
  - Key: `ed25519` key from Windows dev machine (`~/.ssh/id_ed25519.pub`) installed in `/home/ayush/.ssh/authorized_keys` (`chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys`).
  - No passwords are ever hardcoded in repository files or scripts.
  - All deploy and inspection tools (`tools/deploy_dashboard_update.py`, `tools/flash_rover_from_host.py`, `tools/ssh.py`, `tools/host_cmd.py`, etc.) automatically authenticate using the local SSH key, falling back to the `SSH_PASSWORD` environment variable if present.
- **Passwordless Sudo**:
  - `/etc/sudoers.d/ayush`: `ayush ALL=(ALL) NOPASSWD:ALL` (permissions `0440`).

---

## 3. Directory Layout on Host

```
/home/ayush/
├── mecanum_robot/
│   ├── venv/                       # Unified Python virtualenv (FastAPI, ROS 2, PyTorch/SciPy/NumPy, Opus)
│   ├── run_telemetry.sh            # Telemetry launch script (sources ROS 2 and venv)
│   ├── telemetry_server.log        # Live stdout/stderr log
│   └── mecanum_host_ws/            # ROS 2 Colcon Workspace
│       ├── install/
│       ├── build/
│       └── src/
│           ├── mecanum_robot/      # Web dashboard & telemetry package
│           │   ├── mecanum_robot/
│           │   │   ├── web_telemetry_server.py
│           │   │   └── protocol.py
│           │   └── web/static/
│           │       ├── index.html
│           │       └── rover_crawl.mp4
│           └── ros2_rover/         # ROS 2 Mecanum Rover Stack (URDF, TF, Nav2, Nodes)
├── workbench_hub/                  # Feynman Workbench FastAPI Hub
│   ├── main.py                     # FastAPI backend
│   ├── workbench.db                # SQLite database (WAL mode)
│   ├── .env                        # Untracked API keys (permissions 600)
│   ├── personas.json
│   ├── static/
│   └── notes/
├── workbench_vault/                # Obsidian RAG Vault
│   └── Diary/
├── espota_modern.py                # Patched ESP32 OTA flash script (recv 1024)
└── espota.py
```

---

## 4. One-Command Automated Recovery

If the host server is ever reinstalled or wiped again, run the following bootstrap script from the Windows dev box:

```powershell
# 1. Clear stale host key on dev box
ssh-keygen -R 192.168.0.111

# 2. Copy SSH key to host (runtime password only, never saved)
$env:SSH_PASSWORD = "..." # passed at runtime
python tools/ssh.py "mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo '$(Get-Content ~/.ssh/id_ed25519.pub)' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
python tools/ssh.py sudo "echo 'ayush ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/ayush && chmod 0440 /etc/sudoers.d/ayush"
Remove-Item Env:\SSH_PASSWORD

# 3. Copy bootstrap script and execute
scp tools/bootstrap_host.sh ayush@192.168.0.111:/home/ayush/bootstrap_host.sh
ssh ayush@192.168.0.111 "chmod +x /home/ayush/bootstrap_host.sh && /home/ayush/bootstrap_host.sh"

# 4. Deploy Dashboard & Telemetry
python tools/deploy_dashboard_update.py
```

---

## 5. Systemd Services

1. **`rover-telemetry.service`**:
   - Auto-starts `run_telemetry.sh`
   - Port 8090 TCP (FastAPI HTTP + WebSocket telemetry)
   - Port 8888 UDP (zero-latency rover packet receiver)
   - Commands:
     ```bash
     sudo systemctl status rover-telemetry.service
     sudo systemctl restart rover-telemetry.service
     journalctl -u rover-telemetry.service -f
     ```

2. **`workbench-hub.service`**:
   - Auto-starts Uvicorn FastAPI hub on `127.0.0.1:8000` (reverse proxied by Nginx on port 80)
   - Commands:
     ```bash
     sudo systemctl status workbench-hub.service
     sudo systemctl restart workbench-hub.service
     ```

3. **`nginx.service`**:
   - Listens on port 80, forwards to `127.0.0.1:8000`, supports WebSocket upgrades for `/ws/xiaozhi`.

---

## 6. Verification Commands & Healthchecks

| Service / Layer | Verification Command | Expected Output |
|---|---|---|
| Dashboard Web | `curl -i http://192.168.0.111:8090/` | `HTTP/1.1 200 OK`, HTML content |
| Live Telemetry API | `curl -s http://192.168.0.111:8090/api/telemetry` | JSON telemetry state dictionary |
| Pi Camera Snapshot Proxy | `curl -i http://192.168.0.111:8090/api/pi_cam/snapshot` | `HTTP/1.1 200 OK`, `image/jpeg` |
| Workbench Hub & Nginx | `curl -i http://192.168.0.111/` | `HTTP/1.1 200 OK`, `Server: nginx` |
| ROS 2 Lyrical & Workspace | `cd ~/mecanum_robot/mecanum_host_ws && source /opt/ros/lyrical/setup.bash && colcon build` | `Finished <<< ros2_rover` |
| ROS 2 Unit Tests | `python3 ~/mecanum_robot/mecanum_host_ws/src/ros2_rover/test_ros2_layer.py` | `ALL 9 TESTS PASSED SUCCESSFULLY!` |
| ESP32 OTA Tooling | `python3 /home/ayush/espota_modern.py -h` | espota usage options |
| Arduino CLI & Core | `arduino-cli core list` | `esp32:esp32 3.3.12` |

---

## 7. Environment Variables (`/home/ayush/workbench_hub/.env`)

The file is untracked and protected (`chmod 600`):

```env
GROQ_API_KEY=your_groq_api_key_here
GEMINI_API_KEY=your_gemini_api_key_here
ELEVENLABS_API_KEY=your_elevenlabs_api_key_here
ELEVENLABS_VOICE_ID=x4eeaCQG3dqOPKBvawgo
```

The user must provide the real keys for Groq, Gemini, and ElevenLabs to enable voice synthesis and LLM processing.

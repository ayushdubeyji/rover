#!/usr/bin/env bash
# ==============================================================================
# Host Server Rebuild & Bootstrap Script
# Idempotent bootstrap for Ubuntu (Resolute 26.04 / Noble 24.04 / Jammy 22.04)
# Target Host: 192.168.0.111 (user: ayush)
# NO SECRETS OR PASSWORDS CONTAINED IN THIS SCRIPT
# ==============================================================================
set -euo pipefail

echo "=== [1/8] Updating package lists & installing base system packages ==="
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -y
sudo apt-get install -y \
    git python3-venv python3-pip build-essential curl openssh-server \
    avahi-daemon avahi-utils ufw libusb-1.0-0 libopus-dev nginx software-properties-common

echo "=== [2/8] Configuring UFW Firewall ==="
sudo ufw default allow outgoing
sudo ufw default deny incoming
sudo ufw allow 22/tcp
sudo ufw allow 8090/tcp
sudo ufw allow 8888
sudo ufw allow 3232/udp
sudo ufw allow 80/tcp
sudo ufw allow 8000/tcp
sudo ufw allow 5353/udp
sudo ufw --force enable
sudo systemctl enable --now avahi-daemon

echo "=== [3/8] Setting up Directory Tree ==="
mkdir -p /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot
mkdir -p /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/web/static
mkdir -p /home/ayush/workbench_hub/static
mkdir -p /home/ayush/workbench_hub/notes
mkdir -p /home/ayush/workbench_vault/Diary

echo "=== [4/8] Setting up Unified Python Venv ==="
if [ ! -d /home/ayush/mecanum_robot/venv ]; then
    python3 -m venv /home/ayush/mecanum_robot/venv
fi
/home/ayush/mecanum_robot/venv/bin/pip install --upgrade pip
/home/ayush/mecanum_robot/venv/bin/pip install \
    fastapi uvicorn websockets numpy scipy psutil requests sentry-sdk \
    groq httpx python-dotenv opuslib edge-tts python-multipart pyserial

echo "=== [5/8] Installing ROS 2 & Dependencies ==="
UBUNTU_CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME")
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt-get update -y

if [ "$UBUNTU_CODENAME" = "resolute" ]; then
    ROS_DISTRO="lyrical"
elif [ "$UBUNTU_CODENAME" = "noble" ]; then
    ROS_DISTRO="jazzy"
else
    ROS_DISTRO="humble"
fi
echo "Detected Ubuntu $UBUNTU_CODENAME -> Installing ROS 2 $ROS_DISTRO"
sudo apt-get install -y \
    "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-robot-state-publisher" \
    "ros-${ROS_DISTRO}-xacro" \
    "ros-${ROS_DISTRO}-tf2-ros" \
    python3-colcon-common-extensions python3-rosdep \
    python3-scipy python3-numpy python3-serial

echo "=== [6/8] Installing Arduino CLI & ESP32 Core ==="
if ! command -v arduino-cli &> /dev/null; then
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=/home/ayush/bin sh
    sudo cp /home/ayush/bin/arduino-cli /usr/local/bin/arduino-cli
fi
arduino-cli config init --overwrite || true
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json || true
arduino-cli core update-index
arduino-cli core install esp32:esp32

# Setup espota helper
ESPOTA_SRC=$(find /home/ayush/.arduino15 -name "espota.py" 2>/dev/null | head -n 1)
if [ -n "$ESPOTA_SRC" ]; then
    cp "$ESPOTA_SRC" /home/ayush/espota.py
    cp "$ESPOTA_SRC" /home/ayush/espota_modern.py
    python3 -c "
import pathlib
p = pathlib.Path('/home/ayush/espota_modern.py')
c = p.read_text()
p.write_text(c.replace('res = connection.recv(10)', 'res = connection.recv(1024)'))
"
    chmod +x /home/ayush/espota.py /home/ayush/espota_modern.py
fi

echo "=== [7/8] Configuring Nginx Reverse Proxy & Database ==="
sudo tee /etc/nginx/sites-available/default > /dev/null << 'EOF'
server {
    listen 80 default_server;
    listen [::]:80 default_server;
    server_name _;
    client_max_body_size 25M;

    location / {
        proxy_pass http://127.0.0.1:8000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_read_timeout 86400s;
        proxy_send_timeout 86400s;
    }
}
EOF
sudo nginx -t
sudo systemctl reload nginx

# Initialize SQLite database schema
/home/ayush/mecanum_robot/venv/bin/python3 - << 'EOF'
import sqlite3
conn = sqlite3.connect('/home/ayush/workbench_hub/workbench.db')
c = conn.cursor()
c.execute('''CREATE TABLE IF NOT EXISTS tasks (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    title TEXT NOT NULL,
    description TEXT,
    status TEXT DEFAULT 'pending',
    due_date TEXT,
    priority INTEGER DEFAULT 1,
    created_at TEXT DEFAULT CURRENT_TIMESTAMP
);''')
c.execute('''CREATE TABLE IF NOT EXISTS projects (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    description TEXT,
    target_hardware TEXT,
    status TEXT DEFAULT 'in_progress',
    progress INTEGER DEFAULT 0,
    icon TEXT DEFAULT 'memory',
    created_at TEXT DEFAULT CURRENT_TIMESTAMP
);''')
c.execute('''CREATE TABLE IF NOT EXISTS reminders (
    id INTEGER PRIMARY KEY,
    text TEXT,
    due_datetime TEXT,
    created_at TEXT
);''')
c.execute('''CREATE TABLE IF NOT EXISTS sessions (
    session_id TEXT PRIMARY KEY,
    device_id TEXT,
    language TEXT DEFAULT 'en-US',
    created_at TEXT DEFAULT CURRENT_TIMESTAMP,
    last_active TEXT
);''')
conn.commit()
print("Workbench database schema verified.")
EOF

# Create untracked .env placeholder if missing
if [ ! -f /home/ayush/workbench_hub/.env ]; then
    cat << 'EOF' > /home/ayush/workbench_hub/.env
GROQ_API_KEY=your_groq_api_key_here
GEMINI_API_KEY=your_gemini_api_key_here
ELEVENLABS_API_KEY=your_elevenlabs_api_key_here
ELEVENLABS_VOICE_ID=x4eeaCQG3dqOPKBvawgo
EOF
    chmod 600 /home/ayush/workbench_hub/.env
fi

echo "=== [8/8] Installing & Starting Systemd Services ==="
sudo tee /etc/systemd/system/rover-telemetry.service > /dev/null << 'EOF'
[Unit]
Description=Mecanum Rover Telemetry Hub and ROS 2 Bridge
After=network.target

[Service]
Type=simple
User=ayush
WorkingDirectory=/home/ayush/mecanum_robot
ExecStart=/bin/bash /home/ayush/mecanum_robot/run_telemetry.sh
Restart=always
RestartSec=3
StandardOutput=append:/home/ayush/mecanum_robot/telemetry_server.log
StandardError=append:/home/ayush/mecanum_robot/telemetry_server.log

[Install]
WantedBy=multi-user.target
EOF

sudo tee /etc/systemd/system/workbench-hub.service > /dev/null << 'EOF'
[Unit]
Description=Smart Workbench FastAPI Hub
After=network.target

[Service]
User=ayush
WorkingDirectory=/home/ayush/workbench_hub
EnvironmentFile=/home/ayush/workbench_hub/.env
ExecStart=/home/ayush/mecanum_robot/venv/bin/uvicorn main:app --host 127.0.0.1 --port 8000 --workers 2
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now rover-telemetry.service
sudo systemctl enable --now workbench-hub.service

echo "=== Bootstrap Completed Successfully! ==="

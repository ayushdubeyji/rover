import paramiko
import time
import requests

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', key_filename=os.path.expanduser('~/.ssh/id_ed25519') if os.path.exists(os.path.expanduser('~/.ssh/id_ed25519')) else None, password=os.environ.get('SSH_PASSWORD'), timeout=5)

# First kill any previous instance of web_telemetry_server
c.exec_command("pkill -f 'web_telemetry_server.py' || true")
time.sleep(1)

# Launch web_telemetry_server in background with nohup
launch_cmd = (
    "bash -c 'source /opt/ros/jazzy/setup.bash; "
    "export PORT=8090; "
    "export PYTHONPATH=/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot:$PYTHONPATH; "
    "nohup python3 /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py > /tmp/telemetry_server.log 2>&1 &'"
)
c.exec_command(launch_cmd)

# Wait 2 seconds and check if port 8090 is listening
time.sleep(2)
_, stdout, _ = c.exec_command("ss -tulpn | grep 8090; tail -n 15 /tmp/telemetry_server.log")
log_out = stdout.read().decode()
print("Server status:\n", log_out)

# Launch Firefox on display :0 of Lubuntu host
firefox_cmd = (
    "bash -c 'export DISPLAY=:0; "
    "export XAUTHORITY=/home/ayush/.Xauthority; "
    "nohup firefox http://localhost:8090 > /dev/null 2>&1 &'"
)
c.exec_command(firefox_cmd)
time.sleep(1)
_, stdout, _ = c.exec_command("ps aux | grep firefox | grep -v grep")
print("Firefox status:\n", stdout.read().decode())

c.close()

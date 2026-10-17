import paramiko
import time

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=10)

c.exec_command("pkill -9 -f 'web_telemetry_server.py'")
time.sleep(1)

cmd = (
    "bash -c 'source /opt/ros/jazzy/setup.bash; "
    "export PORT=8090; "
    "export PYTHONPATH=/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot:$PYTHONPATH; "
    "nohup python3 /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py > /tmp/telemetry_server.log 2>&1 &'"
)
c.exec_command(cmd)
time.sleep(2)

_, stdout, _ = c.exec_command("ss -tulpn | grep 8090; tail -n 10 /tmp/telemetry_server.log")
print(stdout.read().decode())
c.close()

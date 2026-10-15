import paramiko, time

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect('192.168.0.111', username='ayush', password='REMOVED', timeout=10)

script = """#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/ayush/mecanum_robot/mecanum_host_ws/install/setup.bash

ros2 launch rover_localization bringup.launch.py > /tmp/bringup_run.log 2>&1 &
LPID=$!
sleep 5

echo "=== ROS 2 ACTIVE NODES ==="
ros2 node list

echo ""
echo "=== ROS 2 ACTIVE TOPICS ==="
ros2 topic list

echo ""
echo "=== LAUNCH LOG OUTPUT ==="
head -n 25 /tmp/bringup_run.log

kill $LPID 2>/dev/null
wait $LPID 2>/dev/null || true
"""

sftp = c.open_sftp()
with sftp.file('/tmp/test_bringup.sh', 'w') as f:
    f.write(script)
sftp.close()

c.exec_command('chmod +x /tmp/test_bringup.sh')
_, o, e = c.exec_command('/tmp/test_bringup.sh', timeout=30)
print(o.read().decode())
err = e.read().decode()
if err:
    print("STDERR:", err)
c.close()

import os
import paramiko

host = '192.168.0.111'
user = 'ayush'
pw = os.environ.get('SSH_PASSWORD')
key_path = os.path.expanduser('~/.ssh/id_ed25519')

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
if os.path.exists(key_path):
    c.connect(host, username=user, key_filename=key_path, timeout=10)
else:
    c.connect(host, username=user, password=pw, timeout=10)

sftp = c.open_sftp()

files_to_sync = [
    (r"C:\Users\dell\Desktop\rover\ros2\urdf\rover.urdf.xacro",
     "/home/ayush/mecanum_robot/mecanum_host_ws/src/ros2_rover/urdf/rover.urdf.xacro"),
    (r"C:\Users\dell\Desktop\rover\ros2\launch\static_tf.launch.py",
     "/home/ayush/mecanum_robot/mecanum_host_ws/src/ros2_rover/launch/static_tf.launch.py"),
    (r"C:\Users\dell\Desktop\rover\ros2\ros2_rover\tof_laserscan_publisher.py",
     "/home/ayush/mecanum_robot/mecanum_host_ws/src/ros2_rover/ros2_rover/tof_laserscan_publisher.py"),
    (r"C:\Users\dell\Desktop\rover\ros2\ros2_rover\doa_srp_phat_node.py",
     "/home/ayush/mecanum_robot/mecanum_host_ws/src/ros2_rover/ros2_rover/doa_srp_phat_node.py"),
    (r"C:\Users\dell\Desktop\rover\ros2\test_ros2_layer.py",
     "/home/ayush/mecanum_robot/mecanum_host_ws/src/ros2_rover/test_ros2_layer.py"),
    (r"C:\Users\dell\Desktop\rover\tests\test_rover_modules.py",
     "/home/ayush/mecanum_robot/test_rover_modules.py"),
]

for local_path, remote_path in files_to_sync:
    print(f"[SYNC] Uploading {os.path.basename(local_path)} -> {remote_path}")
    sftp.put(local_path, remote_path)

sftp.close()

build_cmd = """
bash -l -c '
set -e
if [ -f /opt/ros/lyrical/setup.bash ]; then
    source /opt/ros/lyrical/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

cd /home/ayush/mecanum_robot/mecanum_host_ws
colcon build --packages-select ros2_rover
'
"""
print("[BUILD] Running colcon build on host...")
stdin, stdout, stderr = c.exec_command(build_cmd)
exit_status = stdout.channel.recv_exit_status()
out = stdout.read().decode()
err = stderr.read().decode()
print(f"[BUILD EXIT]: {exit_status}")
print(out)
if err:
    print("[BUILD STDERR]:", err)

test1_cmd = "bash -l -c 'cd /home/ayush/mecanum_robot/mecanum_host_ws/src/ros2_rover && python3 test_ros2_layer.py'"
print("[TEST 1] Running test_ros2_layer.py on host...")
stdin, stdout, stderr = c.exec_command(test1_cmd)
t1_exit = stdout.channel.recv_exit_status()
print(f"[TEST 1 EXIT]: {t1_exit}")
print(stdout.read().decode())
print(stderr.read().decode())

test2_cmd = "bash -l -c 'cd /home/ayush/mecanum_robot && python3 test_rover_modules.py'"
print("[TEST 2] Running test_rover_modules.py on host...")
stdin, stdout, stderr = c.exec_command(test2_cmd)
t2_exit = stdout.channel.recv_exit_status()
print(f"[TEST 2 EXIT]: {t2_exit}")
print(stdout.read().decode())
print(stderr.read().decode())

c.close()

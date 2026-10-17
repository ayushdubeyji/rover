#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f /home/ayush/mecanum_robot/mecanum_host_ws/install/setup.bash ]; then
    source /home/ayush/mecanum_robot/mecanum_host_ws/install/setup.bash
fi
export PYTHONPATH="/home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot:${PYTHONPATH}"
exec python3 /home/ayush/mecanum_robot/mecanum_host_ws/src/mecanum_robot/mecanum_robot/web_telemetry_server.py

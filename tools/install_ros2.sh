#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive
apt-get install -y -qq software-properties-common curl
add-apt-repository -y universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
apt-get update -qq
apt-get install -y -qq ros-jazzy-ros-base ros-jazzy-rosbridge-suite ros-jazzy-foxglove-bridge ros-jazzy-robot-localization ros-jazzy-imu-filter-madgwick ros-jazzy-tf2-ros ros-jazzy-tf2-tools python3-colcon-common-extensions python3-rosdep python3-serial
echo ROS_INSTALL_DONE

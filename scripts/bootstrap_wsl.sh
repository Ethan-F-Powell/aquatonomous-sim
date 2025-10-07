
#!/usr/bin/env bash
set -e

sudo apt update
sudo apt install -y curl gnupg lsb-release

# ROS 2 Humble
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# Gazebo Sim and bridge
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-bridge

# CycloneDDS
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Dev tools
sudo apt install -y git build-essential python3-vcstool

# Environment
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
grep -qxF 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' ~/.bashrc || echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
grep -qxF 'export ROS_DOMAIN_ID=42' ~/.bashrc || echo 'export ROS_DOMAIN_ID=42' >> ~/.bashrc

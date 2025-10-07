
#!/usr/bin/env bash
set -e
# Example headless run for Jetson
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
ros2 launch boat_bringup bringup.launch.py

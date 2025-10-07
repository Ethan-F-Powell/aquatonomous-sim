
#!/usr/bin/env bash
set -e
# Connect to an existing graph (Jetson or another machine) and open RViz
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix boat_bringup)/share/boat_bringup/config/rviz.rviz

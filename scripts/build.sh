
#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
cd sim_ws
colcon build
source install/setup.bash
ros2 pkg list | grep boat_bringup || true
echo "Build done."

#!/bin/bash
set -e

echo "=== Swarm Controller Starting ==="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-1}"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
else
  cd /root/ros2_ws
  colcon build --symlink-install
  source /root/ros2_ws/install/setup.bash
fi

ros2 launch swarm_control compose_decentralized.launch.py


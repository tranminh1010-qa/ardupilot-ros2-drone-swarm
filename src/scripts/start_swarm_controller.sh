#!/bin/bash
set -e

echo "=== Swarm Controller Starting ==="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-1}"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

cd /ros2_ws
colcon build --symlink-install --packages-select=swarm_control
source /ros2_ws/install/setup.bash

ros2 launch swarm_control compose_decentralized.launch.py


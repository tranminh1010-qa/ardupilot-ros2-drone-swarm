#!/bin/bash
set -e

echo "=== Swarm Controller Starting ==="
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-1}"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

cd /ros2_ws
colcon build --symlink-install --packages-select=swarm_control
source /ros2_ws/install/setup.bash

while ! ros2 node list 2>/dev/null | grep -q -E "(drone|ap)"; do
    echo "Waiting for ROS2 nodes to be available..."
    sleep 10
done

echo "=== ROS2 Topic Monitor $(date) ==="
timeout 5 ros2 topic list 2>/dev/null | grep -E "(drone|ap)"

ros2 launch swarm_control compose_decentralized.launch.py num_drones:="${NUM_DRONES:-2}"



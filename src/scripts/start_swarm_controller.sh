#!/bin/bash
set -e

echo "=== Swarm Controller Starting ==="
echo "NUM_DRONES: ${NUM_DRONES:-2}"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

cd /ros2_ws
colcon build --symlink-install --packages-select=swarm_control
source /ros2_ws/install/setup.bash

# Archive the previous run's artifacts: the prescription planner counts
# sidecar frames and the sprayers poll for prescription files — stale ones
# from an earlier mission would short-circuit both with old data.
if ls -d /root/logs/drone_* >/dev/null 2>&1 || [ -d /root/logs/prescriptions ]; then
    ARCHIVE="/root/logs/run_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$ARCHIVE"
    mv /root/logs/drone_* /root/logs/prescriptions "$ARCHIVE"/ 2>/dev/null || true
    echo "Previous mission artifacts archived to $ARCHIVE"
fi

export ROS_DOMAIN_ID=1
while ! ros2 node list 2>/dev/null | grep -q -E "(drone|ap)"; do
    echo "Waiting for ROS2 nodes to be available..."
    sleep 10
done

echo "=== ROS2 Topic Monitor $(date) ==="
timeout 5 ros2 topic list 2>/dev/null | grep -E "(drone|ap)"

if [ -n "${SPRAYER_INSTANCES:-}" ]; then
    ros2 launch swarm_control compose_decentralized.launch.py \
        num_drones:="${NUM_DRONES}" \
        sprayer_instances:="${SPRAYER_INSTANCES}"
else
    ros2 launch swarm_control compose_decentralized.launch.py \
        num_drones:="${NUM_DRONES}"
fi



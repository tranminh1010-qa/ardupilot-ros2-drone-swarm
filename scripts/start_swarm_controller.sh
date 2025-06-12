#!/bin/bash

# Swarm Controller Startup Script
# This script runs inside the swarm_controller container

set -e

echo "=== Swarm Controller Starting ==="
echo "NUM_DRONES: ${NUM_DRONES:-4}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-1}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the workspace if needed
cd /root/ros2_ws

if [ ! -f install/setup.bash ]; then
    echo "Building ROS2 workspace..."
    if [ -d src/swarm_control ]; then
        colcon build --packages-select swarm_control --symlink-install
        if [ $? -eq 0 ]; then
            echo "Build successful"
        else
            echo "Build failed, but continuing..."
        fi
    else
        echo "Warning: swarm_control package not found in src/"
    fi
fi

# Source the workspace
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo "ROS2 workspace sourced successfully"
else
    echo "Warning: ROS2 workspace not found, using system packages only"
fi

echo "Starting swarm controller for ${NUM_DRONES:-4} drones..."

# Wait for SITL instances to be ready
echo "Checking SITL readiness..."
for i in $(seq 1 ${NUM_DRONES:-4}); do
    PORT=$((14550 + (i-1) * 10))
    TIMEOUT=60
    COUNTER=0

    echo "Waiting for drone $i on port $PORT..."
    while ! nc -z localhost $PORT && [ $COUNTER -lt $TIMEOUT ]; do
        sleep 2
        COUNTER=$((COUNTER + 2))
        if [ $((COUNTER % 20)) -eq 0 ]; then
            echo "  Still waiting for drone $i... ($COUNTER/$TIMEOUT seconds)"
        fi
    done

    if [ $COUNTER -ge $TIMEOUT ]; then
        echo "Warning: Timeout waiting for drone $i on port $PORT"
    else
        echo "Drone $i ready on port $PORT"
    fi
done

echo "SITL readiness check completed"

# Check which launch files are available
LAUNCH_DIR="/root/ros2_ws/src/swarm_control/launch"
COMPOSE_LAUNCH="$LAUNCH_DIR/compose_decentralized.launch.py"
DEFAULT_LAUNCH="$LAUNCH_DIR/decentralized_swarm.launch.py"

if [ -f "$COMPOSE_LAUNCH" ]; then
    echo "Using Docker Compose optimized launch file"
    LAUNCH_FILE="swarm_control compose_decentralized.launch.py"
elif [ -f "$DEFAULT_LAUNCH" ]; then
    echo "Using standard decentralized launch file"
    LAUNCH_FILE="swarm_control decentralized_swarm.launch.py"
else
    echo "No launch files found. Starting basic monitoring..."

    # Basic monitoring loop
    while true; do
        echo "=== ROS2 Topic Monitor $(date) ==="
        timeout 5 ros2 topic list 2>/dev/null | grep -E "(drone|swarm|mavros|ap/)" || echo "No drone topics found"

        echo "Active nodes:"
        timeout 5 ros2 node list 2>/dev/null | grep -E "(drone|swarm)" || echo "No drone nodes found"

        echo "Port status:"
        for i in $(seq 1 ${NUM_DRONES:-4}); do
            PORT=$((14550 + (i-1) * 10))
            if nc -z localhost $PORT 2>/dev/null; then
                echo "  Drone $i: ✓ Port $PORT"
            else
                echo "  Drone $i: ✗ Port $PORT"
            fi
        done

        echo "---"
        sleep 30
    done

    exit 0
fi

# Launch the swarm
echo "Launching: ros2 launch $LAUNCH_FILE num_drones:=${NUM_DRONES:-4}"

# Set up error handling
set +e
ros2 launch $LAUNCH_FILE num_drones:=${NUM_DRONES:-4}
LAUNCH_EXIT_CODE=$?
set -e

if [ $LAUNCH_EXIT_CODE -ne 0 ]; then
    echo "Launch failed with exit code $LAUNCH_EXIT_CODE"
    echo "Starting fallback monitoring..."

    # Fallback monitoring
    while true; do
        echo "=== Fallback Monitor $(date) ==="
        echo "ROS2 topics:"
        timeout 5 ros2 topic list 2>/dev/null || echo "Failed to get topics"

        echo "ROS2 nodes:"
        timeout 5 ros2 node list 2>/dev/null || echo "Failed to get nodes"

        echo "---"
        sleep 60
    done
fi
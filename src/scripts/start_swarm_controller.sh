#!/bin/bash
set -e

echo "=== Swarm Controller Starting ==="
echo "NUM_DRONES: ${NUM_DRONES:-4}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-1}"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

echo "Waiting for micro_ros_agent instances..."
for i in $(seq 1 ${NUM_DRONES:-4}); do
    PORT=$((2018 + i))  # 2019, 2020, 2021, 2022
    echo "Checking micro_ros_agent drone $i on port $PORT..."

    timeout=30
    while ! nc -z localhost $PORT && [ $timeout -gt 0 ]; do
        sleep 1
        timeout=$((timeout - 1))
    done

    if [ $timeout -eq 0 ]; then
        echo "❌ micro_ros_agent drone $i not ready"
    else
        echo "✅ micro_ros_agent drone $i ready"
    fi
done

echo "Waiting for ArduPilot SITL instances..."
for i in $(seq 1 ${NUM_DRONES:-4}); do
    PORT=$((14550 + (i-1) * 10))
    echo "Checking ArduPilot drone $i on port $PORT..."

    timeout=30
    while ! nc -z localhost $PORT && [ $timeout -gt 0 ]; do
        sleep 1
        timeout=$((timeout - 1))
    done

    if [ $timeout -eq 0 ]; then
        echo "❌ ArduPilot drone $i not ready"
    else
        echo "✅ ArduPilot drone $i ready"
    fi
done

echo "=== System Ready - Starting Monitoring ==="

# Simple monitoring loop
while true; do
    echo "=== Status Check $(date '+%H:%M:%S') ==="

    # Check ROS topics
    echo "micro-ROS Topics:"
    ros2 topic list | grep -E "^/ap" | head -5 || echo "No /ap topics found"

    # Check nodes
    echo "ROS Nodes:"
    ros2 node list | grep -E "(drone|agent)" | head -3 || echo "No drone nodes found"

    # Check connections
    echo "Connections:"
    for i in $(seq 1 ${NUM_DRONES:-4}); do
        AGENT_PORT=$((2018 + i))
        SITL_PORT=$((14550 + (i-1) * 10))

        if nc -z localhost $AGENT_PORT 2>/dev/null && nc -z localhost $SITL_PORT 2>/dev/null; then
            echo "  Drone $i: ✅"
        else
            echo "  Drone $i: ❌"
        fi
    done

    echo "---"
    sleep 15
done
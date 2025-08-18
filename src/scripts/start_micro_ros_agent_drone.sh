#!/bin/bash

set -e

# Environment variables from Docker Compose
INSTANCE=${INSTANCE:-0}
NUM_DRONES=${NUM_DRONES:-2}
ROS_DOMAIN_ID=$((INSTANCE+1))
MICRO_ROS_AGENT_PORT=$((2019+INSTANCE))

echo "=== Micro ROS Agent Drone ${INSTANCE} Starting ==="
echo "NUM_DRONES: ${NUM_DRONES}"
echo "INSTANCE: ${INSTANCE}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "MICRO_ROS_AGENT_PORT: ${MICRO_ROS_AGENT_PORT}"

# Source ROS environment
source /opt/ros/humble/setup.bash
source /uros_ws/install/setup.bash

# Export ROS_DOMAIN_ID for this instance
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# Start micro-ros-agent for this drone instance
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "${MICRO_ROS_AGENT_PORT}" -v4
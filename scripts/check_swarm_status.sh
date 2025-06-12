#!/bin/bash

# Swarm Status Check Script
# Run this script from the host to check swarm status

NUM_DRONES=${1:-4}

echo "=== ArduPilot Swarm Status Check ==="
echo "Checking $NUM_DRONES drones..."
echo ""

# Check Docker containers
echo "=== Container Status ==="
if command -v docker &> /dev/null; then
    docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" | grep -E "(drone|swarm|ros2)"
else
    echo "Docker not available"
fi
echo ""

# Check ports
echo "=== Port Status ==="
for i in $(seq 1 $NUM_DRONES); do
    MAVLINK_PORT=$((14550 + (i-1) * 10))
    ROS_PORT=$((14551 + (i-1) * 10))

    if nc -z localhost $MAVLINK_PORT 2>/dev/null; then
        MAVLINK_STATUS="✓"
    else
        MAVLINK_STATUS="✗"
    fi

    if nc -z localhost $ROS_PORT 2>/dev/null; then
        ROS_STATUS="✓"
    else
        ROS_STATUS="✗"
    fi

    echo "Drone $i: MAVLink $MAVLINK_PORT $MAVLINK_STATUS | ROS $ROS_PORT $ROS_STATUS"
done
echo ""

# Check ROS2 topics (if possible)
echo "=== ROS2 Status ==="
if docker exec swarm_controller bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic list" 2>/dev/null | grep -q "/"; then
    echo "ROS2 topics available:"
    docker exec swarm_controller bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic list" 2>/dev/null | grep -E "(drone|swarm)" | head -10

    echo ""
    echo "ROS2 nodes:"
    docker exec swarm_controller bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 node list" 2>/dev/null | grep -E "(drone|swarm)" | head -10
else
    echo "ROS2 not accessible or no topics found"
fi
echo ""

# Check logs for errors
echo "=== Recent Errors ==="
echo "Checking recent container logs for errors..."

if docker logs swarm_controller --tail=20 2>/dev/null | grep -i error | tail -3; then
    echo "(Errors found in swarm_controller logs)"
else
    echo "No recent errors in swarm_controller"
fi

if docker compose -f compose.yaml logs drone --tail=50 2>/dev/null | grep -i error | head -3; then
    echo "(Errors found in drone logs)"
else
    echo "No recent errors in drone containers"
fi
echo ""

# Connection test
echo "=== Connection Test ==="
echo "Testing MAVLink connections..."
for i in $(seq 1 $NUM_DRONES); do
    PORT=$((14550 + (i-1) * 10))
    if timeout 3 bash -c "</dev/tcp/127.0.0.1/$PORT" 2>/dev/null; then
        echo "Drone $i: Connection OK"
    else
        echo "Drone $i: Connection FAILED"
    fi
done
echo ""

echo "=== Useful Commands ==="
echo "View all logs:           docker compose -f compose.yaml logs -f"
echo "View controller logs:    docker compose -f compose.yaml logs -f swarm_controller"
echo "View drone logs:         docker compose -f compose.yaml logs -f drone"
echo "Enter controller:        docker exec -it swarm_controller bash"
echo "Stop swarm:              docker compose -f compose.yaml down"
echo "Restart swarm:           bash start_swarm.sh -n $NUM_DRONES"
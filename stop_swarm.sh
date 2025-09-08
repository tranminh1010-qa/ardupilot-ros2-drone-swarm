#!/bin/bash

# Script to stop all swarm containers

echo "Stopping swarm containers..."

# Stop all drone and micro-ros-agent containers
docker ps -a | grep -E "drone-ardu-|micro-ros-agent-drone-" | awk '{print $1}' | xargs -r docker stop
docker ps -a | grep -E "drone-ardu-|micro-ros-agent-drone-" | awk '{print $1}' | xargs -r docker rm

# Stop compose services
docker compose down

# Kill any processes using the ports
for port in {2019..2029}; do
    pid=$(lsof -ti :$port 2>/dev/null)
    if [ ! -z "$pid" ]; then
        echo "Killing process on port $port (PID: $pid)"
        kill -9 $pid 2>/dev/null
    fi
done

echo "Swarm stopped and cleaned up"
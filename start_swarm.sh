#!/bin/bash

# Script to start the swarm with proper scaling
# Usage: ./start_swarm.sh [number_of_drones]

set -e

NUM_DRONES=${1:-2}

echo "Starting swarm with ${NUM_DRONES} drones..."

# Clean up any existing containers first
echo "Cleaning up existing containers..."
docker ps -a | grep -E "drone-ardu-|micro-ros-agent-drone" | awk '{print $1}' | xargs -r docker stop 2>/dev/null || true
docker ps -a | grep -E "drone-ardu-|micro-ros-agent-drone" | awk '{print $1}' | xargs -r docker rm 2>/dev/null || true

# Export NUM_DRONES so it's available to all services
export NUM_DRONES=${NUM_DRONES}

# Start only discovery server and swarm controller (don't start drone services)
docker compose up -d discovery_server swarm_controller --remove-orphans

# Wait for services to be ready
sleep 5

# Start scaled services with proper instance numbers
for ((i=0; i<$NUM_DRONES; i++)); do
    echo "Starting drone instance ${i}..."
    
    # Set environment variables for this instance
    export INSTANCE=$i
    export SYSID_THISMAV=$((i+1))
    export ROS_DOMAIN_ID=$((i+1))
    export MICRO_ROS_AGENT_PORT=$((2019+i))
    
    # Start micro-ros-agent for this drone
    docker compose run -d \
        -e INSTANCE=$i \
        -e NUM_DRONES=$NUM_DRONES \
        -e ROS_DOMAIN_ID=$((i+1)) \
        -e MICRO_ROS_AGENT_PORT=$((2019+i)) \
        --name micro-ros-agent-drone-$i \
        micro-ros-agent-drone
    
    # Determine this instance's role: SPRAYER_INSTANCES is a space-separated
    # list of 0-indexed instances that boot as sprayers (AC_Sprayer enabled
    # via sprayer.parm); all others are mappers.
    #   SPRAYER_INSTANCES="3" USE_GAZEBO=1 ./start_swarm.sh 4
    DRONE_ROLE=mapper
    case " ${SPRAYER_INSTANCES:-} " in
        *" $i "*) DRONE_ROLE=sprayer ;;
    esac

    # Start the drone
    # USE_GAZEBO=1 (exported before calling this script) couples SITL physics
    # to a Gazebo instance running on the host (see start_drone_container.sh).
    docker compose run -d \
        -e INSTANCE=$i \
        -e NUM_DRONES=$NUM_DRONES \
        -e SYSID_THISMAV=$((i+1)) \
        -e MICRO_ROS_AGENT_PORT=$((2019+i)) \
        -e USE_GAZEBO=${USE_GAZEBO:-0} \
        -e DRONE_ROLE=$DRONE_ROLE \
        --name drone-ardu-$i \
        drone_ardu
    if (( i < NUM_DRONES - 1 )); then
        echo "Waiting 15 seconds before starting next drone..."
        sleep 15
    fi
done

echo "Swarm started with ${NUM_DRONES} drones"
echo "To view logs: docker compose logs -f"
echo "To stop: docker compose down"
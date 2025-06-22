#!/bin/bash
set -e

# Environment variables from Docker Compose
INSTANCE=${INSTANCE:-0}
SYSID_THISMAV=${SYSID_THISMAV:-$((INSTANCE+1))}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$INSTANCE}

# Calculated ports
MAVLINK_TCP_PORT=$((5760 + 10 * INSTANCE))
SITL_PORT=$((5501 + 10 * INSTANCE))
MAVPROXY_UDP_PORT=$((14550 + INSTANCE))
DDS_UDP_PORT=$((2019 + INSTANCE))
GAZEBO_JSON_PORT=$((9002 + 10 * INSTANCE))

# Create instance directory for parameter isolation
mkdir -p /sitl/instance_${INSTANCE}
cd /sitl/instance_${INSTANCE}

# Wait for DDS Agent startup
sleep 5

# Start ArduPilot SITL with Gazebo and DDS
exec /ardupilot/Tools/autotest/sim_vehicle.py \
    --vehicle ArduCopter \
    --instance ${INSTANCE} \
    --frame gazebo-iris \
    --model=JSON \
    --out udp:0.0.0.0:${MAVPROXY_UDP_PORT} \
    --add-param-file=/config/dds_swarm.parm \
    --console \
    --map \
    --no-rebuild \
    --enable-dds
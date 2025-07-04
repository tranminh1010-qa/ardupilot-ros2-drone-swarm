#!/bin/bash

set -e

# Environment variables from Docker Compose
INSTANCE=${INSTANCE:-0}
SYSID_THISMAV=${SYSID_THISMAV:-$((INSTANCE+1))}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$INSTANCE}
MICRO_ROS_AGENT_PORT=${MICRO_ROS_AGENT_PORT:-2019}

# Calculated ports
MAVLINK_TCP_PORT=$((5760 + 10 * INSTANCE))
SITL_PORT=$((5501 + 10 * INSTANCE))
MAVPROXY_UDP_PORT=$((14550 + INSTANCE))
DDS_UDP_PORT=$((2019 + INSTANCE))
GAZEBO_JSON_PORT=$((9002 + 10 * INSTANCE))

# Create instance directory for parameter isolation
mkdir -p /sitl/instance_${INSTANCE}
cd /sitl/instance_${INSTANCE}

# UPDATE parameter file to set correct DDS port and sys_mav_id:
echo "DDS_UDP_PORT=${MICRO_ROS_AGENT_PORT}" > /tmp/instance_dds.parm
echo "SYSID_THISMAV=${SYSID_THISMAV}" >> /tmp/instance_dds.parm
cat /config/dds_swarm.parm >> /tmp/instance_dds.parm
cat /tmp/instance_dds.parm

# Start ArduPilot SITL with Gazebo and DDS

cd -
sim_vehicle.py \
    --vehicle ArduCopter \
    --out 127.0.0.1:${MAVPROXY_UDP_PORT} \
    --out 127.0.0.1:${DDS_UDP_PORT} \
    --out 127.0.0.1:${SITL_PORT} \
    --out 127.0.0.1:${GAZEBO_JSON_PORT} \
    --out 127.0.0.1:${MAVLINK_TCP_PORT} \
    --custom-location=40.072842,-105.230575,1586,0 \
    --sysid $((INSTANCE+1)) \
    --wipe False \
    --instance "${INSTANCE}" \
    --add-param-file=/tmp/instance_dds.parm \
    --enable-DDS
    #--model=JSON \  #model and frame require gazebo to be running
   # --frame gazebo-iris \


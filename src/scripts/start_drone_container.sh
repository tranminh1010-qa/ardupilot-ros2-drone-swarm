#!/bin/bash

set -e

LAT_BASE=40.072842
LON_BASE=-105.230575
ALT_BASE=1586
YAW=0

# Environment variables from Docker Compose
# When scaling with docker compose, INSTANCE should be passed as an env var
INSTANCE=${INSTANCE:-0}
NUM_DRONES=${NUM_DRONES:-2}

# Calculate derived values
SYSID_THISMAV=${SYSID_THISMAV:-$((INSTANCE+1))}
ROS_DOMAIN_ID=$((INSTANCE+1))
MICRO_ROS_AGENT_PORT=$((2019+INSTANCE))

# Offset calculation (approximately 5 meters between drones)
# 0.000045 degrees ≈ 5 meters
LAT_OFFSET=$(awk -v base=40.072842 -v inst="$INSTANCE" 'BEGIN{printf "%.6f", base + (inst * 0.000045)}')
LON_OFFSET=$(awk -v base=-105.230575 -v inst="$INSTANCE" 'BEGIN{printf "%.6f", base + (inst * 0.000045)}')

# Calculated ports
MAVLINK_TCP_PORT=$((5760 + 10 * INSTANCE))
SITL_PORT=$((5501 + 10 * INSTANCE))
MAVPROXY_UDP_PORT=$((14550 + INSTANCE))
DDS_UDP_PORT=$((2019 + INSTANCE))
GAZEBO_JSON_PORT=$((9002 + 10 * INSTANCE))

# Create instance directory for parameter isolation
mkdir -p /sitl/instance_"${INSTANCE}"
cd /sitl/instance_"${INSTANCE}"

# UPDATE parameter file to set correct DDS port, domain ID, and sys_mav_id:
touch instance_dds.parm
echo -e "DDS_UDP_PORT=${MICRO_ROS_AGENT_PORT}\nSYSID_THISMAV=${SYSID_THISMAV}\nDDS_DOMAIN_ID=${ROS_DOMAIN_ID}\n" > instance_dds.parm
cat /config/dds_swarm.parm >> instance_dds.parm

# Gazebo mode: sim_vehicle passes our file as the ONLY --defaults (frame defaults
# are not merged), so append gazebo-iris frame params AFTER dds_swarm.parm — the
# Gazebo iris model is an X-frame quad (FRAME_TYPE 1); dds_swarm.parm's
# FRAME_TYPE 0 (plus) would break motor mixing and the drones never lift off.
if [ "${USE_GAZEBO:-0}" = "1" ]; then
    cat /root/ardu_ws/src/ardupilot/Tools/autotest/default_params/gazebo-iris.parm >> instance_dds.parm
fi
cat instance_dds.parm

# Start ArduPilot SITL
# USE_GAZEBO=1 switches physics from SITL's built-in model to Gazebo via the
# JSON FDM backend: SITL instance N exchanges FDM data with the ArduPilotPlugin
# listening on host port 9002 + 10*N (see src/custom_gz/worlds/swarm_drone.sdf).
# Gazebo must be running on the host BEFORE the drones start, else SITL blocks.
if [ "${USE_GAZEBO:-0}" = "1" ]; then
    FRAME_ARGS="--frame gazebo-iris --model JSON"
else
    FRAME_ARGS=""
fi

cd -
sim_vehicle.py \
    --vehicle ArduCopter \
    --no-rebuild \
    ${FRAME_ARGS} \
    --out 127.0.0.1:${MAVPROXY_UDP_PORT} \
    --out 127.0.0.1:${DDS_UDP_PORT} \
    --out 127.0.0.1:${MAVLINK_TCP_PORT} \
    --custom-location="${LAT_OFFSET}","${LON_OFFSET}",${ALT_BASE},${YAW} \
    --sysid $((INSTANCE+1)) \
    --wipe False \
    --instance "${INSTANCE}" \
    --add-param-file=/sitl/instance_"${INSTANCE}"/instance_dds.parm \
    --enable-DDS


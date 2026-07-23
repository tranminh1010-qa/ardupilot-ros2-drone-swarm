#!/bin/bash

# Launch Gazebo (host) with the 4-drone swarm world.
# Run this BEFORE ./start_swarm.sh when using USE_GAZEBO=1:
#
#   ./start_gazebo.sh              # GUI
#   ./start_gazebo.sh --headless   # server only
#
#   USE_GAZEBO=1 ./start_swarm.sh 4
#
# Each drone model binds an FDM port (9002 + 10*instance) that SITL's JSON
# backend connects to (see src/scripts/start_drone_container.sh).

set -e

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
GZWS="/home/prince/Documents/projects/swarm/gz_ws/src/ardupilot_gazebo"

export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZWS}/build"
export GZ_SIM_RESOURCE_PATH="${REPO_DIR}/src/custom_gz/models:${REPO_DIR}/src/custom_gz/worlds:${GZWS}/models:${GZWS}/worlds"

WORLD="${REPO_DIR}/src/custom_gz/worlds/swarm_drone.sdf"

if [ "$1" = "--headless" ]; then
    exec gz sim -s -r -v2 "${WORLD}"
else
    exec gz sim -r -v2 "${WORLD}"
fi

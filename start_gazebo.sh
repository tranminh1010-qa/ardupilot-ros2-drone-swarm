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

# World selection: default is the 4-mapper world; set SWARM_WORLD to swap,
# e.g. the mixed fleet (3 camera mappers + 1 sprayer with spray plumes):
#   SWARM_WORLD=swarm_drone_spray.sdf ./start_gazebo.sh
WORLD="${REPO_DIR}/src/custom_gz/worlds/${SWARM_WORLD:-swarm_drone.sdf}"

# GstCameraPlugin does not stream until it receives a Boolean(true) on its
# <image_topic>/enable_streaming topic. Enable every drone camera once the
# world is up (runs in the background while gz sim starts).
(
    for _try in $(seq 1 60); do
        topics=$(gz topic -l 2>/dev/null | grep "down_camera/image/enable_streaming" || true)
        if [ -n "${topics}" ]; then
            sleep 2   # let the plugins finish subscribing
            for t in ${topics}; do
                gz topic -t "$t" -m gz.msgs.Boolean -p "data: true" >/dev/null 2>&1
            done
            echo "camera streams enabled: $(echo "${topics}" | wc -l)"
            exit 0
        fi
        sleep 2
    done
    echo "WARNING: camera enable_streaming topics never appeared" >&2
) &

if [ "$1" = "--headless" ]; then
    exec gz sim -s -r -v2 "${WORLD}"
else
    exec gz sim -r -v2 "${WORLD}"
fi

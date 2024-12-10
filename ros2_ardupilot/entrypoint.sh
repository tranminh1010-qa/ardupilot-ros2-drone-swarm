#!/bin/bash

# Source ROS2 setup
source /opt/ros/humble/setup.bash
if [ -f "/root/ardu_ws/install/setup.bash" ]; then
  source /root/ardu_ws/install/setup.bash
fi
# Execute the command passed to docker run
exec "$@"
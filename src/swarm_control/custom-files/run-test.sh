#!/bin/bash

# Check if xterm is installed
if ! command -v xterm &> /dev/null; then
    echo "xterm is not installed. Please install it first."
    exit 1
fi

# Terminal 1: Run sim_vehicle.py in a new interactive xterm window
xterm -hold -e "cd /root/ardu_ws/src/swarm_control/custom-files; sim_vehicle.py --vehicle-binary=./arducopter --add-param-file=./uas0.parm -v ArduCopter -f gazebo-iris --model JSON; exec bash" &

# Terminal 2: Run gz sim in a new interactive xterm window
xterm -hold -e "gz sim -v4 -r iris_runway.sdf; exec bash" &

# Terminal 3: Run movement-test.py in a new interactive xterm window
xterm -hold -e "cd /root/ardu_ws/src/swarm_control/custom-files; python movement-test.py; exec bash" &

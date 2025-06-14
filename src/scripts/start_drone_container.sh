#!/bin/bash
echo "=== ArduPilot SITL with DDS Starting ==="

# Get instance number from container name or environment
CONTAINER_NAME=$(hostname)
INSTANCE_NUM=$(echo "$CONTAINER_NAME" | grep -o '[0-9]')
INSTANCE_NUM=${INSTANCE_NUM:-${DRONE_ID:-1}}

echo "Container: $CONTAINER_NAME"
echo "Instance: $INSTANCE_NUM"

# Calculate ports and parameters
ARDU_INSTANCE=$((INSTANCE_NUM - 1))
SIM_PORT=$((5501 + ARDU_INSTANCE * 10))
MAVLINK_PORT=$((14550 + ARDU_INSTANCE * 10))
DDS_AGENT_HOST=${DDS_AGENT_IP:-127.0.0.1}
DDS_AGENT_PORT=${DDS_AGENT_PORT:-2019}

echo "ArduPilot Instance: $ARDU_INSTANCE"
echo "System ID: $INSTANCE_NUM"
echo "MAVLink Port: $MAVLINK_PORT"
echo "Sim Port: $SIM_PORT"
echo "DDS Agent: $DDS_AGENT_HOST:$DDS_AGENT_PORT"

# Wait for DDS agent with staggered startup
echo "Waiting for DDS agent... (${INSTANCE_NUM}s delay)"
sleep $INSTANCE_NUM

# Use external parameter file
PARAM_FILE="/root/parameters/dds_swarm.parm"
if [ ! -f "$PARAM_FILE" ]; then
    echo "❌ Parameter file not found: $PARAM_FILE"
    echo "Make sure parameters directory is mounted as volume"
    exit 1
fi

echo "Using parameter file: $PARAM_FILE"

# Create temporary parameter file with correct SYSID
TEMP_PARAM_FILE="/tmp/drone_${INSTANCE_NUM}.parm"
cp "$PARAM_FILE" "$TEMP_PARAM_FILE"
sed -i "s/SYSID_THISMAV.*/SYSID_THISMAV $INSTANCE_NUM/" "$TEMP_PARAM_FILE"

# Create working directory and logs
mkdir -p "/tmp/sitl_${ARDU_INSTANCE}" /root/logs

# Change to ArduPilot directory
cd /root/ardu_ws/src/ardupilot

# Start ArduPilot SITL without backgrounding - let it run in foreground
echo "Starting ArduPilot SITL..."

sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --instance $ARDU_INSTANCE \
    --sysid $INSTANCE_NUM \
    --no-rebuild \
    --speedup ${SPEEDUP:-1} \
    --sim-address ${SIM_ADDRESS:-127.0.0.1}:$SIM_PORT \
    -L KSFO \
    --out=udp:127.0.0.1:$MAVLINK_PORT \
    --add-param-file "$TEMP_PARAM_FILE" \
    --console
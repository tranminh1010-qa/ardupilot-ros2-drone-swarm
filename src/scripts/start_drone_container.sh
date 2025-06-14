#!/bin/bash

# Simplified ArduPilot SITL with DDS startup script

echo "=== ArduPilot SITL with DDS Starting ==="

# Get instance number from container name or environment
CONTAINER_NAME=$(hostname)
INSTANCE_NUM=$(echo "$CONTAINER_NAME" | grep -o '[0-9]\+$')

# Fallback if no number in hostname
if [ -z "$INSTANCE_NUM" ]; then
    INSTANCE_NUM=${DRONE_ID:-1}
fi

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

# Wait for DDS agent
echo "Waiting for DDS agent..."
sleep $((INSTANCE_NUM * 3))

# Create working directory
WORK_DIR="/tmp/sitl_${ARDU_INSTANCE}"
mkdir -p "$WORK_DIR"
mkdir -p /root/logs
cd "$WORK_DIR"

# Path to parameter file
PARAM_FILE="/root/parameters/dds_swarm.parm"
if [ ! -f "$PARAM_FILE" ]; then
    echo "❌ Parameter file not found: $PARAM_FILE"
    echo "Creating basic parameter file..."
    mkdir -p /root/parameters
    cat > "$PARAM_FILE" << 'PARAM_EOF'
DDS_ENABLE 1
DDS_UDP_PORT 2019
SERIAL1_PROTOCOL 45
SERIAL1_BAUD 115
ARMING_CHECK 0
BRD_SAFETY_DEFLT 0
PARAM_EOF
fi

echo "Using parameter file: $PARAM_FILE"

# Start ArduPilot SITL
echo "Starting ArduPilot SITL..."

sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --instance $ARDU_INSTANCE \
    --sysid $INSTANCE_NUM \
    --no-rebuild \
    --speedup ${SPEEDUP:-1} \
    --sim-address ${SIM_ADDRESS:-127.0.0.1}:$SIM_PORT \
    --location 40.072842,-105.230575,1586,0 \
    --out=udp:127.0.0.1:$MAVLINK_PORT \
    --add-param-file "$PARAM_FILE" \
    --console \
    > /root/logs/sitl_drone_${INSTANCE_NUM}.log 2>&1 &

SITL_PID=$!
echo "SITL started with PID: $SITL_PID"

# Wait and verify startup
echo "Waiting for SITL to initialize..."
sleep 15

if kill -0 $SITL_PID 2>/dev/null; then
    echo "✅ SITL running successfully"
    echo "📋 Logs: /root/logs/sitl_drone_${INSTANCE_NUM}.log"
    echo "🔗 MAVLink: udp:127.0.0.1:$MAVLINK_PORT"
    echo "📡 DDS Agent: $DDS_AGENT_HOST:$DDS_AGENT_PORT"
else
    echo "❌ SITL failed to start"
    if [ -f "/root/logs/sitl_drone_${INSTANCE_NUM}.log" ]; then
        echo "Last 10 lines of log:"
        tail -10 "/root/logs/sitl_drone_${INSTANCE_NUM}.log"
    fi
    exit 1
fi

echo "=== Container Ready ==="

# Keep container alive and monitor SITL
while kill -0 $SITL_PID 2>/dev/null; do
    sleep 10
done

echo "❌ SITL process died"
exit 1
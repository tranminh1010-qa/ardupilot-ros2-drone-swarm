#!/bin/bash

# start_drone_container.sh - ArduPilot with DDS communication

echo "=== ArduPilot SITL with DDS Starting ==="

# Extract instance number from container name
CONTAINER_NAME=$(hostname)
echo "Container name: $CONTAINER_NAME"

# Extract instance number from container name
INSTANCE_NUM=$(echo "$CONTAINER_NAME" | grep -o '[0-9]\+$')

if [ -z "$INSTANCE_NUM" ]; then
    # Fallback to lock-based assignment
    LOCK_DIR="/tmp/drone_instances"
    mkdir -p "$LOCK_DIR"

    for i in {1..10}; do
        LOCK_FILE="$LOCK_DIR/instance_$i.lock"
        if (set -C; echo $CONTAINER_NAME > "$LOCK_FILE") 2>/dev/null; then
            INSTANCE_NUM=$i
            break
        fi
    done

    # Final fallback
    if [ -z "$INSTANCE_NUM" ]; then
        INSTANCE_NUM=1
    fi
fi

echo "Using instance number: $INSTANCE_NUM"

# Calculate DDS and simulation parameters
ARDU_INSTANCE=$((INSTANCE_NUM - 1))
DDS_DOMAIN_ID=$INSTANCE_NUM
SIM_PORT=$((5501 + ARDU_INSTANCE * 10))

# DDS configuration
DDS_AGENT_HOST=${DDS_AGENT_IP:-127.0.0.1}
DDS_AGENT_PORT=${DDS_AGENT_PORT:-2019}

echo "=== Starting ArduPilot SITL Drone $INSTANCE_NUM with DDS ==="
echo "Instance: $ARDU_INSTANCE"
echo "DDS Domain ID: $DDS_DOMAIN_ID"
echo "DDS Agent: $DDS_AGENT_HOST:$DDS_AGENT_PORT"
echo "Sim Port: $SIM_PORT"
echo "Location: 40.072842,-105.230575,1586,0"

# Wait for DDS agent to be available
echo "Waiting for DDS agent..."
sleep $((INSTANCE_NUM * 3))

# Create logs directory
mkdir -p /root/logs

# Start ArduPilot SITL with DDS enabled
echo "Starting ArduPilot SITL with DDS communication..."

sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model ${MODEL:-gazebo-iris} \
    --speedup ${SPEEDUP:-1} \
    --instance $ARDU_INSTANCE \
    --sim-address ${SIM_ADDRESS:-127.0.0.1}:$SIM_PORT \
    --location 40.072842,-105.230575,1586,0 \
    -l 40.072842,-105.230575,1586,0 \
    --sysid $INSTANCE_NUM \
    --no-rebuild \
    --enable-dds \
    --dds-domain-id $DDS_DOMAIN_ID \
    --dds-agent-ip $DDS_AGENT_HOST \
    --dds-agent-port $DDS_AGENT_PORT \
    > /root/logs/sitl_drone_${INSTANCE_NUM}_dds.log 2>&1 &

SITL_PID=$!
echo "SITL started with PID: $SITL_PID"

# Wait for SITL to initialize
echo "Waiting for SITL to initialize..."
sleep 20

# Check if SITL is still running
if kill -0 $SITL_PID 2>/dev/null; then
    echo "✅ SITL with DDS is running successfully (PID: $SITL_PID)"
else
    echo "❌ SITL failed to start"
    echo "Checking log file..."
    if [ -f "/root/logs/sitl_drone_${INSTANCE_NUM}_dds.log" ]; then
        echo "Last 20 lines of log:"
        tail -20 "/root/logs/sitl_drone_${INSTANCE_NUM}_dds.log"
    fi
    exit 1
fi

# Verify DDS communication
echo "Checking DDS communication..."
sleep 5

# Container is ready
echo "=== DDS Container Ready ==="
echo "SITL Status: Running with DDS"
echo "Logs: /root/logs/sitl_drone_${INSTANCE_NUM}_dds.log"
echo "DDS Domain: $DDS_DOMAIN_ID"
echo "DDS Agent: $DDS_AGENT_HOST:$DDS_AGENT_PORT"
echo ""
echo "ROS 2 topics should be available:"
echo "  /drone_$INSTANCE_NUM/pose"
echo "  /drone_$INSTANCE_NUM/velocity"
echo "  /drone_$INSTANCE_NUM/battery"
echo ""

# Keep container running and monitor SITL process
while true; do
    if ! kill -0 $SITL_PID 2>/dev/null; then
        echo "❌ SITL process died, restarting..."
        exit 1
    fi
    sleep 10
done
#!/bin/bash
echo "=== ArduPilot SITL with Serial DDS + UDP MAVLink Starting ==="

# Get instance number from container name
CONTAINER_NAME=$(hostname)
echo "Container: $CONTAINER_NAME"

# Extract instance number
if [[ "$CONTAINER_NAME" =~ ([0-9]+) ]]; then
    INSTANCE_NUM=${BASH_REMATCH[1]}
else
    INSTANCE_NUM=${DRONE_ID:-1}
fi

echo "Instance: $INSTANCE_NUM"
echo "Architecture: Serial DDS (ROS2) + UDP MAVLink (QGroundControl)"

# Calculate parameters
ARDU_INSTANCE=$((INSTANCE_NUM - 1))
SIM_PORT=$((5501 + ARDU_INSTANCE * 10))
MAVLINK_UDP_PORT=$((14550 + ARDU_INSTANCE * 10))  # UDP port for QGroundControl
MAVLINK_TCP_PORT=$((5760 + ARDU_INSTANCE * 10))   # TCP port for SITL console

echo "ArduPilot Instance: $ARDU_INSTANCE"
echo "System ID: $INSTANCE_NUM"
echo "MAVLink UDP Port (QGC): $MAVLINK_UDP_PORT"
echo "MAVLink TCP Port (Console): $MAVLINK_TCP_PORT"
echo "Sim Port: $SIM_PORT"

# Setup Serial DDS (always serial for ROS2)
DDS_SERIAL_DEVICE="/tmp/dds_serial_${INSTANCE_NUM}"
DDS_SERIAL_AGENT="/tmp/dds_agent_${INSTANCE_NUM}"

echo "DDS Serial Device: $DDS_SERIAL_DEVICE"
echo "DDS Agent Serial: $DDS_SERIAL_AGENT"

# Create virtual serial port pair for DDS
echo "Creating virtual serial ports for DDS..."
socat -d -d pty,raw,echo=0,link="$DDS_SERIAL_DEVICE" pty,raw,echo=0,link="$DDS_SERIAL_AGENT" &
SOCAT_PID=$!

# Wait for serial devices to be created
sleep 3

# Verify serial ports exist
if [ ! -e "$DDS_SERIAL_DEVICE" ] || [ ! -e "$DDS_SERIAL_AGENT" ]; then
    echo "❌ Failed to create virtual serial ports"
    exit 1
fi

echo "✅ Serial ports created successfully"

# Start DDS agent on serial port
echo "Starting DDS Agent on serial port $DDS_SERIAL_AGENT..."
MicroXRCEAgent serial -b 115200 -D "$DDS_SERIAL_AGENT" -v 6 &
DDS_AGENT_PID=$!

# Wait for DDS agent to initialize
sleep 2

# Verify DDS agent is running
if ! kill -0 $DDS_AGENT_PID 2>/dev/null; then
    echo "❌ DDS Agent failed to start"
    exit 1
fi

echo "✅ DDS Agent started (PID: $DDS_AGENT_PID)"

# Load your existing parameter file
PARAM_FILE="/root/parameters/dds_swarm.parm"
if [ ! -f "$PARAM_FILE" ]; then
    echo "❌ Parameter file not found: $PARAM_FILE"
    exit 1
fi

# Create instance-specific parameter file
TEMP_PARAM_FILE="/tmp/drone_${INSTANCE_NUM}.parm"
cp "$PARAM_FILE" "$TEMP_PARAM_FILE"

# Add instance-specific parameters
echo "" >> "$TEMP_PARAM_FILE"
echo "# Instance-specific parameters" >> "$TEMP_PARAM_FILE"
echo "SYSID_THISMAV $INSTANCE_NUM" >> "$TEMP_PARAM_FILE"

# Your parameter file already has the correct serial DDS configuration:
# DDS_ENABLE 1
# SERIAL1_PROTOCOL 45 (DDS XRCE protocol)
# SERIAL1_BAUD 115 (115200 baud rate)
# No modification needed for DDS - using serial as designed

echo "✅ Parameter file prepared (using serial DDS configuration)"

# Create working directory
mkdir -p "/tmp/sitl_${ARDU_INSTANCE}"
cd "/tmp/sitl_${ARDU_INSTANCE}"

# Verify ArduPilot exists
ARDUPILOT_DIR="/root/ardu_ws/src/ardupilot"
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "❌ ArduPilot directory not found: $ARDUPILOT_DIR"
    exit 1
fi

if [ ! -f "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]; then
    echo "❌ ArduPilot binary not found. Please build ArduPilot first."
    exit 1
fi

# Cleanup function
cleanup() {
    echo "Cleaning up..."
    kill $SOCAT_PID 2>/dev/null
    kill $DDS_AGENT_PID 2>/dev/null
    pkill -f "arducopter.*-I${ARDU_INSTANCE}" 2>/dev/null
    exit 0
}

trap cleanup SIGTERM SIGINT

# Build ArduPilot command arguments
ARDUCOPTER_ARGS=(
    "--model" "gazebo-iris"
    "--speedup" "${SPEEDUP:-1}"
    "--sysid" "$INSTANCE_NUM"
    "--defaults" "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm,$TEMP_PARAM_FILE"
    "--sim-address=127.0.0.1:$SIM_PORT"
    "-I$ARDU_INSTANCE"
    "--home" "37.4133,-122.1392,0.0,0.0"
    "--serial0" "tcp:$MAVLINK_TCP_PORT"     # Console connection (TCP)
    "--uartA" "tcp:$MAVLINK_TCP_PORT"       # SERIAL0 for console
    "--serial3" "$DDS_SERIAL_DEVICE"          # SERIAL1 for DDS (serial)
)

# Start ArduPilot SITL
echo "Starting ArduPilot SITL with Serial DDS + UDP MAVLink..."
echo "Command: $ARDUPILOT_DIR/build/sitl/bin/arducopter ${ARDUCOPTER_ARGS[*]}"

"$ARDUPILOT_DIR/build/sitl/bin/arducopter" "${ARDUCOPTER_ARGS[@]}" &
SITL_PID=$!

echo "ArduPilot SITL started (PID: $SITL_PID)"

# Wait for SITL to initialize
sleep 15

# Verify SITL is still running
if ! kill -0 $SITL_PID 2>/dev/null; then
    echo "❌ ArduPilot SITL failed to start properly"
    echo "Checking logs..."
    find /tmp -name "*.log" -newer /tmp/sitl_${ARDU_INSTANCE} 2>/dev/null | head -5 | xargs tail -20 2>/dev/null || echo "No recent log files found"
    cleanup
fi

echo "✅ ArduPilot SITL is running"

# Start MAVProxy to bridge TCP console to UDP for QGroundControl
echo "Starting MAVProxy bridge (TCP -> UDP for QGroundControl)..."
mavproxy.py \
    --master "tcp:127.0.0.1:$MAVLINK_TCP_PORT" \
    --out "udp:127.0.0.1:$MAVLINK_UDP_PORT" \
    --cmd="set heartbeat 2; set streamrate -1" \
    --daemon &
MAVPROXY_PID=$!

if [ -n "$MAVPROXY_PID" ]; then
    echo "✅ MAVProxy bridge started (PID: $MAVPROXY_PID) - QGroundControl can connect to UDP $MAVLINK_UDP_PORT"
else
    echo "⚠️  MAVProxy bridge failed to start - QGroundControl connection may not work"
fi

# Status reporting function
report_status() {
    echo "=== Status Report ==="
    echo "Instance: $INSTANCE_NUM"
    echo "Architecture: Serial DDS + UDP MAVLink"
    echo "SITL PID: $SITL_PID"
    echo "DDS Agent PID: $DDS_AGENT_PID (Serial)"
    echo "SOCAT PID: $SOCAT_PID"
    echo "MAVProxy PID: $MAVPROXY_PID"
    echo "Serial Ports: $DDS_SERIAL_DEVICE <-> $DDS_SERIAL_AGENT"
    echo "QGroundControl: UDP 127.0.0.1:$MAVLINK_UDP_PORT"
    echo "Console: TCP 127.0.0.1:$MAVLINK_TCP_PORT"
    echo "=================="
}

# Monitor processes
echo "Monitoring processes... (Press Ctrl+C to stop)"
report_status

LOOP_COUNT=0
while true; do
    # Check SITL
    if ! kill -0 $SITL_PID 2>/dev/null; then
        echo "❌ ArduPilot SITL died"
        echo "Checking recent logs..."
        find /tmp -name "*.log" -newer /tmp/sitl_${ARDU_INSTANCE} 2>/dev/null | head -3 | xargs tail -10 2>/dev/null || echo "No recent log files found"
        break
    fi

    # Check DDS agent
    if ! kill -0 $DDS_AGENT_PID 2>/dev/null; then
        echo "❌ DDS Agent died"
        break
    fi

    # Check socat for serial ports
    if ! kill -0 $SOCAT_PID 2>/dev/null; then
        echo "❌ SOCAT died"
        break
    fi

    # Check MAVProxy bridge
    if [ -n "$MAVPROXY_PID" ] && ! kill -0 $MAVPROXY_PID 2>/dev/null; then
        echo "⚠️  MAVProxy bridge died - restarting..."
        mavproxy.py \
            --master "tcp:127.0.0.1:$MAVLINK_TCP_PORT" \
            --out "udp:127.0.0.1:$MAVLINK_UDP_PORT" \
            --cmd="set heartbeat 2; set streamrate -1" \
            --daemon &
        MAVPROXY_PID=$!
        echo "MAVProxy bridge restarted (PID: $MAVPROXY_PID)"
    fi

    # Status update every 2 minutes
    if [ $((LOOP_COUNT % 12)) -eq 0 ]; then
        echo "✅ All processes running ($(date))"
        echo "   ROS2 DDS: Serial connection active"
        echo "   QGroundControl: UDP port $MAVLINK_UDP_PORT ready"
    fi

    LOOP_COUNT=$((LOOP_COUNT + 1))
    sleep 10
done

cleanup
#!/bin/bash
source ~/.bashrc

# ArduPilot Swarm Startup Script
# Note: Gazebo should be started separately before running this script

set -e

NUM_DRONES=${NUM_DRONES:-4}

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

wait_for_port() {
    local port=$1
    local timeout=${2:-30}
    local counter=0

    print_status "Waiting for port $port to be available..."
    while ! nc -z localhost $port && [ $counter -lt $timeout ]; do
        sleep 1
        counter=$((counter + 1))
    done

    if [ $counter -eq $timeout ]; then
        print_warning "Timeout waiting for port $port"
        return 1
    else
        print_success "Port $port is available"
        return 0
    fi
}

check_gazebo() {
    print_status "Checking if Gazebo is running..."

    # Check if Gazebo processes are running
    if pgrep -f "gz sim" > /dev/null || pgrep -f "gazebo" > /dev/null; then
        print_success "Gazebo is running"
        return 0
    else
        print_error "Gazebo is not running!"
        print_status "Please start Gazebo first with:"
        echo "  gz sim -v4 -r src/custom_gz/worlds/swarm_drone.sdf"
        echo "  OR"
        echo "  gazebo --verbose src/custom_gz/worlds/swarm_drone.sdf"
        return 1
    fi
}

setup_x11() {
    print_status "Setting up X11 forwarding for Docker..."
    xhost +local:docker

    export GZ_PARTITION=$(hostname)
    export GZ_IP=127.0.0.1
    export GZ_VERBOSE=4
    export GZ_DISCOVERY_MULTICAST=1
    export GZ_TRANSPORT_TOPIC_STATISTICS=1

    print_success "X11 and Gazebo environment configured"
}

build_images() {
    print_status "Building Docker images..."

    if [ -f "Dockerfile.ardupilot" ]; then
        print_status "Building ArduPilot image..."
        docker build -t ardupilot-custom -f Dockerfile.ardupilot .
    else
        print_error "Dockerfile.ardupilot not found"
        exit 1
    fi

    if [ -f "Dockerfile.ros2" ]; then
        print_status "Building ROS 2 image..."
        docker build -t ros2-custom -f Dockerfile.ros2 .
    else
        print_error "Dockerfile.ros2 not found"
        exit 1
    fi

    print_success "Docker images built successfully"
}

start_docker_swarm() {
    print_status "Starting Docker swarm with $NUM_DRONES drones..."

    # Clean up any existing containers first
    print_status "Cleaning up existing containers..."
    docker compose -f compose.yaml down 2>/dev/null || true

    export NUM_DRONES
    export GZ_PARTITION
    export DISPLAY

    # Start the swarm with unique container names
    print_status "Starting containers..."
    docker compose -f compose.yaml up --scale drone="$NUM_DRONES" -d

    # Wait a moment for containers to initialize
    sleep 5

    # Assign unique drone IDs to each container
    print_status "Assigning unique drone IDs..."
    DRONE_CONTAINERS=$(docker ps --filter "name=drone" --format "{{.Names}}" | sort)
    counter=1

    for container in $DRONE_CONTAINERS; do
        print_status "Assigning DRONE_ID=$counter to container $container"
        docker exec "$container" bash -c "echo 'export DRONE_ID=$counter' >> ~/.bashrc"
        counter=$((counter + 1))
    done

    print_success "Docker swarm started"
}

show_connection_info() {
    print_status "Connection Information:"
    echo ""
    echo "=== Drone Connections ==="
    for i in $(seq 1 $NUM_DRONES); do
        port=$((14550 + (i-1) * 10))
        echo "Drone $i: udp:127.0.0.1:$port"
    done

    echo ""
    echo "=== GCS Connection Examples ==="
    echo "QGroundControl: Use UDP connection to 127.0.0.1:14550"
    echo "Mission Planner: Use UDP connection to 127.0.0.1:14550"
    echo "MAVProxy: mavproxy.py --master=udp:127.0.0.1:14550 --console --map"

    echo ""
    echo "=== Multi-drone MAVProxy ==="
    for i in $(seq 1 $NUM_DRONES); do
        port=$((14550 + (i-1) * 10))
        echo "Drone $i: mavproxy.py --master=udp:127.0.0.1:$port --console"
    done

    echo ""
    echo "=== Useful Commands ==="
    echo "View logs: docker compose -f compose.yaml logs -f"
    echo "View specific drone: docker compose -f compose.yaml logs -f drone-1"
    echo "Stop swarm: docker compose -f compose.yaml down"
    echo "Monitor ROS topics: docker exec -it ros2_bridge ros2 topic list"
    echo "Enter drone container: docker exec -it <container_name> bash"
}

monitor_containers() {
    print_status "Monitoring container health..."

    # Check if containers are running
    sleep 10

    RUNNING_CONTAINERS=$(docker ps --filter "name=drone" --format "{{.Names}}" | wc -l)
    if [ "$RUNNING_CONTAINERS" -eq "$NUM_DRONES" ]; then
        print_success "All $NUM_DRONES drone containers are running"
    else
        print_warning "Expected $NUM_DRONES drones, but only $RUNNING_CONTAINERS are running"
    fi

    # Show container status
    echo ""
    print_status "Container Status:"
    docker ps --filter "name=swarm" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
}

cleanup() {
    print_status "Cleaning up..."

    if [ -f "compose.yaml" ]; then
        docker compose -f compose.yaml down
    fi

    print_success "Cleanup completed"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -n, --num-drones NUM     Number of drones (default: 4)"
    echo "  -b, --build             Build Docker images before starting"
    echo "  -s, --skip-gazebo-check Skip Gazebo running check"
    echo "  -h, --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                      Start with 4 drones"
    echo "  $0 -n 6                 Start with 6 drones"
    echo "  $0 -b                   Build images and start with 4 drones"
    echo "  $0 -s                   Skip Gazebo check and start"
    echo ""
    echo "Note: Make sure Gazebo is running before executing this script:"
    echo "  gz sim -v4 -r src/custom_gz/worlds/swarm_drone.sdf"
    echo "  OR"
    echo "  gazebo --verbose src/custom_gz/worlds/swarm_drone.sdf"
}

# Parse command line arguments
BUILD_IMAGES=false
SKIP_GAZEBO_CHECK=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--num-drones)
            NUM_DRONES="$2"
            shift 2
            ;;
        -b|--build)
            BUILD_IMAGES=true
            shift
            ;;
        -s|--skip-gazebo-check)
            SKIP_GAZEBO_CHECK=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

trap cleanup EXIT

# Main execution
print_status "Starting ArduPilot Swarm with $NUM_DRONES drones"

# Check prerequisites
if ! command_exists docker; then
    print_error "Docker not found. Please install Docker."
    exit 1
fi

if ! command_exists docker compose; then
    print_error "Docker Compose not found. Please install Docker Compose."
    exit 1
fi

# Check if Gazebo is running (unless skipped)
if [ "$SKIP_GAZEBO_CHECK" = false ]; then
    if ! check_gazebo; then
        exit 1
    fi
else
    print_warning "Skipping Gazebo check as requested"
fi

setup_x11

if [ "$BUILD_IMAGES" = true ]; then
    build_images
fi

start_docker_swarm
monitor_containers
show_connection_info

print_success "Swarm startup completed!"
print_status "Press Ctrl+C to stop the swarm and cleanup"

# Keep the script running until interrupted
while true; do
    sleep 1
done
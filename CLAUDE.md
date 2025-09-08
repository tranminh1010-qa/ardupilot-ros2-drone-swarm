# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ArduPilot ROS2 Swarm - A scalable drone swarm simulation system combining ArduPilot SITL with ROS2 for autonomous multi-drone operations using Docker containers.

## Essential Commands

### Starting the Swarm
```bash
# Build and start swarm (use -b flag to rebuild Docker images)
./start_swarm.sh -b    # Build images first
./start_swarm.sh 4     # Start with 4 drones
./start_swarm.sh 8     # Start with 8 drones

# Check swarm status
./src/scripts/check_swarm_status.sh 4  # Check status for 4 drones
```

### Stopping the Swarm
```bash
./stop_swarm.sh
```

### Development and Debugging
```bash
# Access containers
docker exec -it swarm_controller bash
docker exec -it drone-ardu-0 bash      # Replace 0 with drone ID

# Monitor logs
docker compose logs -f
docker compose logs -f drone-ardu-0

# Monitor ROS2 topics (inside swarm_controller)
ros2 topic list
ros2 topic echo /drone_0/state
```

### Testing Individual Components
```bash
# Test single drone (inside container)
ros2 run swarm_control base_drone_dds.py --ros-args -p drone_id:=0

# Launch multiple drones with ROS2
ros2 launch swarm_control compose_decentralized.launch.py num_drones:=4
```

## Architecture Overview

### Core Communication Flow
```
ArduPilot SITL ←→ DDS/Micro-ROS Agent ←→ ROS2 Nodes (base_drone_dds.py)
     ↓
MAVLink (port 5760+ID for GCS connection)
```

### Key Components

1. **BaseDrone** (`src/swarm_control/scripts/base_drone_dds.py`): Main drone control node
   - Handles state management, navigation, and mission execution
   - Communicates via DDS with ArduPilot
   - Implements autonomous takeoff, waypoint following, and landing

2. **Swarm Controller** (`src/swarm_control/scripts/points_distributor.py`): Waypoint distribution
   - Generates grid or circular patterns for multiple drones
   - Coordinates mission timing across swarm

3. **Docker Services**:
   - `discovery_server`: FastRTPS discovery for ROS2 networking
   - `swarm_controller`: ROS2 control nodes and launch files
   - `drone-ardu-{ID}`: ArduPilot SITL instances (dynamically scaled)
   - `micro-ros-agent-drone-{ID}`: DDS communication bridges

### Port Allocation
- MAVLink: 5760 + drone_id (e.g., drone 0 = 5760, drone 1 = 5761)
- SITL: 9002 + drone_id
- ROS2 Domain: 10 + drone_id (domain isolation per drone)

## Critical Implementation Details

### GPS and Coordinate Systems
- GPS origin simulation: Base lat/lon defined in parameters
- Each drone gets offset position based on ID (5m spacing)
- Coordinate transformations handled by `pyproj` and `geopy`
- Home position must be set before arming

### State Management
The system uses a state machine in `base_drone_dds.py`:
- IDLE → ARMING → TAKING_OFF → NAVIGATING → LANDING → LANDED
- State transitions controlled by ArduPilot mode changes and mission progress

### Message Types
- ArduPilot DDS messages: `GlobalPosition`, `Heartbeat`, `Altitude`, `LocalPose`
- Custom ROS2 messages: `DroneState`, `MissionCommand`
- MAVLink protocol for GCS compatibility

### Domain Isolation
Each drone operates in its own ROS2 domain (10 + drone_id) to prevent cross-talk between instances.

## Common Development Tasks

### Adding New Drone Behaviors
Modify `src/swarm_control/scripts/base_drone_dds.py`:
- Add new states to the state machine
- Implement behavior in the main control loop
- Update state transition logic

### Modifying Waypoint Patterns
Edit `src/swarm_control/scripts/points_distributor.py`:
- Implement new pattern generation functions
- Update the waypoint distribution logic

### Adjusting Drone Parameters
ArduPilot parameters in `src/dockerfiles/config/`:
- Flight modes, speeds, and safety settings
- GPS simulation parameters

## Environment Variables and Configuration

Key environment variables set in Docker Compose:
- `DRONE_ID`: Unique identifier for each drone
- `ROS_DOMAIN_ID`: ROS2 domain for network isolation
- `SITL_PORT`: ArduPilot SITL communication port
- `MAVLINK_PORT`: Ground station connection port

## Dependencies and Requirements

- Host system: Ubuntu 22.04+ with Docker and Docker Compose
- Gazebo simulation (runs on host, not in containers)
- Python packages: pymavlink, dronekit, numpy, pyproj, geopy
- ROS2 Humble with FastRTPS or CycloneDDS

## Debugging Tips

1. Check container health: `docker ps` and verify all services are running
2. Verify ROS2 communication: Use `ros2 topic list` inside swarm_controller
3. Monitor ArduPilot: Connect QGroundControl to ports 5760+
4. Check DDS agent logs: `docker logs micro-ros-agent-drone-0`
5. Verify GPS initialization: Ensure home position is set before arming
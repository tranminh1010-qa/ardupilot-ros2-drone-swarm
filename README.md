# ArduPilot ROS2 Swarm

Multi-drone swarm simulation using ArduPilot SITL, ROS2, and Docker Compose.

## Quick Start

```bash
# Start swarm with N drones (builds images if needed)
./start_swarm.sh -b    # Build Docker images first
./start_swarm.sh 4     # Launch 4 drones

# Monitor status
./src/scripts/check_swarm_status.sh 4

# Stop all drones
./stop_swarm.sh
```

## Architecture

Each drone runs in isolated Docker containers:
- **ArduPilot SITL** container with Micro-ROS Agent (DDS bridge)
- **Shared ROS2 controller** for swarm coordination
- **FastRTPS Discovery Server** for ROS2 networking

### Port Allocation
- MAVLink GCS: `5760 + drone_id` (connect QGroundControl here)
- SITL: `9002 + drone_id`
- ROS2 Domain: `10 + drone_id` (network isolation)

## File Structure

```
ardupilot_ros2_swarm/
├── docker-compose.yml          # Core services (discovery, controller)
├── docker-compose.drones.yml   # Drone template (scaled dynamically)
├── start_swarm.sh              # Main launch script
├── stop_swarm.sh              # Cleanup script
├── .env                        # Environment configuration
│
├── src/
│   ├── swarm_control/         # ROS2 control package
│   │   ├── scripts/
│   │   │   ├── base_drone_dds.py      # Main drone control node (DDS)
│   │   │   ├── points_distributor.py  # Waypoint pattern generator
│   │   │   └── compose_decentralized.launch.py  # ROS2 launch file
│   │   └── msg/               # Custom ROS2 messages
│   │
│   ├── dockerfiles/           # Container definitions
│   │   ├── ardupilot.dockerfile       # SITL + Micro-ROS Agent
│   │   ├── swarm_controller.dockerfile # ROS2 control nodes
│   │   └── config/            # ArduPilot parameters
│   │
│   └── scripts/
│       ├── init_swarm.sh      # Container startup scripts
│       └── check_swarm_status.sh  # Health monitoring
```

## Docker Compose Details

### Service Scaling
The `start_swarm.sh` script dynamically scales drone services:
```bash
docker compose -f docker-compose.yml -f docker-compose.drones.yml \
    up -d --scale drone-ardu=$NUM_DRONES --scale micro-ros-agent-drone=$NUM_DRONES
```

### Network Architecture
- **swarm_network**: Bridge network for all containers
- Each drone uses unique ROS_DOMAIN_ID for topic isolation
- Discovery server enables cross-domain communication when needed

### Container Services

#### `discovery_server`
FastRTPS discovery service for ROS2 node discovery across domains.

#### `swarm_controller`
- Runs ROS2 control nodes
- Launches `compose_decentralized.launch.py` for N drones
- Distributes waypoints and coordinates missions

#### `drone-ardu-{ID}`
- ArduPilot SITL instance
- Micro-ROS Agent for DDS↔MAVLink bridge
- Unique ports per drone ID

### Environment Variables
Set in `.env` and docker-compose:
- `NUM_DRONES`: Total drone count
- `DRONE_ID`: Per-container unique ID (0-indexed)
- `ROS_DOMAIN_ID`: Network isolation (10 + DRONE_ID)
- `SITL_PORT`: ArduPilot communication (9002 + DRONE_ID)
- `MAVLINK_PORT`: GCS connection (5760 + DRONE_ID)

## Development

### Access Containers
```bash
docker exec -it swarm_controller bash
docker exec -it drone-ardu-0 bash
```

### Monitor ROS2 Topics
```bash
# Inside swarm_controller
ros2 topic list
ros2 topic echo /drone_0/state
```

### View Logs
```bash
docker compose logs -f drone-ardu-0
docker compose logs -f swarm_controller
```

### Connect Ground Station
Open QGroundControl and add TCP connections:
- Drone 0: `localhost:5760`
- Drone 1: `localhost:5761`
- etc.

## Communication Flow

```
ArduPilot SITL ←→ Micro-ROS Agent (DDS) ←→ ROS2 Nodes
     ↓                                          ↓
MAVLink (TCP)                            ROS2 Topics
     ↓                                          ↓
QGroundControl                          Swarm Controller
```

## Notes

- Gazebo runs on host (not containerized) for visualization
- Each drone spawns with 5m offset based on ID
- GPS origin simulated via ArduPilot parameters
- State machine: IDLE → ARMING → TAKING_OFF → NAVIGATING → LANDING
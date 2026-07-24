# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ArduPilot ROS2 Swarm - A scalable drone swarm simulation system combining ArduPilot SITL with ROS2 for autonomous multi-drone operations using Docker containers.

## Essential Commands

### Starting the Swarm
```bash
# start_swarm.sh takes ONLY a drone count (no build flag). Default is 2.
./start_swarm.sh 4     # Start with 4 drones
./start_swarm.sh 8     # Start with 8 drones

# Rebuild images after Dockerfile/dependency changes, then start
docker compose build
./start_swarm.sh 4

# Check swarm status
./src/scripts/check_swarm_status.sh 4  # Check status for 4 drones
```

`start_swarm.sh` first brings up `discovery_server` + `swarm_controller` via
`docker compose up -d`, then loops `docker compose run -d` per instance to launch the
`drone_ardu` and `micro-ros-agent-drone` services (both gated behind the `manual` compose
profile), naming containers `drone-ardu-<i>` and `micro-ros-agent-drone-<i>`.

### Gazebo Visualization (optional)
```bash
# Start Gazebo FIRST (host GUI; --headless for server only), THEN the swarm:
./start_gazebo.sh &
USE_GAZEBO=1 ./start_swarm.sh 4
```
`USE_GAZEBO=1` switches SITL physics to Gazebo's JSON FDM backend (`--frame
gazebo-iris --model JSON`); each drone model in `src/custom_gz/worlds/swarm_drone.sdf`
binds FDM port `9002 + 10*instance` via a dedicated model dir
(`src/custom_gz/models/base_drone{,_1,_2,_3}`) — SDF `<include>` cannot override a
nested plugin's port, so the port is baked into each model. In Gazebo mode the
container startup appends `gazebo-iris.parm` (X-frame) after `dds_swarm.parm`;
without it the plus-frame mixing produces no lift while the mission runs open-loop.
Requires the ardupilot_gazebo plugin build on the host (paths set in `start_gazebo.sh`).

Verify flight by actual `rel_alt` gain (pymavlink on UDP `14550+instance`), never by
takeoff service ACKs — those succeed even if the vehicle stays on the ground.

Gotcha: QGroundControl's virtual joystick (`virtualJoystick=true` in
`~/.config/QGroundControl/QGroundControl.ini`) streams centered throttle to its active
vehicle, which then fails arming with "Throttle too high". Keep it disabled.

### Camera Mapping Pipeline (OpenCV)
Each drone flies a balanced 1/n serpentine strip of the field
(`split_serpentine` in `points_distributor.py` — exactly n contiguous chunks,
sizes differ by ≤1, nothing dropped; the old `split_by_grid` was unbalanced and
silently dropped waypoints for non-square drone counts). All drones share one
field origin (`field_origin_lat/lon` params, set in the launch file — must match
`LAT_BASE/LON_BASE` in `start_drone_container.sh`) so the strips tile a single
field instead of shifting with each drone's home.

The camera is an **extension**, at both layers:
- Models: `base_drone_cam{,_1,_2,_3}` wrap the plain `base_drone*` models
  (iris_with_gimbal pattern: `<include>` + fixed joint) and add a down-facing
  camera streaming H.264/RTP via GstCameraPlugin on UDP `5600 + instance`.
  The world references the `_cam` wrappers; swap back to `base_drone*` to fly
  camera-less. GstCameraPlugin only streams after a Boolean(true) on its
  `.../image/enable_streaming` gz topic — `start_gazebo.sh` does this
  automatically.
- Nodes: `camera_drone_dds.py` defines `CameraDrone(BaseDrone)` which overrides
  the `on_waypoint_reached` hook to capture a geotagged frame (+ ArUco
  detection) per waypoint into `/root/logs/drone_<id>/` (host: `./logs`).
  `BaseDrone` stays camera-free; camera failures never break the mission.
  The capture uses OpenCV's GStreamer backend — the container needs Ubuntu's
  `python3-opencv` (pip's opencv-python lacks GStreamer).

Simulated weeds: `src/custom_gz/worlds/generate_weeds.py` generates
`models/weed_field` — oversized magenta visual-only blobs over the surveyed
80×80 m area (seeded; ground truth lat/lon in
`models/weed_field/ground_truth.json`), included by both swarm worlds.
`camera_processor.py` detects them per waypoint via an HSV band
(H 140–170) + contour area filter and georeferences each detection
(nadir approximation, yaw not compensated); results land in the sidecar
JSON as `weed_detections`. Regenerate the layout with
`python3 src/custom_gz/worlds/generate_weeds.py --count N --seed S`.

After a mission, build the field mosaic on the host:
```bash
python3 src/scripts/stitch_field_map.py --logs ./logs --out field_map.jpg
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
MAVLink TCP (port 5760 + 10*ID for GCS connection)
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
Authoritative source is `src/scripts/start_drone_container.sh` (`INSTANCE` is 0-indexed):
- MAVLink TCP (GCS/QGC):   `5760 + 10*INSTANCE`  (drone 0 = 5760, drone 1 = 5770)
- SITL:                    `5501 + 10*INSTANCE`
- MAVProxy UDP out:        `14550 + INSTANCE`
- Micro-ROS Agent / DDS:   `2019 + INSTANCE`
- Gazebo JSON:             `9002 + 10*INSTANCE`
- ROS_DOMAIN_ID:           `INSTANCE + 1`  (domain isolation per drone)
- SYSID_THISMAV:           `INSTANCE + 1`

Note: `check_swarm_status.sh` computes the MAVProxy port as `14550 + (i-1)*10` with a
1-indexed loop — a known inconsistency with the per-instance formula above.

## Critical Implementation Details

### GPS and Coordinate Systems
- GPS origin simulation: Base lat/lon defined in parameters
- Each drone gets offset position based on ID (5m spacing)
- Coordinate transformations handled by `pyproj` and `geopy`
- Home position must be set before arming

### State Management
The `DroneState` enum lives in `src/swarm_control/scripts/drone_state.py` (imported by
`base_drone_dds.py`). The full set of states:
- Connection: `INITIALIZING` → `CONNECTING` → `CONNECTED`
- Pre-flight: `DISARMED` → `ARMING` → `ARMED`
- Operation: `TAKING_OFF` → `FLYING` → `RETURNING` → `LANDING` → `LANDED`
- Error: `ERROR`, `FAILSAFE`

`drone_state.py` also defines the `FlightMode` enum (ArduCopter mode numbers: GUIDED=4,
AUTO=3, RTL=6, LAND=9, etc.). State transitions are driven by ArduPilot mode/heartbeat
changes and mission progress.

### Message Types
- ArduPilot DDS messages: `GlobalPosition`, `Heartbeat`, `Altitude`, `LocalPose`
- MAVLink protocol for GCS compatibility
- Note: there is no `msg/` package with custom `.msg` definitions. `DroneState` and
  `FlightMode` are plain Python enums, not ROS2 interface messages.

### Domain Isolation
Each drone operates in its own ROS2 domain (`drone_id + 1`) to prevent cross-talk between
instances.

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
ArduPilot SITL parameters live in `src/swarm_control/parameters/dds_swarm.parm` (flight
modes, speeds, safety, and GPS/DDS settings). At container startup,
`start_drone_container.sh` prepends per-instance `DDS_UDP_PORT`, `SYSID_THISMAV`, and
`DDS_DOMAIN_ID` into `instance_dds.parm`, then appends `dds_swarm.parm`.

FastDDS transport config is separate, in `src/dockerfiles/config/fastdds_profile.xml`.

## Environment Variables and Configuration

Key environment variables passed by `start_swarm.sh` / `compose.yaml` to each instance:
- `INSTANCE`: 0-indexed drone identifier (drives all per-drone port/ID math)
- `NUM_DRONES`: total swarm size
- `SYSID_THISMAV`: MAVLink system ID (`INSTANCE + 1`)
- `ROS_DOMAIN_ID`: ROS2 domain for network isolation (`INSTANCE + 1`)
- `MICRO_ROS_AGENT_PORT`: DDS agent UDP port (`2019 + INSTANCE`)
- `RMW_IMPLEMENTATION`: DDS middleware (`rmw_fastrtps_cpp`)

Port values like SITL/MAVLink/Gazebo are **computed inside `start_drone_container.sh`**
from `INSTANCE`, not passed as environment variables.

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
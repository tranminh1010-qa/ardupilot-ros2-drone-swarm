# ArduPilot ROS2 Swarm — Multi-Drone Field Mapping Simulation

Scalable drone-swarm simulation combining **ArduPilot SITL**, **ROS2 (Humble)**,
**Gazebo (Harmonic)** and **Docker Compose**. Four (or N) simulated quadcopters
autonomously split a field into balanced 1/n strips, sweep them in serpentine
mapping patterns, capture geotagged imagery with down-facing cameras, and the
frames are stitched into a single field map with OpenCV.

This is the simulation platform for a USDA STTR concept: *same-day drone
mapping → on-farm edge processing → targeted spraying*.

## Quick Start

```bash
# One-time: build images
docker compose build

# SITL-only physics (no Gazebo needed)
./start_swarm.sh 4

# With Gazebo physics + visualization + cameras (start Gazebo FIRST)
./start_gazebo.sh &                # GUI (use --headless for server only)
USE_GAZEBO=1 ./start_swarm.sh 4

# After the mission: build the stitched field map from captured frames
python3 src/scripts/stitch_field_map.py --logs ./logs --out field_map.jpg

# Stop everything
./stop_swarm.sh
```

QGroundControl auto-connects to all drones via UDP 14550–14553 (also TCP
`5762 + 10*i`). Port 5600 carries drone 1's H.264 camera stream (QGC default
video port).

## What Works Today

- **4-drone autonomous missions**: arm → GUIDED → takeoff → serpentine survey
  → RTL, fully autonomous via ArduPilot's DDS interface (no MAVLink in the
  control loop)
- **Balanced 1/n field split** (`split_serpentine`): every drone gets a
  contiguous strip of one shared field (common GPS origin), chunk sizes differ
  by at most one waypoint, nothing dropped for any drone count
- **Camera as an extension** (both layers):
  - Gazebo: `base_drone_cam*` models wrap the plain `base_drone*` models with a
    down-facing camera streaming H.264/RTP via GstCameraPlugin on UDP `5600+i`
  - ROS2: `CameraDrone(BaseDrone)` overrides an `on_waypoint_reached` hook to
    capture geotagged frames (+ ArUco detection) per waypoint into
    `./logs/drone_<id>/`
- **OpenCV mapping pipeline**: GStreamer decode → per-waypoint JPEG + JSON
  geotag sidecar → position-based mosaic (`stitch_field_map.py`)
- **Self-healing startup**: nodes respawn through EKF/GPS warmup races;
  arming/mode timeouts sized for multi-SITL contention

## Architecture

```
                       HOST
  Gazebo (server headless + GUI attached separately)
    └─ 4× base_drone_cam models: FDM 9002+10i, camera RTP 5600+i
         ▲ JSON FDM                      │ H.264/RTP
         ▼                               ▼
  ── docker (host network) ──────────────────────────────
  drone-ardu-{i}          ArduPilot SITL (--model JSON)
  micro-ros-agent-{i}     DDS bridge, UDP 2019+i, ROS_DOMAIN_ID i+1
  swarm_controller        launch file → CameraDrone node per drone
  discovery_server        FastRTPS discovery
```

- **Domain isolation**: drone i lives in `ROS_DOMAIN_ID = i+1`; inspect with
  `docker exec swarm_controller bash -c "ROS_DOMAIN_ID=1 ros2 topic list"`
- **Port scheme** (instance i, 0-indexed): MAVLink TCP `5760+10i` (SERIAL1
  `5762+10i` is free for GCS), MAVProxy UDP `14550+i`, DDS `2019+i`, Gazebo
  FDM `9002+10i`, camera `5600+i`

## Key Files

| Path | Purpose |
|---|---|
| `src/swarm_control/scripts/base_drone_dds.py` | Base flight node (DDS state machine, waypoint mission, extension hook) |
| `src/swarm_control/scripts/camera_drone_dds.py` | `CameraDrone` extension: per-waypoint geotagged capture |
| `src/swarm_control/scripts/camera_processor.py` | GStreamer→OpenCV capture, ArUco detection, geotag sidecars |
| `src/swarm_control/scripts/points_distributor.py` | Waypoint generation + balanced serpentine 1/n split |
| `src/swarm_control/launch/compose_decentralized.launch.py` | Per-drone node launch, field origin + camera ports |
| `src/custom_gz/models/base_drone*` | Plain drone models (per-instance FDM port) |
| `src/custom_gz/models/base_drone_cam*` | Camera extension wrappers |
| `src/custom_gz/worlds/swarm_drone.sdf` | Farm world with 4 camera drones |
| `src/scripts/stitch_field_map.py` | Position-based field mosaic from geotagged frames |
| `src/swarm_control/tests/test_points_distributor.py` | Splitter balance/coverage tests |

## Roadmap (toward the STTR architecture)

Gap analysis vs. the project narrative, in build order:

1. **Sprayer drone** — ✅ model + firmware done: `base_drone_sprayer` Gazebo
   extension (tank, boom, particle-emitter plumes on `/sprayer/spray_cmd`) and
   ArduPilot's native `AC_Sprayer` enabled via `sprayer.parm` (T25-class rates,
   pump servo 22, aux 15) when booted with `SPRAYER_INSTANCES="3"`. Remaining:
   a `SprayerDrone(BaseDrone)` node flying prescription plans with tank +
   battery constraints
2. **On-premise processing server** — pull detection out of the drone nodes
   into a containerized microservice: ingest the RTP streams, run AI models,
   map-reduce field segments across workers, store detections in
   Postgres/PostGIS
3. **Weeds in the world** — spawn ground-truth weed patches in the farm world
   (fix `farm_world_generator.py`) so detection rate is measurable
4. **Real detection models** — YOLOv5s segmentation + YOLOv5-tiny
   classification (WeedMap / CoFly-WeedDB transfer learning), weed size from
   GSD; simulated NIR band for EVI
5. **Prescription plans** — detections → georeferenced clusters →
   shapefile/GeoJSON → spray waypoints + rates; detection-threshold sweep
6. **Concurrent task queue** — priority-queue allocator streaming spray tasks
   to sprayer drones *while mapping continues* (the same-day story)
7. **Metrics harness** — detection rate vs ground truth, herbicide-saved %,
   mapping→spray latency, per-service timings (Prometheus/Grafana)
8. **Robustness** — server-failure fallback (drones RTL/loiter), closed-loop
   waypoint arrival before capture/spray, scale test to 14 drones

## Known Gotchas

- Run the Gazebo **server headless** and attach the GUI separately (built into
  `start_gazebo.sh` / `gz sim -g`) — a combined GUI+server process can crash
  under multi-camera rendering on NVIDIA/EGL
- QGroundControl's **virtual joystick must stay disabled** — it streams
  centered throttle to the active vehicle, which then fails arming with
  "Throttle too high"
- GstCameraPlugin streams only after an `enable_streaming` trigger
  (`start_gazebo.sh` does this automatically)
- Verify missions by **actual altitude/position telemetry**, not takeoff ACKs
- `./logs` is written by root inside the container; `sudo chown` it if you
  need to clean it

## Requirements

- Ubuntu 22.04+ host with Docker + Docker Compose
- Gazebo Harmonic (`gz sim` 8.x) on the host with the
  [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) plugin
  built (path configured in `start_gazebo.sh`)
- QGroundControl (optional, ground-station view + video)

# Simulated Weeds + OpenCV Weed Detection — Design

Date: 2026-07-23
Status: approved

## Goal

Add oversized, deliberately unrealistic weeds to the Gazebo farm world so the
drone swarm's camera pipeline has something to detect, and extend the existing
`CameraProcessor` with OpenCV weed detection so detections land in the
per-waypoint geotag sidecars and can be checked against known ground truth.

Weeds are intentionally NOT true to life: large magenta blobs on green/yellow
fields. The point is a reliable, tunable detection target for testing the
pipeline end to end — not photorealism.

## Context (current system)

- Active world: `src/custom_gz/worlds/swarm_drone.sdf` (and the sprayer variant
  `swarm_drone_spray.sdf`, selected via `SWARM_WORLD` in `start_gazebo.sh`).
  Fields are flat colored 30×30 m boxes; ground is green.
- Drones spawn at Gazebo (45,50)–(60,50). The shared field origin
  (`field_origin_lat/lon` = `LAT_BASE/LON_BASE` = drone 0's home) corresponds
  to Gazebo ≈ (45, 50). The mission surveys an 80×80 m grid centered there
  (`generate_grid_waypoints(field_size=80.0, grid_points=5, height=30.0)` in
  `compose_decentralized.launch.py`), split into per-drone serpentine strips.
- Camera: down-facing, 640×480, `horizontal_fov` 1.2 rad, H.264/RTP on UDP
  5600+instance. `CameraProcessor` (`src/swarm_control/scripts/camera_processor.py`)
  captures a geotagged frame per waypoint and already runs ArUco detection;
  everything is non-fatal by contract.

## Component 1: Weed generator

New script `src/custom_gz/worlds/generate_weeds.py` (peer of
`farm_world_generator.py`).

- Output: a single static Gazebo model at `src/custom_gz/models/weed_field/`
  (`model.sdf` + `model.config`) containing N weed links.
- Each weed: a squashed magenta ellipsoid/cylinder, diameter drawn uniformly
  from 1.2–2.5 m, height ~0.2 m, resting just above the field surface
  (z ≈ 0.2). Visual only — **no collision geometry**, so weeds can never
  interfere with flight or landing.
- Material: magenta `1 0 1` diffuse/ambient plus a matching emissive term so
  directional lighting and shadows do not shift the hue out of the detection
  band.
- Placement: seeded RNG (`--seed`, default 42), `--count` (default 12),
  uniform over the surveyed 80×80 m square centered at Gazebo (45, 50)
  (`--center`/`--area` overridable). This guarantees every drone's serpentine
  strip crosses weeds.
- Ground truth: the script also writes
  `src/custom_gz/models/weed_field/ground_truth.json` with each weed's Gazebo
  x/y, diameter, and derived lat/lon (offset from `LAT_BASE/LON_BASE` using
  the same simple meters-per-degree math as the mission code).
- A default generated model (seed 42, count 12) is committed so the sim works
  out of the box; rerun the script only to change the layout.
- Both `swarm_drone.sdf` and `swarm_drone_spray.sdf` gain one
  `<include><uri>model://weed_field</uri></include>`. No other world changes.

## Component 2: Weed detection in CameraProcessor

New private method `_detect_weeds(frame)` alongside `_detect_aruco`, called
from `capture()`:

1. Convert BGR → HSV.
2. `cv2.inRange` on the magenta band (H ≈ 140–170 on OpenCV's 0–179 scale,
   S ≥ 100, V ≥ 80).
3. Morphological open then close (small elliptical kernel) to remove speckle
   and fill holes.
4. `cv2.findContours`; keep contours with area ≥ ~50 px (at 30 m altitude a
   1.2 m weed is ~19 px across ≈ 280 px area, so real weeds clear this easily
   while sensor noise does not).
5. Return `[{"centroid_px": [x, y], "bbox": [x, y, w, h], "area_px": a,
   "lat": .., "lon": ..}]`.

Georeferencing (nadir pinhole approximation): focal length in pixels
`f = (width/2) / tan(hfov/2)`; ground offset east/north from the image center
is `(pixel_offset / f) * altitude_agl`, rotated by 0 (camera assumed
north-aligned nadir — acceptable error at survey speed, noted as a
limitation). Convert meters to lat/lon deltas with the standard
meters-per-degree approximation.

Results are stored in the existing JSON sidecar as `"weed_detections"`,
next to `"aruco_detections"`. Same non-fatal contract: any exception → `[]`,
the mission never breaks.

## Component 3: Verification

- **Unit (no Gazebo):** a small test script draws magenta ellipses on a green
  synthetic background with cv2 and asserts `_detect_weeds` finds them with
  correct count and approximate centroids, and finds nothing on a weed-free
  frame.
- **Live:** `./start_gazebo.sh` + `USE_GAZEBO=1 ./start_swarm.sh 4`, fly the
  mission, then confirm `./logs/drone_*/wp_*.json` sidecars contain
  `weed_detections` and that detected lat/lon cluster near
  `ground_truth.json` positions (within a few meters — georeferencing is
  approximate).

## Research: OpenCV approaches considered

| Approach | Fit for this stage | Notes |
|---|---|---|
| **HSV color thresholding + contours (chosen)** | Excellent | Weeds are a color we fully control; deterministic, fast (~ms/frame), zero training data, trivially tunable. The standard first-stage approach for high-contrast targets. |
| Vegetation indices (ExG, ExG−ExR, NDI) | Later | The real-world method for green-on-green weed/crop separation: compute an index image, Otsu-threshold it. The natural upgrade path when weeds become realistic dark-green clumps. |
| `cv2.SimpleBlobDetector` | Marginal | Convenience wrapper over thresholding + contour filtering; less control over the color mask than doing HSV explicitly. No advantage here. |
| Watershed / superpixel segmentation | Overkill | Helps separate touching plants in dense canopy imagery; our weeds are sparse, isolated blobs. |
| Classical ML (color+texture features → SVM/RF) | Later | Needed once color alone is ambiguous; requires labeled patches. |
| Deep learning (YOLO/segmentation) | Much later | The production-grade answer for real weeds; heavy for a sim smoke test and needs a dataset. The per-waypoint geotagged captures this pipeline produces are exactly how such a dataset would be collected. |

Chosen: HSV thresholding, because the simulation controls the target color —
the detector tests the *pipeline* (capture → detect → geotag → ground-truth
comparison), and each row above is a drop-in replacement for step 2 when
realism increases.

## Out of scope

- Marking weed detections on the stitched field map
  (`stitch_field_map.py` unchanged).
- Realistic weed appearance, camera gimbal/yaw compensation in
  georeferencing, and any ML-based detection.

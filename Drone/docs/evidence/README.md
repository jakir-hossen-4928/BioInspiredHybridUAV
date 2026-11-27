# Validation evidence

Every directory here was produced by a live Webots run, not by hand. The
commands below regenerate each one. All navigation poses come from
`/lio_sam/odometry/imu`; `/webots/ground_truth` is logged only for scoring.

## Layout

| Directory | What it holds |
|---|---|
| `validated-flight/calibration-v2/` | Physics-only IMU excitation. `sensor.csv` (5000 samples), `native-scan.json` (ring/timestamp audit), `trajectory.json` (ATE/RPE), `trajectory.svg`. |
| `flight-matrix-v3/` | Nine arena cells: `camera`/`lidar`/`combined` × `static`/`moving`/`multiple`. Each has `result.json`, `track.csv`, `ros.jsonl`, `trajectory.json`, `trajectory.svg`, `map.svg`, `routes.svg`, `replan.png`, camera frames. |
| `farm-mission/` | `mission.json`, `farm_flight.wbt` generator output, `run-8` (contact), `run-9` (planner deadlock) with the same per-run files. |
| `flight-matrix-v2/`, `validated-flight/calibration/` | Earlier runs kept for comparison. Superseded by v3/calibration-v2. |

## What each file is

- `ros.jsonl` — one JSON array per line from `Drone/validation/observe_flight.py`:
  `["odom"|"truth"|"nav"|"map"|"scan", sim_time, payload]`. `nav` is what the
  controller read; `truth` is the supervisor GPS.
- `track.csv` — per-0.2 s truth pose, the odometry pose the controller used,
  and the instantaneous body clearance.
- `result.json` — mode, scenario, reason, replan events (with the measured
  range and stereo disparity), observed detections, minimum clearance,
  contact steps, goal error, `passed`.
- `trajectory.json` — SE3-aligned ATE, RPE (1 s pairs), final error, drift.
- `trajectory.svg` / `map.svg` / `routes.svg` — offline renders of the same
  data (odometry translated to the launch pose; no fitting).

## Reproduce

```powershell
# Sensor calibration (IMU torque, native LiDAR timestamps, rings, extrinsics)
python Drone/validation/run_inertial_trial.py --container uav-liosam-nov27 `
  --output Drone/docs/evidence/validated-flight/calibration-v2 --mode calibration

# One matrix cell
python Drone/validation/run_inertial_trial.py --container uav-liosam-nov27 `
  --output Drone/docs/evidence/flight-matrix-v3/camera-static `
  --mode camera --scenario static

# Farm world mission
python Drone/validation/farm_mission.py --output Drone/docs/evidence/farm-mission
python Drone/validation/run_inertial_trial.py --container uav-liosam-nov27 `
  --output Drone/docs/evidence/farm-mission/run-9 --mode combined --scenario static `
  --world "$PWD/Drone/worlds/farm_flight.wbt" `
  --mission Drone/docs/evidence/farm-mission/mission.json `
  --gravity zero-world --timeout 1200

# Scoring and rendering
python Drone/validation/evaluate_trajectory.py <run>/ros.jsonl --json <run>/trajectory.json
python Drone/validation/render_evidence.py <run>
```

`--output` must not already exist; the runner refuses to overwrite evidence.

## Container

`uav-liosam-nov27` is the prepared ROS 2 Humble + LIO-SAM image (built from
`Drone/integration/ros2/Dockerfile.lio-sam`). The runner restarts it before
every trial so the ROS clock starts from zero, copies the bridge source in,
rebuilds `drone_ros_bridge`, and launches
`drone_ros_bridge/lio_sam.launch.py`.

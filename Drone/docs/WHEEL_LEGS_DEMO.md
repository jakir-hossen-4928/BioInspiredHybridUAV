# Hybrid drone with four wheel-equipped legs (Webots R2025a)

This update adds four off-road tires at the tips of the four articulated legs. The
original walking and flight worlds, original survey cameras, LiDAR/IMU and ROS 2
bridge remain intact.

## Launch on Windows

1. Extract `Drone_Wheel_Legs_Update.zip` into your **existing** Drone project folder.
   Merge the inner `Drone/` directory with your existing `Drone/` directory. Do
   not delete the existing `textures/` folder containing the 4K farm textures.
2. Open `hybrid_wheels.bat`, or open `worlds/hybrid_wheels.wbt` in Webots R2025a.
3. Click **Reset** then **Play**. The demonstration starts at the ground pad,
   rolls autonomously on four wheels, holds for a camera photo, then takes off
   into the original aerial survey.
4. Use `hybrid_wheels_ros2.bat` only if you also want to stream LiDAR/IMU/camera
   data via the ROS 2 bridge. ROS 2 Humble and external FAST-LIVO2 still need
   to be installed, configured and tested separately on Ubuntu.

## Controls

| Key | Action |
| --- | --- |
| `T` | Request wheel drive. From flight requires the existing depth-based SAFE terrain check |
| `W` | Change to leg-walking mode, retaining the same ground path when available |
| `I` | Return to aerial flight |
| `D` | Toggle ultrasonic obstacle sensing and ground/flight replanning |
| `R` | Replan current route (base controller) |
| `Y` | Terrain assessment |
| `C` / `B` | Save downward/front camera image |

Click inside the Webots 3D view before pressing keyboard shortcuts. The status
label says `ROLLING`, `WALKING`, `TO_FLIGHT` or `FLYING`.

## Files added / updated

- `worlds/hybrid_wheels.wbt`: four wheel attachments for the flight/walk demo.
- `worlds/hybrid_wheels_ros2.wbt`: the same demo with the original ROS sensor bridge.
- `controllers/wheels/wheels.py`: autonomous wheel-drive state, wheel rotation,
  heading-aligned route following, safety-gated mode changes and telemetry.
- `controllers/hybrid/hybrid.py`: accepts `ROLLING` in ultrasonic obstacle checks.
- `controllers/drone/settings.json`: wheel radius, ground speed and ride-height knobs.
- `tools/add_wheels_worlds.py`: recreates wheel worlds from the untouched base worlds.
- `hybrid_wheels.bat`, `hybrid_wheels_ros2.bat`: Windows launchers.
- `tests/test_wheels.py`: offline controller and world tests.

## Simulation limits (read before demonstrating)

- The four wheel tires and hubs are **rendered and rotate**, but the movement
  of the drone body is **Supervisor-kinematic**. This is **not** a wheel motor,
  contact/friction dynamics, torque or traction validation.
- Ground routing uses the existing heading-aware walking A* states in
  `hybrid_planner.py`; the `ROLLING` behavior is a controller execution mode,
  not a separately optimized full 3-mode State-Augmented Hybrid A* research method.
- The four `walk_force_*` TouchSensors remain available. This zero-gravity,
  position-resetting demo cannot validate real tire ground reaction forces.
  `estimated_support_n` is a calculation, not a force-sensor measurement.
- The simulator does not prove FAST-LIVO2 SLAM is running. For that, build the
  external estimator in Ubuntu ROS 2, calibrate, and verify live poses/maps.
- Offline unit tests check code, world structure and simulated transitions;
  actual Webots R2025a GUI loading, rolling visuals, ROS integration and
  contact physics must be tested on your PC.

## Telemetry

`captures/<session>/hybrid_status.json` is saved every approximately 2
simulation seconds. When rolling, it includes mode `ROLL` and the four wheel
rotation angles plus cumulative ground travel distance.

## Recommended screen recording sequence

Start `hybrid_wheels.wbt`; record 10-20 seconds of autonomous wheel travel,
press `W` for walking, press `T` for wheel drive, press `I` for takeoff, show
ultrasonic replanning with `D` on, then display the 4K survey view. Do not call
this demo physically validated wheel control, original FAST-LIVO2 localization,
or a research-equivalent Hybrid A* implementation.

# Drone flight and walking mode - Webots R2025a

## Software and files

Use **`worlds/hybrid.wbt`** to demonstrate both motion modes. It is a separate world and does not modify the original `farm.wbt`. Do not open the older `farm.wbt` to test walking; that world still uses the flight-only controller. The 4K farm textures must be installed in the existing `Drone/textures/` folder.

The Webots Robot named `Drone` uses controller `controllers/hybrid/hybrid.py`. That controller imports the existing flight mission in `controllers/drone/drone.py`, so both directories must remain together.

### Start on Windows

1. Keep your existing `Drone` project with 14 textures.
2. Merge the upgrade's `Drone/` folder into the existing `Drone/` folder without deleting textures.
3. Open `Drone/worlds/hybrid.wbt` in Webots R2025a, or double-click `Drone/hybrid.bat`.
4. Reset and then Play. The drone first **walks automatically** from the designated home pad to a short safe goal and then takes off to continue the normal survey. The initial home pad is preselected as safe, not evaluated by FAST-LIVO2.
5. While flying over a flat area, press `W` to request a sensor-screened flight-to-walk transition. Press `I` to take off again. Press `D` to toggle live obstacle-based route replanning. Click inside the 3D window first so it receives keyboard focus.
6. To see positions, modes, measured raw contact-force vectors and analytical force estimates, inspect `Drone/captures/<session>/hybrid_status.json`. The file is refreshed about every 2 simulation seconds.

Existing controls are unchanged (M mission, O manual obstacle, R route replan, Y ground scan, Z guarded landing, C/B camera photos, U manual waypoint reload).

### What the navigation code actually does

* `controllers/drone/hybrid_planner.py`: search with state `(x_cell, y_cell, heading_bin, mode)`, where `mode` is FLIGHT or WALK. Eight heading bins represent steering limits. Mode-switch edges are allowed only at explicitly approved grid locations. Walking is slower and heading constrained. This is a **reference state-augmented A* prototype**, **not a tested reproduction** of any particular paper's continuous-curvature State-Augmented Hybrid A*.
* `controllers/hybrid/hybrid.py`: Supervisor-based motion state machine `TO_WALK -> WALKING -> WALK_HOLD -> TO_FLIGHT -> FLYING`; safe aerial-to-ground transitions require the depth-based SAFE classification. Ground walking follows planned heading states and animates four upper and lower leg pieces with a diagonal trot.
* `controllers/drone/hybrid_support.py`: gait generation, distance-to-XY sensor projection and analytical weight-sharing estimates.
* `worlds/hybrid.wbt`: four visually animated legs plus four `TouchSensor type "force-3d"` devices.
* `integration/ros2/`: previously implemented topic bridge for LiDAR, IMU and camera. The original FAST-LIVO2 SLAM estimator is a separate external application.

### Safety and validity notes

- Flight and walking translate the robot pose with Webots **Supervisor**, not realistic thrust, gait torque or body dynamics. This project's world has `gravity 0` and moving the robot resets its physics; realistic Newton contact forces cannot be inferred from its kinematic gait.
- `walk_force_*` use direct Newton units in the farm worlds. The gravity-free, kinematic farm model gives them no representative load. `estimated_support_n` is the analytical reference `mass * 9.81 / 4`. Use the separate [foot-force validation world](../validation/worlds/foot_force_validation.wbt) for physics-driven drop, walking and per-foot reference-load tests.
- Ultrasonic sensor `ultrasonic_front` can stop travel below 1.8 m or add a conservatively projected obstacle for replanning farther away. It does not run a full sensor-fusion occupancy mapper.
- Generic Webots 3D LiDAR / IMU / camera are streamed through the bridge; this is **not** validated Livox Mid-360 hardware emulation.
- The external FAST-LIVO2 process published estimates and registered point clouds. Position accuracy failed the comparison with Webots ground truth, and quantitative map accuracy remains open. See [FAST-LIVO2 validation](FAST_LIVO2_VALIDATION.md).

### Validation tasks for the Windows laptop

1. Confirm `hybrid.wbt` loads without red errors and `4/4` leg and force nodes are detected.
2. Verify automatic walking, four legs moving, and return to flight.
3. Test `W` over a safe flat pad and `W` over unsuitable ground; note accepted and rejected transitions.
4. Put a front obstacle within sensor detection range and compare replan/pause response.
5. Read raw `walk_force_*` output in `hybrid_status.json`; label any force result carefully.
6. Save actual screenshots using Webots' camera/screenshot toolbar. Existing included screenshots predate the new hybrid world and must not be labeled as new results.
7. Start Ubuntu ROS2 bridge and verify topics; only call FAST-LIVO2 integrated after estimator publishes actual odometry and map.

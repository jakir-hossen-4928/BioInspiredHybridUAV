# UAV integration and acceptance status

## Physics-driven flight and perception

See the [v5 acceptance evidence](evidence/flight-matrix-v5/README.md) for the
simulation criteria, results by run, and exact source revisions.

The main wheel controller's LIO-SAM flight entry point uses fused
LiDAR–IMU odometry and rectified stereo for camera avoidance. RGB supplies
a red-object gate for the stereo matcher. The arena and farm simulations
start airborne and apply net forces in Webots under 9.81 m/s² gravity.

| Acceptance item | Measured outcome |
|---|---|
| Arena avoidance, all nine scenarios repeated | 18/18 passed at `71f3199`. All flights recorded zero contacts. Minimum clearance was 0.973 m and maximum goal error was 0.094 m. |
| Camera-only avoidance of a moving obstacle | Both flights passed. Minimum body clearances were 1.663 m and 1.679 m, goal errors were 0.049 m and 0.054 m, and the runs recorded 22 and 19 replans. |
| Quantitative 3D map accuracy in the static arena | Last-cloud point-to-surface RMSE was 0.183 m and 0.111 m on the two repeats. The corresponding 95th-percentile errors were 0.128 m and 0.113 m. [RViz capture](evidence/flight-matrix-v5/71f3199-repeat-1/camera-static/rviz.png) |
| Farm traverse with denser LiDAR and LIO-SAM | The controller reached the estimated goal with zero contacts and 25 replans. Measured goal error was 0.803 m, above the 0.35 m limit. [Measured result](evidence/flight-matrix-v5/farm-dense/run-2/result.json) |
| Farm traverse with LIO-SAM and onboard altitude fusion | The controller reached the estimated goal with zero contacts. Measured clearance was 1.206 m and goal error was 0.425 m, above the 0.35 m limit. [Measured result](evidence/flight-matrix-v5/farm-altimeter/run-1/result.json) |
| Automated regression | 89/89 passed at `5938be1`, including all 82 baseline tests. |

## Prior FAST-LIVO2 integration

| Item | Evidence/status |
|---|---|
| Webots 3D LiDAR data export | Implemented as actual `Lidar.getPointCloud()` frames; exported on TCP, optionally ROS2 PointCloud2 and Livox message format |
| Webots IMU, gyro, accelerometer export | Implemented as live simulation measurements, published to `/livox/imu` |
| Webots camera export | Implemented; 640x480 LVI camera, original 4K survey cameras unaffected |
| ROS 2 receiving bridge | Built and ran in ROS 2 Humble on Kiri. The live run delivered Webots LiDAR, IMU, RGB and reference pose messages. |
| FAST-LIVO2 original estimator | Not bundled; requires separate third-party source, ROS2 Humble build and calibration |
| FAST-LIVO2 pose and cloud outputs | Observed upstream odometry, path and registered point-cloud messages. Pose accuracy failed and metric map accuracy remains open. |
| Livox Mid-360 physical scan timing/pattern | Not simulated; Webots LiDAR is generic; synthetic Livox message structure only |
| Sensor extrinsic calibration | Not validated; Webots CAD mount offsets documented |
| End-to-end trajectory benchmark | Rigid alignment and comparison against the Webots reference are recorded in [FAST-LIVO2 validation](FAST_LIVO2_VALIDATION.md). |
| Existing A*, sensor dashboard, terrain landing assessment | Existing functions preserved, not driven by FAST-LIVO2 estimate |

The original kinematic farm demos use Webots reference pose. Their FAST-LIVO2
integration produced estimator outputs with position disagreement too
large for navigation.

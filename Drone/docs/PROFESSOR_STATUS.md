# Integration status for the project report

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

**Summary for report:** The simulator streams LiDAR, IMU and camera measurements to the third-party FAST-LIVO2 ROS 2 process. It produces estimator outputs, with position disagreement too large for navigation. The farm missions use Webots reference pose.

# Webots sensor geometry / calibration notes

For the specific `livo2.wbt` world, the robot uses a Webots `ENU` world (`Z` vertical). Position offsets below are **world-model declarations, not experimentally calibrated extrinsics**.

| Device name | Robot-local translation in meters | Notes |
|---|---|---|
| `lidar` | `[0, 0, 0.555]` | 24-layer generic 360 deg scan; vertical FOV 1.0 rad |
| `slam_camera` | `[0.69, 0, -0.26565]` | 640x480 RGB / horizontal FOV 1.05 rad |
| `camera` | `[0.69, 0, -0.26565]` | Independent 3840x2160 front RGB photo camera |
| `survey_camera` | `[0.135, 0, -0.300]` | Independent 3840x2160 downward RGB photo camera; rotation 0 1 0 1.570796... |
| `inertial unit` | `[0, 0, 0]` | Node inherits robot body pose |
| `gyro` | `[0, 0, 0]` | Node inherits robot body pose |
| `accelerometer` | `[0, 0, 0]` | Node inherits robot body pose |

Approximate lidar to SLAM camera translation in robot coordinates = `[+0.69, 0, -0.82065]` meters. **This is not a complete LiDAR-to-camera extrinsic matrix**: camera optical frame orientation and Webots LiDAR point axes must be verified and converted to the ROS optical-frame convention. Do not use identity `Rcl` unless independently established.

The receiver computes a pinhole-camera intrinsics estimate from horizontal field of view:

`fx = width / (2*tan(1.05/2))`, `fy = fx`, `cx=(width-1)/2`, `cy=(height-1)/2`

The simulated camera has ideal zero distortion. The actual FAST-LIVO2 parameter file may use an alternate camera model or require a camera YAML. The correct extrinsic quaternion/matrix must be measured or derived and validated in RViz before experiments.

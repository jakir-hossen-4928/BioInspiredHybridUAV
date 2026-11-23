# Drone | Sensor and Algorithm Status

| Requested component | Current implementation | What actually works |
|---|---|---|
| Front RGB camera | Yes | Simulated camera image, JPG/PNG save |
| Down RGB camera | Yes | Simulated camera images at survey stops |
| 3D LiDAR | Partial | Generic Webots LiDAR range scan; nearest range and valid count |
| Exact Livox Mid-360 | No | Different scan pattern and timing; driver/custom PROTO not implemented |
| IMU | Yes | Webots InertialUnit/Gyro/Accelerometer, orientation telemetry |
| GPS / GNSS | Simulated GPS only | Webots local XYZ coordinates, not true GNSS receiver modeling |
| Barometer | Approximation only | ISA pressure derived from simulated altimeter, NOT pressure sensor |
| Ultrasonic | Yes | Webots front/down DistanceSensor raw readings |
| Foot contact | Yes | 4 bumper contact sensors; proper physical contact not GUI-validated |
| Foot force | No | Physical force in Newtons NOT implemented under current kinematic model |
| Localization | Partial | Known simulated GPS pose. No visual-inertial localization |
| FAST-LIVO2 | Interface only | Webots sensor stream and ROS2 message publisher supplied; estimator, calibrated fusion, map and trajectory not yet verified |
| Mapping | Partial | Small observed ground-height grid from depth plus known GPS, NOT SLAM |
| Terrain perception | Yes (educational heuristic) | Depth samples, measured slope, RMS roughness, height classification |
| Landing assessment | Yes (educational heuristic) | SAFE/UNSAFE/NO_DATA from measured depth, rejects Z for unsafe/unknown |
| Autonomous navigation | Yes (simplified) | Known-map 2D A* + Python Supervisor movement |
| State-Augmented Hybrid A* | No | Standard 2D A*, no vehicle-state search |

The landing check assumes the drone's sensor is vertical (roll/pitch near zero) and the farm ground reference height is Z=0. Thresholds need tuning for different scenes. The code cannot guarantee real-world safe landing.

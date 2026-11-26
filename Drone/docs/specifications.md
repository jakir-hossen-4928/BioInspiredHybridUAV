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
| Foot contact | Device response verified | All four farm bumpers switched on and released during a controlled height probe. They stayed off during the normal walking and rolling missions. |
| Foot force | Validated in the separate Webots rig | Four three-axis force sensors in Newtons passed drop, standing, motor-driven walking and signed reference loads applied to each foot. The farm demo remains kinematic. |
| Localization | Partial | Navigation uses the known simulated pose. The external visual–LiDAR–inertial estimate does not meet the localization objective. |
| FAST-LIVO2 | Runs with external estimator | ROS 2 inputs and external odometry, path and registered point-cloud outputs were observed. Localization accuracy failed and map accuracy was unverified. |
| Mapping | Partial | Small observed ground-height grid from depth plus known GPS, NOT SLAM |
| Terrain perception | Yes (educational heuristic) | Depth samples, measured slope, RMS roughness, height classification |
| Landing assessment | Yes (educational heuristic) | SAFE/UNSAFE/NO_DATA from measured depth, rejects Z for unsafe/unknown |
| Autonomous navigation | Yes (simplified) | Known-map 2D A* + Python Supervisor movement |
| State-Augmented Hybrid A* | Implemented for defined transitions | The planner handles flight and walking states through screened descent and uses planar obstacle clearance with fixed mode heights. |

The landing check assumes the drone's sensor is vertical (roll/pitch near zero) and the farm ground reference height is Z=0. Thresholds need tuning for different scenes. The code cannot guarantee real-world safe landing.

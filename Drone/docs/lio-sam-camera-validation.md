# LIO-SAM and forward obstacle avoidance in Webots

The wheel worlds model the standard-FOV OAK-D Pro with a simulated 1280 × 800 OV9782 RGB camera and an 80° forward depth image that represents the output of the OV9282 stereo pair. Airborne avoidance uses the depth image and has an independent LiDAR input. The Webots depth image represents ideal optical depth. Physical stereo correspondence, IR performance, and the RGB image-processing pipeline remain outside this test. [Luxonis OAK-D Pro specifications](https://docs.luxonis.com/hardware/products/OAK-D%20Pro)

An obstacle detected in the forward corridor is projected onto the XY map using the simulation pose. If its inflated footprint blocks the active route, the existing A* planner attempts a detour to the same waypoint. The simulator pauses for a close obstacle or a failed route. This is a forward corridor check with a fixed obstacle footprint and planar routing.

## Run the obstacle tests

From the repository root, run the following in PowerShell with Webots R2025a installed.

```powershell
.\Drone\validation\run_flight_avoidance.ps1
```

The script starts two separate runs in an isolated copy of the wheel world. Each inserts a 1.2 m × 1.2 m box across the planned flight segment without adding it to the map. It checks for a sensor-triggered route change, arrival at the original goal and at least 1.0 m horizontal clearance from the box footprint. Clearance is measured from the vehicle origin. The script prints the evidence directory and writes one JSON result per sensor. It saves a simulated OV9782 RGB frame at the camera-triggered replan.

## Run LIO-SAM

The ROS 2 Humble launch consumes `/webots/lidar/points` and `/livox/imu`. The fixed 360 × 24 LiDAR returns an instantaneous scan with zero per-point time offsets. The bridge restores specific force due to gravity for the zero-gravity world. GPS samples are published as evaluation reference poses. The flight controller continues using the Webots Supervisor pose. [Upstream ROS 2 LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ff4aff79d628411883120109905278d7460caca0)

Run the following from the repository root. These commands use the existing Kiri Docker context. The base image build is needed on a fresh workstation. Close any other receiver using port `15882` first.

```powershell
docker --context kiri build -t uav-fast-livo2-check https://github.com/U-AMC/FAST-LIVO2-ROS2.git#a09599a288c56b832114d7ad9e898248a4111d9e
docker --context kiri build -f Drone/integration/ros2/Dockerfile.lio-sam -t uav-lio-sam:humble Drone/integration/ros2
docker --context kiri run -d --name uav-lio-sam -p 127.0.0.1:15882:15882 uav-lio-sam:humble sleep infinity
docker --context kiri cp Drone/ros2_ws/src uav-lio-sam:/tmp/bridge-src
docker --context kiri exec -it uav-lio-sam bash -lc 'source /opt/ros/humble/setup.bash && source /lio_sam_ws/install/setup.bash && mkdir -p /tmp/bridge-ws && cd /tmp/bridge-ws && colcon build --base-paths /tmp/bridge-src --packages-select drone_ros_bridge && source install/setup.bash && ros2 launch drone_ros_bridge lio_sam.launch.py host:=0.0.0.0'
```

After the receiver reports `Listening for Webots`, run `.\Drone\RUN_ROS2_DEMO.bat` in another PowerShell window and click **Play / Real-time**. In the container, use `ros2 topic hz /lio_sam/mapping/odometry` and `ros2 topic hz /lio_sam/mapping/map_global` after sourcing `/opt/ros/humble/setup.bash`. Restart the LIO-SAM launch after a Webots clock reset. Stop the test container with `docker --context kiri stop uav-lio-sam`.

## Webots R2025a results, 27 November 2026

| Check | Observed result |
| --- | --- |
| OAK depth avoidance, LiDAR avoidance input disabled | A* changed the route and reached the original goal. Minimum horizontal clearance was 1.669 m. |
| LiDAR avoidance, OAK depth avoidance input disabled | A* changed the route and reached the original goal. Minimum horizontal clearance was 1.656 m. |
| Full wheel-world farm mission with the LIO-SAM bridge | The controller landed in 194.616 simulated seconds, about 0.10 m horizontally from home. The stream sent 26,912 packets and reported zero queue drops. The isolated run disabled aerial photo saving and route overlays. |
| LIO-SAM localization | The estimator published 1,214 mapping odometry samples. After interpolation of the reference poses and rigid alignment with no scale adjustment, position RMSE was 2,041.79 m and maximum position error was 3,695.53 m against a 436.12 m reference path. |
| LIO-SAM mapping | The observer received 27 global map clouds, with up to 26,595 points and 22,325 points in the last received cloud. Metric map accuracy remains unverified. |

The estimator reported insufficient scan features and repeatedly reset IMU preintegration after large velocity estimates. A streamed IMU sample on the kinematic mission showed zero angular velocity and gravity-only acceleration. The current mission writes the pose and resets physics each step, which limits the realism of those inertial readings. Reliable localization remains open. A physics-driven motion model with validated IMU output is the next step before evaluating estimator accuracy again. Physical OAK-D Pro hardware was unavailable for this run.

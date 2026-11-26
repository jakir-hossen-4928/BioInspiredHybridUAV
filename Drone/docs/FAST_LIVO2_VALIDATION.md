# FAST-LIVO2 transport and trajectory validation

## 26 November 2026 full Webots mission

The external U-AMC FAST-LIVO2 ROS 2 build ran in a Kiri Docker container with ROS 2 Humble, receiving the live `hybrid_wheels_ros2.wbt` stream. The bridge reported LiDAR, IMU, and camera traffic, and a separate ROS subscriber observed both point-cloud formats, 640 × 480 RGB, camera intrinsics and ground truth. It received **624 odometry messages**, **624 path messages** and **1,248 registered point-cloud messages**. The ROS 2 wheel mission reached `LANDED` at **196.024 s** with **27,107 TCP packets sent** and zero reported drops. The mission used Webots reference pose for navigation.

**Localization result: failed.** The comparison used 624 unique odometry samples with interpolated Webots reference positions at the same simulation timestamps, spanning 1.048 to 200.408 s. The nearest original reference sample was at most 0.128 s away. A least-squares rigid transform aligned the two frames by rotation and translation with scale fixed to one. Position RMSE was **7,654.23 m**, median error **5,017.16 m**, and maximum error **22,874.97 m**. Registered cloud publication confirms estimator activity. Metric map accuracy remains open.

[Mission and topic results](evidence/delivery_2026-11-26.json) · [Timestamped trajectory pairs, alignment and per-sample errors](evidence/fast_livo2_delivery_trajectory.csv)

In the verified startup sequence, the `parameter_blackboard` started first. The test queried `/parameter_blackboard cam_model` before starting the bridge and mapper. A parallel startup attempt returned `Camera model not correctly specified`. In a fresh container with the README's ROS 2 Humble setup and generated parameters, the following are the commands for the ordered sequence. Run the first, third and fourth commands in separate shells with the same ROS workspaces sourced.

```bash
ros2 run demo_nodes_cpp parameter_blackboard --ros-args --params-file /tmp/farm-config/camera_webots.yaml
ros2 param get /parameter_blackboard cam_model
ros2 run drone_ros_bridge webots_receiver --add-sim-gravity --host 0.0.0.0 --port 15882
ros2 run fast_livo fastlivo_mapping --ros-args -r __node:=laserMapping --params-file /tmp/farm-config/mid360_webots.yaml
```

Start Webots after the bridge is listening. Keep the published TCP port local to the host. Restart the workspace processes before replaying a world from time zero.

## Additional checks

External project: https://github.com/U-AMC/FAST-LIVO2-ROS2
Original algorithm: https://github.com/hku-mars/FAST-LIVO2

Use Ubuntu 22.04 + ROS2 Humble, which the referenced Mid-360 ROS2 port documents. Install the port and `livox_ros_driver2` in a ROS2 workspace. In one shell, run `integration/ros2/bridge_receiver.py`, and in another start `hybrid.wbt` in Webots. Check:

```
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /webots/slam/image_raw
ros2 topic echo /webots/slam/camera_info --once
```

After adjusting a copy of the fork's YAML to use `/livox/lidar`, `/livox/imu` and `/webots/slam/image_raw` and supplying real sensor extrinsics, run the fork's `mapping_mid360_lvi.launch.py`. Confirm estimator outputs such as `/aft_mapped_to_init`, `/path` and `/cloud_registered` with recorded timestamps and compare with `/webots/ground_truth`.

The bridge transports measurements to the external estimator. The generic Webots LiDAR does not reproduce the Mid-360 non-repeating scan pattern or the physical unit's reflectivity and point timing. Calibrated extrinsics, metric map accuracy and physically representative IMU data remain open.

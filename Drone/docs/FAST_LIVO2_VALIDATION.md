# FAST-LIVO2 validation procedure (not completed)

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

Until `ros2 topic echo` shows actual estimated poses and registered cloud, **do not claim FAST-LIVO2 was integrated**. The bridge is a sensor transport, not an estimator. The generic Webots LiDAR does not reproduce Mid-360 non-repeating scanning or true reflectivity timing. A public Mid-360 bag can be a separate alternative for demonstrating the estimator, with provenance clearly labeled.

# Drone with ROS 2

Open `worlds/hybrid_ros2.wbt` in Webots on Windows (or `hybrid_ros2.bat`). For the ROS 2 connection, use Ubuntu 22.04 + ROS 2 Humble on WSL2 and follow `docs/ROS2_CONNECTION.md`.

What is included: ROS 2 `drone_ros_bridge` package, Webots 8-ms world, LiDAR/IMU/RGB publishers, optional FAST-LIVO2 launcher, calibration-file generator, source, and tests.

What is still unverified: actual Ubuntu build, FAST-LIVO2 estimator, calibrated LiDAR/IMU/camera frames, 3D mapping, accurate foot-force physics. Full 4K textures stay in your existing `Drone/textures/` folder.

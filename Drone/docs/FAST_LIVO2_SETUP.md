# FAST-LIVO2 sensor interface: Windows Webots + Linux ROS 2

## Implementation status

**Implemented in this package:** Webots LiDAR point-cloud, IMU, front RGB camera and GPS/ground-truth stream, TCP transfer, ROS 2 topic publishing, Livox-compatible `CustomMsg` *message format*, simulated timestamps, and optional Windows stream monitor.

**Not verified or implemented in this package:** the FAST-LIVO2 estimator source/build, converged trajectory or 3D map, physically realistic non-repetitive Livox Mid-360 scan pattern, authentic LiDAR reflectivity and per-point firing timestamps, calibrated camera-LiDAR-IMU extrinsics, physically realistic IMU dynamics, and improved flight dynamics. The unchanged farm navigation uses known-map A*, not FAST-LIVO2 output.

### Why these distinctions matter

The **original** HKU MARS FAST-LIVO2 project is a C++ ROS 1 / Ubuntu application, released under GPLv2. A third-party ROS 2 Humble port supports Mid-360. This project deliberately does not republish or misrepresent its source code as our own. Original repo: https://github.com/hku-mars/FAST-LIVO2 and ROS2 port: https://github.com/U-AMC/FAST-LIVO2-ROS2 (see its links to the actual fork). 

## Step 1. Confirm Webots sensor stream on Windows first (no ROS installation)

1. Your existing 4K `Drone` folder must have `textures/` and `worlds/farm.wbt`.
2. Apply the upgrade via `install.bat` in the patch ZIP.
3. Double-click `Drone/integration/monitor.bat` to open the live sensor receiver.
4. Double-click `Drone/livo2.bat` or open `Drone/worlds/livo2.wbt` in Webots R2025a. For optional 125 Hz nominal IMU sampling, choose `worlds/livo2_fast.wbt` (8 ms basic time step); this may run significantly slower with 4K textures and sensors.
5. Press **Play**. The console should say `[LIVO STREAM] Connected ...` and the receiver should report IMU, LiDAR, camera and GPS frames. The monitor records one `integration/recordings/lidar_sample.ply` and one `camera_sample.bmp`.
6. Stop the monitor with Ctrl+C before using ROS 2, because only one TCP receiver may use port `15882`.

If the monitor connects and receives packets, **transport is verified**. This does **not** verify FAST-LIVO2 localization.

## Step 2. Prepare ROS 2 Humble on Ubuntu 22.04

For a Windows 11 machine, WSL2 Ubuntu 22.04 or a dedicated Ubuntu 22.04 installation is recommended. Follow the official ROS2 Humble setup: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

In your Ubuntu shell, after installation:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-numpy python3-colcon-common-extensions git
```

The ROS2 `bridge_receiver.py` program needs `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `rosgraph_msgs`, and the `livox_ros_driver2` message package if you want `/livox/lidar`. If the Livox message package is absent, only `/webots/lidar/points` is published; FAST-LIVO2's Mid-360 configuration will **not** receive the required `CustomMsg` data.

### WSL2 network address

Run the ROS receiver in WSL2 with `--host 0.0.0.0`. The default Webots setting is `127.0.0.1:15882`. This works with WSL2 mirrored networking, and may work depending on your forwarding configuration. On WSL2 NAT networking, run `wsl hostname -I` in Windows PowerShell to identify the WSL2 IPv4 address. Set `fast_livo_host` in `Drone/controllers/drone/settings.json` to that IP and restart Webots. Never forward this unauthenticated debug port to the public Internet.

## Step 3. Start the ROS 2 receiving bridge

Copy the `Drone/integration/ros2` folder into Ubuntu (or access it through `/mnt/c/...`). Then:

```bash
source /opt/ros/humble/setup.bash
source ~/fast_ws/install/setup.bash  # if you have built livox_ros_driver2
cd /path/to/Drone/integration/ros2
python3 bridge_receiver.py --host 0.0.0.0 --port 15882
```

Start Webots `livo2.wbt` on Windows and press Play. In a **separate** Ubuntu terminal (with ROS sourced), check:

```bash
ros2 topic list
ros2 topic hz /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /webots/slam/image_raw
ros2 topic echo /webots/slam/camera_info --once
```

Published topics:

| Topic | ROS 2 message | Meaning |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Generic Webots cloud converted to Livox *message structure*; not authentic Mid-360 sampling |
| `/webots/lidar/points` | `sensor_msgs/msg/PointCloud2` | Same simulated range points |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Webots IMU+gyro+accelerometer |
| `/webots/slam/image_raw` | `sensor_msgs/msg/Image` | Live 640x480 camera (4K survey cameras retained independently) |
| `/webots/slam/camera_info` | `sensor_msgs/msg/CameraInfo` | Pinhole intrinsics inferred from Webots FOV |
| `/webots/ground_truth` | `geometry_msgs/msg/PoseStamped` | GPS/supervisor-derived comparison only, never an odometry estimator |
| `/clock` | `rosgraph_msgs/msg/Clock` | Webots simulation time |

Re-start the FAST-LIVO2 process whenever Webots world time resets, because some SLAM estimators cannot handle backwards timestamps.

## Step 4. Install and test actual FAST-LIVO2

Read the complete documentation of the selected third-party ROS2 port before compiling: https://github.com/U-AMC/FAST-LIVO2-ROS2

Its maintained README provides Ubuntu 22.04 / Humble dependencies, a self-contained Docker option, and `mapping_mid360_lvi.launch.py` for LiDAR-visual-inertial mode. `setup_fast_livo2.sh` in this folder only fetches upstream source into `~/fast_ws/src/`; it does **not** install OS dependencies or guarantee a successful build. After dependencies are installed, the fork documents `colcon build --symlink-install`.

Run the bridge first, and set the fork's configuration to use these topics (make a copy of the original YAML):

```yaml
common:
  lid_topic: /livox/lidar
  imu_topic: /livox/imu
  img_topic: /webots/slam/image_raw
  img_en: 1
  lidar_en: 1
```

The fork's YAML nesting may vary by commit. Check the specific file (`config/mid360_lvi.yaml`) and validate its existing structure rather than blindly pasting these keys. Use the Livox custom-message preprocessing path; **do not** claim simulated Webots LiDAR is a real Livox Mid-360 sensor.

An example launch from that fork after a successful build and calibration is:

```bash
source ~/fast_ws/install/setup.bash
ros2 launch fast_livo mapping_mid360_lvi.launch.py use_rviz:=false
```

Depending on the fork, you may need to set `use_sim_time: true` on the FAST-LIVO2 node or adapt its clock handling. Calibration of intrinsics/extrinsics and real timing is mandatory before reporting odometry accuracy. The Webots camera/lidar mount metadata is in `docs/SENSOR_MOUNTS.md`.

## Critical physical limitations (read before claiming a successful experiment)

- The default `livo2.wbt` uses 16 ms simulation steps (nominal 62.5 Hz IMU). An optional `livo2_fast.wbt` uses 8 ms simulation steps (nominal 125 Hz IMU), but the actual real-time update rate depends on Webots performance.
- The Webots LiDAR publishes a **generic fixed 360-degree scan**, not the Livox Mid-360 scanning pattern; reflectivity is a fixed simulated value and per-point timestamps are zero unless explicitly configured to use an assumed scan duration.
- The existing drone is **kinematically driven by Supervisor** and calls `resetPhysics()` during movement. Its IMU acceleration/gyro outputs can therefore be poorly representative of real flight and may cause LIVO2 to fail or drift.
- The SLAM camera is a *different* low-bandwidth imaging device from the two 4K photo cameras. Intrinsics are estimated from a pinhole FOV model, and a rigid camera-LiDAR transform is not yet rigorously calibrated.
- Webots time alignment is based on one simulation clock, but **hardware-style synchronization, LiDAR scan deskewing and actual visual-inertial estimation are not validated**.
- GNSS/ground truth is published for evaluation only, not fused into FAST-LIVO2. Terrain landing assessment is a separate module and not derived from FAST-LIVO2's map.

### Successful integration acceptance criteria

Only mark FAST-LIVO2 as **fully integrated and tested** after all of the following are confirmed on the target Ubuntu machine:

1. ROS2 bridge shows live, nonempty `/livox/lidar`, `/livox/imu` and `/webots/slam/image_raw` with monotonic timestamps.
2. Camera-LiDAR-IMU intrinsics/extrinsics are calibrated and logged; input image and point cloud axes are validated.
3. The third-party FAST-LIVO2 node builds and launches in LVI mode, without missing dependency errors.
4. An *estimated trajectory* topic (e.g. `/aft_mapped_to_init`) and a *map* topic (e.g. `/Laser_map`) publish genuine FAST-LIVO2 outputs rather than supervisor ground truth.
5. Results are plotted against `/webots/ground_truth`; absolute pose and drift metrics are recorded on repeatable routes.
6. If the estimator fails due to insufficient IMU physics or scan timing, state that accurately in the project report and do not claim full SLAM.

## Useful reference material

- Official FAST-LIVO2: https://github.com/hku-mars/FAST-LIVO2
- ROS2 Humble port, Mid-360: https://github.com/U-AMC/FAST-LIVO2-ROS2
- Livox ROS2 `CustomMsg`: https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomMsg.msg
- Webots LiDAR documentation: https://github.com/cyberbotics/webots/blob/master/docs/reference/lidar.md

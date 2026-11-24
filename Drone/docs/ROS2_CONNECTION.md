# ROS 2 connection | Drone hybrid Webots project

## Status and requirements

The **ament_python ROS 2 package** (`drone_ros_bridge`) publishes Webots LiDAR, IMU, RGB, GNSS-derived pose and simulation clock topics. Open `hybrid_wheels_ros2.wbt` to demonstrate rolling Hybrid A*, flight and the ROS 2 sensor feed in one world. The 8 ms Webots step supports a nominal 125 Hz IMU feed. `hybrid_ros2.wbt` provides the walking and flight version.

The ROS 2 bridge was exercised with Webots R2025a and Humble. The wheel world rolled using Hybrid A*, flew the 17-stop survey and landed. A ROS 2 subscriber received IMU, point cloud, RGB, camera intrinsics, pose reference and clock messages. The combined launch depends on the U-AMC FAST-LIVO2 port and Livox message definitions, which are installed separately. Flight and ground control use kinematic motion and the known farm map. Webots supplies a generic LiDAR scan pattern, fixed reflectivity and simulated timing.

A live combined launch with the U-AMC port published finite odometry, a path, nonempty registered clouds and RGB output from Webots. The airborne estimate drifted substantially from the simulated pose, so the combined run establishes data flow and estimator execution.

Source of upstream estimator (third-party): https://github.com/U-AMC/FAST-LIVO2-ROS2 ; original research project: https://github.com/hku-mars/FAST-LIVO2 . The third-party source is not redistributed in this package; fetch its source separately and comply with its license.

## 1. Start Windows Webots + WSL2

1. Keep your full `Drone` folder including original 4K `textures/`, `controllers/`, and `worlds/`.
2. Install Windows WSL2 Ubuntu 22.04, or use a separate Ubuntu 22.04 computer. From an **elevated Windows PowerShell** you can install it with `wsl --install -d Ubuntu-22.04` and reboot when requested; then finish Ubuntu's first-run setup.
3. Inside Ubuntu 22.04 install ROS **Humble** according to the official installation page https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html . This is a prerequisite; `apt install ros-humble-desktop` alone will not work until the official ROS apt source is configured.
4. Install bridge build dependencies in Ubuntu:

   ```bash
   source /opt/ros/humble/setup.bash
   sudo apt update
   sudo apt install -y python3-colcon-common-extensions python3-yaml python3-numpy git
   ```

5. Access the project from Ubuntu WSL2 through the mounted Windows C drive, e.g.:

   ```bash
   cd /mnt/c/Users/Eng/Desktop/Drone  # replace with your actual extracted Drone path
   bash integration/ros2/setup_ros2_workspace.sh
   ```

   If building under `/mnt/c` causes permission/symlink errors, copy only the small `ros2_ws` directory to your Ubuntu home directory and run `colcon build --symlink-install --packages-select drone_ros_bridge` there after sourcing ROS 2.

6. In Ubuntu **terminal A** run:

   ```bash
   cd /mnt/c/Users/Eng/Desktop/Drone
   bash integration/ros2/run_ros2_bridge.sh
   ```

7. In **Windows**, double-click `hybrid_ros2.bat` or open `worlds/hybrid_ros2.wbt` in Webots R2025a, then press **Play**. The Windows Webots application publishes TCP sensor packets to the ROS 2 receiver at port `15882`.
8. In Ubuntu **terminal B**, verify:

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list -t
   ros2 topic hz /livox/imu
   ros2 topic hz /webots/lidar/points
   ros2 topic hz /webots/slam/image_raw
   ```

   `/livox/lidar` additionally requires `livox_ros_driver2/msg/CustomMsg`; until that package has been compiled, only generic `PointCloud2` is published.

### Windows to WSL2 networking

If the sender cannot connect, the default `127.0.0.1` may be wrong on WSL2 NAT. In Windows PowerShell inspect `wsl hostname -I`; put the WSL2 IPv4 into `controllers/drone/settings.json` under `fast_livo_host` and reopen the Webots world. The receiver binds `0.0.0.0` inside WSL2; keep port 15882 on your **private local machine** only. With WSL2 mirrored networking localhost may work. **Only one receiver may bind port 15882**; close `integration/monitor.bat` before starting ROS 2.

## 2. Build and launch third-party FAST-LIVO2 (advanced)

In Ubuntu with ROS 2 Humble installed:

```bash
cd /path/to/Drone
bash integration/ros2/fetch_fast_livo2_sources.sh
```

Read the **entire fork README** for missing system libraries, Sophus, Livox SDK2 and Vikit support before executing its build instructions:

```bash
cd ~/fast_ws
colcon build --symlink-install --continue-on-error
source ~/fast_ws/install/setup.bash
```

The upstream launch uses the ROS 2 parameter blackboard. If your Humble installation lacks it, install `ros-humble-demo-nodes-cpp`.

Generate simulation YAML from the installed fork's parameter files:

```bash
python3 /path/to/Drone/integration/ros2/prepare_fast_config.py \
  --upstream ~/fast_ws/src/fast_livo --output ~/drone_ros_config
```

The generated defaults use the nominal mounts and pinhole intrinsics of the two hybrid ROS 2 worlds. The fork's AVIA input branch accepts the simulated 24-layer Livox-format messages. The combined launch adds a body-frame specific-force reference for the zero-gravity world. Verify mounts, timing and estimation quality against recorded simulation data before using the results quantitatively.

To run the ROS 2 bridge **and** the external estimator within the same launch context, close the bridge-only terminal and run:

```bash
cd /path/to/Drone
bash integration/ros2/run_ros2_fast_livo2.sh
```

Then open `worlds/hybrid_wheels_ros2.wbt` in Webots and press Play. Look for the `[HYBRID A*] rolling poses:` log, then check actual topic traffic:

```bash
ros2 topic list -t
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /webots/slam/image_raw
ros2 topic echo /webots/ground_truth --once
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /cloud_registered
```

Save actual odometry and point-cloud outputs and compare against `/webots/ground_truth` after aligning the estimator's local frame to the Webots map frame. After resetting the Webots world, restart the estimator because simulation timestamps go backwards.

## 3. Topics and limitations

| ROS 2 topic | Type | Meaning |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Webots generic LiDAR in Livox **message type**, not Mid-360 ray pattern |
| `/webots/lidar/points` | `sensor_msgs/msg/PointCloud2` | Generic Webots points |
| `/livox/imu` | `sensor_msgs/msg/Imu` | InertialUnit, gyro, accelerometer from Webots |
| `/webots/slam/image_raw` | `sensor_msgs/msg/Image` | 640 x 480 RGB view (4K photography cameras remain separate) |
| `/webots/slam/camera_info` | `sensor_msgs/msg/CameraInfo` | Pinhole model from Webots field of view |
| `/webots/ground_truth` | `geometry_msgs/msg/PoseStamped` | Simulated GPS/supervisor-derived pose reference, not SLAM |
| `/clock` | `rosgraph_msgs/msg/Clock` | Webots simulation time |

**ROS 2 `CustomMsg` transport does not mean FAST-LIVO2 is solved.** Confirm estimator uses synchronized actual messages, correct 3D coordinate conventions, extrinsics and meaningful per-point time data. Calibration and estimator computation are still future validation, not a checkbox that can simply be asserted as done.

## 4. What to show the professor

Show the open Webots hybrid ROS2 world, ROS `topic list`/`topic hz` and optionally RViz PointCloud2. Only show mapped trajectory if actually produced by the third-party FAST-LIVO2 process. Keep a note that offline Python tests validate message framing and launch/package structure, but Ubuntu ROS2 runtime and actual full 3D visual SLAM require testing on the intended laptop.

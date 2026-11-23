#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "$0")/../.." && pwd)/ros2_ws/install/setup.bash"
ros2 bag record /clock /livox/lidar /livox/imu /webots/slam/image_raw \
  /webots/slam/camera_info /webots/ground_truth -o "${1:-drone_ros2_recording}"

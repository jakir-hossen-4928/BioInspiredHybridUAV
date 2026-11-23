#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "$HERE/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT/ros2_ws/install/setup.bash"
ros2 topic list -t
printf '
Expected data-rate diagnostics:
'
for topic in /livox/imu /livox/lidar /webots/slam/image_raw /webots/ground_truth; do
  echo "Inspect: ros2 topic hz $topic"
done

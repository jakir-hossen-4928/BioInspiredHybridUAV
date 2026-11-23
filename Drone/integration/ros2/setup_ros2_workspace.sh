#!/usr/bin/env bash
set -euo pipefail
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble not found. Install ROS 2 Humble on Ubuntu 22.04 first.' >&2; exit 2
fi
source /opt/ros/humble/setup.bash
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "$HERE/../.." && pwd)"
WS="$PROJECT/ros2_ws"
cd "$WS"
python3 -c 'import yaml' || { echo 'Please install python3-yaml'; exit 2; }
colcon build --symlink-install --packages-select drone_ros_bridge
source "$WS/install/setup.bash"
ros2 pkg executables drone_ros_bridge
printf '
ROS 2 bridge package built at %s
' "$WS"

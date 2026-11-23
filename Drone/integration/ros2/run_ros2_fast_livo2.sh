#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "$HERE/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$HOME/fast_ws/install/setup.bash" ]]; then
  echo 'FAST-LIVO2 upstream not built. Read docs/ROS2_CONNECTION.md' >&2; exit 2
fi
source "$HOME/fast_ws/install/setup.bash"
source "$PROJECT/ros2_ws/install/setup.bash"
CFG="$HOME/drone_ros_config"
if [[ ! -f "$CFG/mid360_webots.yaml" || ! -f "$CFG/camera_webots.yaml" ]]; then
  echo 'Config missing. Run prepare_fast_config.py first.' >&2; exit 2
fi
exec ros2 launch drone_ros_bridge fast_livo2.launch.py \
  mid360_params_file:="$CFG/mid360_webots.yaml" \
  camera_params_file:="$CFG/camera_webots.yaml" use_rviz:=false host:=0.0.0.0

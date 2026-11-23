#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(cd "$HERE/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT/ros2_ws/install/setup.bash"
exec ros2 launch drone_ros_bridge bridge.launch.py host:=0.0.0.0 port:=15882

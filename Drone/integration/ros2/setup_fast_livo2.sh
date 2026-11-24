#!/usr/bin/env bash
set -euo pipefail
if [[ "${ROS_DISTRO:-}" != 'humble' ]]; then
  echo "Source ROS 2 Humble first: source /opt/ros/humble/setup.bash"
  exit 1
fi
# This script fetches source only. Install the prerequisites listed by the fork
# before attempting to build. Upstream GPLv2 source is not copied into this patch.
mkdir -p "$HOME/fast_ws/src"
cd "$HOME/fast_ws/src"
if [[ ! -e fast_livo ]]; then
  git clone https://github.com/U-AMC/FAST-LIVO2-ROS2.git fast_livo
fi
if [[ ! -e livox_ros_driver2 ]]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git
fi
printf '\nSources fetched. Install dependencies per fork README before colcon build:\n'
printf '  cd ~/fast_ws && colcon build --symlink-install\n'
printf '  source ~/fast_ws/install/setup.bash\n'

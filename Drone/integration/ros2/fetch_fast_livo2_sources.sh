#!/usr/bin/env bash
set -euo pipefail
if [[ "$(. /etc/os-release; echo "$ID-$VERSION_ID")" != 'ubuntu-22.04' ]]; then
  echo 'This FAST-LIVO2 fork is documented for Ubuntu 22.04 + ROS 2 Humble.' >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Install ROS 2 Humble per the official ROS installation guide first.' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
mkdir -p "$HOME/fast_ws/src"
cd "$HOME/fast_ws/src"
if [[ ! -d fast_livo/.git ]]; then git clone https://github.com/U-AMC/FAST-LIVO2-ROS2.git fast_livo; fi
if [[ ! -d rpg_vikit/.git ]]; then git clone https://github.com/U-AMC/rpg_vikit_rational_polynomial.git rpg_vikit; fi
if [[ ! -d livox_ros_driver2/.git ]]; then git clone https://github.com/Livox-SDK/livox_ros_driver2.git livox_ros_driver2; fi
printf '\nFAST-LIVO2 ROS2 source repositories fetched at ~/fast_ws/src\n'
printf 'Follow the U-AMC fork README to install Sophus, Livox SDK2, Vikit dependencies then build:\n'
printf '  cd ~/fast_ws\n  colcon build --symlink-install --continue-on-error\n'
printf 'Upstream GPLv2 code belongs to its original maintainers; this script fetches it unchanged.\n'

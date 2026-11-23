#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
if [[ ! -f "$HOME/fast_ws/install/setup.bash" ]]; then
  echo "FAST-LIVO2 workspace not found at ~/fast_ws. Read docs/FAST_LIVO2_SETUP.md."
  exit 2
fi
source "$HOME/fast_ws/install/setup.bash"
# The exact launch file is provided by the community port noted in the guide.
ros2 launch fast_livo mapping_mid360_lvi.launch.py use_rviz:=false

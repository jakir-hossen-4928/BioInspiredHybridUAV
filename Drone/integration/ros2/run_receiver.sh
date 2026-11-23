#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/fast_ws/install/setup.bash" ]]; then
    source "$HOME/fast_ws/install/setup.bash"
fi
cd "$(dirname "$0")"
python3 bridge_receiver.py --host "${BRIDGE_LISTEN_HOST:-0.0.0.0}" --port "${BRIDGE_PORT:-15882}"

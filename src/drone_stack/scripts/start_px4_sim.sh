#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
python3 "$project_root/src/drone_stack/scripts/prepare_px4_sitl.py"
source /opt/ros/noetic/setup.bash
export PYTHONPATH="$project_root/external/px4_python_deps:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${LD_LIBRARY_PATH:-}"
export GIT_SUBMODULES_ARE_EVIL=1
if [[ "${DRONE_PX4_INTERACTIVE:-0}" != 1 ]]; then
  export NO_PXH=1
else
  unset NO_PXH
fi
export PX4_GAZEBO_JOBS=2
export PX4_SITL_WORLD="${DRONE_SIM_WORLD:-$project_root/src/drone_stack/worlds/inspection_demo.world}"
cd "$project_root/external/PX4-Autopilot"
make px4_sitl_inspection gazebo-classic_inspection_quad

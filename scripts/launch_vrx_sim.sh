#!/usr/bin/env bash
# Milestone M1 Definition of Done: "one command launches simulation."
#
# Wraps `ros2 launch vrx_gz competition.launch.py` with the environment
# fixes documented in docs/environment.md, so they don't need to be
# rediscovered:
#   - strips conda from PATH and unsets CONDA_* (ROS2/colcon need system
#     Python; CMake's FindPython3 prefers CONDA_PREFIX over PATH order)
#   - adds the wamv_description/wamv_gazebo share dirs to
#     GZ_SIM_RESOURCE_PATH (VRX's own package hooks register these one
#     directory level too shallow, and omit wamv_gazebo entirely, so
#     GPS/engine/propeller meshes fail to load without this)
#
# Usage: scripts/launch_vrx_sim.sh [world:=sydney_regatta] [headless:=True] [...]
# Any extra arguments are passed straight through to competition.launch.py.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$REPO_ROOT/ros2_ws"

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "error: $WS not built yet. Run: cd $WS && colcon build --symlink-install" >&2
    exit 1
fi

CLEAN_PATH=$(echo "$PATH" | tr ':' '\n' | grep -v anaconda3 | paste -sd: -)

exec env -u CONDA_PREFIX -u CONDA_EXE -u CONDA_PYTHON_EXE -u CONDA_DEFAULT_ENV \
    -u CONDA_SHLVL PATH="$CLEAN_PATH" bash -c "
source /opt/ros/jazzy/setup.bash
source '$WS/install/setup.bash'
export GZ_SIM_RESOURCE_PATH=\"\$GZ_SIM_RESOURCE_PATH:$WS/install/wamv_description/share:$WS/install/wamv_gazebo/share\"
exec ros2 launch vrx_gz competition.launch.py $*
"

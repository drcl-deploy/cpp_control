# Clean shell for BUILDING this workspace (source me, don't execute me).
#
# The user's shell may source a different ROS distro (this machine has
# /opt/ros/jazzy); mixing distros breaks colcon, so start from a stripped
# environment and bring in only the conda humble env.
unset ROS_DISTRO AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
      LD_LIBRARY_PATH PYTHONPATH ROS_VERSION ROS_PYTHON_VERSION \
      ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE RMW_IMPLEMENTATION

_dr_here=${BASH_SOURCE[0]:-${(%):-%x}}
_dr_here=$(cd "$(dirname "$_dr_here")" && pwd)

source "$(conda info --base 2>/dev/null || echo "$HOME/miniconda3")/etc/profile.d/conda.sh"
conda activate "${ENV_NAME:-drclros}"
export PFX=$CONDA_PREFIX

# ── Where things are ──────────────────────────────────────────────────────────
#
# ONE workspace now, and that is the whole point of the layout:
#
#   unitree_ros2/                       $UNITREE_ROS2
#   ├── cyclonedds_ws/                  $WS  — the colcon workspace root
#   │   ├── src/unitree/                unitree_go, unitree_hg, unitree_api
#   │   ├── src/cpp_control/            this package
#   │   └── src/optitrack_msgs ->       (symlink; the lab's OptiTrack interface)
#   ├── unitree_sdk2/                   what unitree_mujoco links against
#   └── unitree_mujoco/                 $UNITREE_MUJOCO — the simulator
#
# cpp_control used to live in a SECOND colcon workspace (drcl/) and take
# unitree_hg from this one across a prefix-path bridge. It does not any more:
# unitree_hg and cpp_control are built by the same `colcon build`, into the same
# install/, and there is only one overlay for a launch to read. Two install
# trees holding the same package is precisely the "a fresh export does nothing"
# trap, and it cannot happen now.
#
# This file is four levels below the workspace root:
#   $WS/src/cpp_control/workflows/conda_env/env.sh
export WS=${WS:-$(cd "$_dr_here/../../../.." && pwd)}
export UNITREE_ROS2=${UNITREE_ROS2:-$(cd "$WS/.." && pwd)}
export UNITREE_MUJOCO=${UNITREE_MUJOCO:-$UNITREE_ROS2/unitree_mujoco}

# mj_sim (the drcl_deploy plant) resolved its scene paths against this. That
# workspace is gone, so `assets` is normally not importable and this stays
# unset — which is correct: nothing in the unitree workflow reads it.
if python -c 'import assets' 2>/dev/null; then
    export SIM_ASSETS_PATH=$(python -c 'from assets.paths import asset_path; import os; print(os.path.dirname(os.path.dirname(asset_path("g1", "scene_flat.xml"))))')
fi

export LD_LIBRARY_PATH=$PFX/lib:${LD_LIBRARY_PATH}

unset _dr_here

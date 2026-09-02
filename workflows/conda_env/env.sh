# Clean shell for BUILDING the drcl_deploy workspace (source me, don't execute me).
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

# The workspace root is two levels above this package (…/drcl/cpp_control/…).
export WS=${WS:-$(cd "$_dr_here/../../.." && pwd)}

# mj_sim resolves its scene paths against this; unset, it fails with a
# TypeError on None rather than a message.
if python -c 'import assets' 2>/dev/null; then
    export SIM_ASSETS_PATH=$(python -c 'from assets.paths import asset_path; import os; print(os.path.dirname(os.path.dirname(asset_path("g1", "scene_flat.xml"))))')
fi

export LD_LIBRARY_PATH=$PFX/lib:${LD_LIBRARY_PATH}

unset _dr_here

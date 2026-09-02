# Shell environment for RUNNING the workspace (source me, don't execute me).
#
#   source workflows/conda_env/runenv.sh
#   ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk
#
# Works from bash and zsh. This is the counterpart to env.sh: env.sh sets up the
# build environment only, deliberately WITHOUT the workspace overlay, because
# running colcon build with its own install/ already sourced causes trouble.
# This one adds the overlay on top.

_dr_here=${BASH_SOURCE[0]:-${(%):-%x}}
_dr_here=$(cd "$(dirname "$_dr_here")" && pwd)

. "$_dr_here/env.sh"

# Overlay. colcon writes one per shell flavour; picking the wrong one silently
# leaves AMENT_PREFIX_PATH unset, and ros2 then falls back to whatever the
# system has and fails with
#   OSError: Environment variable 'AMENT_PREFIX_PATH' is not set or empty
if [ -n "$ZSH_VERSION" ]; then
    _dr_setup="$WS/install/setup.zsh"
    # colcon's zsh hooks call compdef; without compinit having run, zsh prints
    # "_comps: assignment to invalid subscript range" for every package.
    if ! (( ${+_comps} )); then
        autoload -Uz compinit && compinit -u -d "${XDG_CACHE_HOME:-$HOME/.cache}/drcl_zcompdump" >/dev/null 2>&1
    fi
else
    _dr_setup="$WS/install/setup.bash"
fi

if [ -f "$_dr_setup" ]; then
    . "$_dr_setup"
else
    echo "runenv.sh: no overlay at $_dr_setup"
    echo "           build first:  bash $_dr_here/build.sh --packages-up-to cpp_control"
fi

# Keep DDS traffic on this machine. A lab network has other ROS nodes on it, and
# a simulator that discovers them is a confusing way to find that out. It also
# survives a VPN or firewall blocking multicast, which otherwise leaves two
# nodes on THIS machine unable to find each other.
#
# For the robot, or a remote operator PC, DDS must cross the network:
#     DRCL_ROS_NETWORK=1 source runenv.sh
if [ "${DRCL_ROS_NETWORK:-0}" = "1" ]; then
    export ROS_LOCALHOST_ONLY=0
else
    export ROS_LOCALHOST_ONLY=1
fi

_dr_ros2=$(command -v ros2 2>/dev/null)
case "$_dr_ros2" in
    "$CONDA_PREFIX"/*)
        echo "drcl env ready: ros2=$_dr_ros2"
        echo "                workspace=$WS"
        echo "                assets=$SIM_ASSETS_PATH"
        ;;
    "")  echo "runenv.sh: no ros2 on PATH -- the conda env did not activate." ;;
    *)   echo "runenv.sh: WARNING ros2 resolves to $_dr_ros2, not the ${ENV_NAME:-drclros} env." ;;
esac

unset _dr_here _dr_setup _dr_ros2

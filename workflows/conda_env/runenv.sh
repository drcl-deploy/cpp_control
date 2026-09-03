# Shell environment for RUNNING the workspace (source me, don't execute me).
#
#   source workflows/conda_env/runenv.sh
#   ros2 launch cpp_control g1_difftrack.launch.py motion:=g1_walk
#
# The workspace is unitree_ros2/cyclonedds_ws and it holds BOTH cpp_control and
# the unitree message packages, so there is one overlay to source rather than
# two prefixes to keep in agreement. See env.sh for the layout.
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

# unitree_go / unitree_hg / unitree_api used to be sourced separately here, out
# of their own workspace. They are in THIS one now, so the overlay below brings
# them in with everything else; a cpp_control binary built against unitree_hg
# will not start without it on AMENT_PREFIX_PATH, and one source now does both.

if [ -f "$_dr_setup" ]; then
    . "$_dr_setup"
else
    echo "runenv.sh: no overlay at $_dr_setup"
    echo "           build first:  bash $_dr_here/build.sh --packages-select unitree_go unitree_hg unitree_api"
    echo "                         bash $_dr_here/build.sh --packages-select cpp_control"
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

# ── DDS middleware ────────────────────────────────────────────────────────────
#
# rmw_cyclonedds_cpp on the loopback interface, and this is the whole reason the
# workflow interoperates: unitree_mujoco and the robot publish raw CycloneDDS on
# `rt/lowstate` etc. via unitree_sdk2, and a ROS 2 node on the SAME middleware,
# the same domain and the same interface sees them as /lowstate — because ROS 2's
# DDS topic for /lowstate is literally `rt/lowstate`.
#
# This is workflows/unitree.md's setup.sh, with the interface pinned to `lo` as
# its setup_local.sh does for simulation.
#
# It is now the DEFAULT rather than an opt-in. There used to be a second plant
# in a second workspace (mj_sim, over messages/G1State, on whatever RMW the
# environment had) and picking between them had to be a deliberate choice per
# shell, because a mismatch is silent: every node starts, every topic is
# declared, and nothing is ever received. That workspace is gone; unitree is the
# only message path this package ships against now, and it is the robot's.
#
#   DRCL_WORKFLOW=none source runenv.sh    leave RMW/domain/interface alone
#
# ROS_LOCALHOST_ONLY is cleared on purpose: it makes rmw_cyclonedds synthesise
# its own config, which then competes with the explicit interface below. The
# `lo` interface already confines the traffic to this machine, and more tightly.
#
# ROS_DOMAIN_ID must equal the simulator's -i / config.yaml domain_id. 0 is what
# unitree.md's usage line passes.
#
# On the ROBOT the interface is a real NIC, not `lo`:
#     DRCL_ROS_NETWORK=1 CYCLONE_IFACE=enp3s0 source runenv.sh
if [ "${DRCL_WORKFLOW:-unitree}" = "unitree" ]; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_LOCALHOST_ONLY=0
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
    _dr_iface=${CYCLONE_IFACE:-lo}
    # Assembled in a plain variable rather than inside ${VAR:-...}: the default
    # word of a parameter expansion eats the quoting needed to interpolate the
    # interface name, and the result is a URI with an unterminated attribute
    # that CycloneDDS accepts and then ignores.
    if [ -z "${CYCLONEDDS_URI:-}" ]; then
        export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"$_dr_iface\" priority=\"default\" multicast=\"default\" />
</Interfaces></General></Domain></CycloneDDS>"
    fi
fi

# Both shells cache command locations, and everything above just changed PATH
# out from under that cache. Without this, `ros2` still resolves to whatever the
# system had (here /opt/ros/jazzy) for the rest of the shell's life — while
# `command -v` reports the right one, so the check below would happily pass.
if [ -n "$ZSH_VERSION" ]; then
    rehash
else
    hash -r 2>/dev/null || true
fi

_dr_ros2=$(command -v ros2 2>/dev/null)
case "$_dr_ros2" in
    "$CONDA_PREFIX"/*)
        echo "drcl env ready: ros2=$_dr_ros2"
        echo "                workspace=$WS"
        if [ "${DRCL_WORKFLOW:-unitree}" = "unitree" ]; then
            echo "                workflow=unitree  rmw=$RMW_IMPLEMENTATION" \
                 "domain=$ROS_DOMAIN_ID iface=${_dr_iface:-lo}"
            echo "                unitree_mujoco=$UNITREE_MUJOCO"
        else
            echo "                workflow=$DRCL_WORKFLOW (middleware left untouched)"
        fi
        ;;
    "")  echo "runenv.sh: no ros2 on PATH -- the conda env did not activate." ;;
    *)   echo "runenv.sh: WARNING ros2 resolves to $_dr_ros2, not the ${ENV_NAME:-drclros} env." ;;
esac

unset _dr_here _dr_setup _dr_ros2 _dr_iface

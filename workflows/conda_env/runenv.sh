# Compatibility shim. The run environment lives in unitree_ros2/setup.sh now.
#
#     source unitree_ros2/setup.sh              auto: the robot's NIC, else `lo`
#     source unitree_ros2/setup.sh robot        force hardware
#     source unitree_ros2/setup.sh sim          force `lo`
#
# There used to be two things to source and they disagreed: this file defaulted
# the interface to `lo` and only set CYCLONEDDS_URI when it was still unset, so
# sourcing the repo's setup.sh first and this one second produced a shell with
# jazzy's ros2 on PATH and the conda env's libraries underneath it. One file now.
#
# Everything that used to be passed to this script still works:
#
#     old                                          new
#     source runenv.sh                             source setup.sh
#     DRCL_ROS_NETWORK=1 CYCLONE_IFACE=eth0 \      source setup.sh robot
#         source runenv.sh
#     DRCL_WORKFLOW=none source runenv.sh          (gone -- there is one plant)
#
# Kept because scripts and muscle memory point at it. It will be removed.

_dr_here=${BASH_SOURCE[0]:-${(%):-%x}}
_dr_here=$(cd "$(dirname "$_dr_here")" && pwd)
_dr_root=$(cd "$_dr_here/../../../../.." && pwd)

echo "runenv.sh is deprecated -- use:  source $_dr_root/setup.sh [sim|robot]"

# DRCL_ROS_NETWORK=1, or an explicitly named non-loopback interface, meant
# "hardware" in the old vocabulary.
if [ "${DRCL_ROS_NETWORK:-0}" = "1" ] || { [ -n "${CYCLONE_IFACE:-}" ] && [ "${CYCLONE_IFACE}" != "lo" ]; }; then
    _dr_fwd=robot
else
    _dr_fwd=sim
fi

. "$_dr_root/setup.sh" "$_dr_fwd"

unset _dr_here _dr_root _dr_fwd

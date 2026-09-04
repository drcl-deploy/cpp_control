#!/usr/bin/env bash
# Run the G1 onboard base-state estimator container.
#
#   bash docker/estimator/run.sh [options] [-- extra docker args]
#
#   -i IFACE     network interface for CycloneDDS. Default: `lo` when
#                ROS_LOCALHOST_ONLY=1 or nothing else is given (which is the
#                simulator case), otherwise pass the robot's, e.g. eth0.
#   -d ID        ROS_DOMAIN_ID (default: $ROS_DOMAIN_ID, else 0)
#   -t TAG       image tag (default g1-estimator)
#   -n NAME      container name (default g1-estimator)
#   -D           detached; prints the container id and returns
#   -q           quiet: no container log on stdout
#   -h           this help
#
# WHY --network=host. The estimator has to see the same DDS traffic as
# everything else — `/lowstate` from the robot or from unitree_mujoco, and its
# own `/odom` has to reach the controller. Docker's default bridge network is a
# separate broadcast domain and DDS discovery does not cross it: the container
# comes up clean, logs nothing wrong, and publishes an estimate nobody receives.
# Host networking is the supported configuration for ROS 2 containers and is
# what the README on the robot assumes.
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

iface=""
domain="${ROS_DOMAIN_ID:-0}"
tag=g1-estimator
name=g1-estimator
detach=0
quiet=0

while getopts "i:d:t:n:Dqh" opt; do
    case "$opt" in
        i) iface=$OPTARG ;;
        d) domain=$OPTARG ;;
        t) tag=$OPTARG ;;
        n) name=$OPTARG ;;
        D) detach=1 ;;
        q) quiet=1 ;;
        h) sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) exit 2 ;;
    esac
done
shift $((OPTIND - 1))
[ "${1:-}" = "--" ] && shift

if ! docker image inspect "$tag" >/dev/null 2>&1; then
    echo "run.sh: no image '$tag' — build it first:" >&2
    echo "        bash $here/build.sh" >&2
    exit 1
fi

# Interface. Loopback is the right default: both the simulator and the
# controller run on this machine, and unitree_ros2/setup.sh already pins DDS
# to localhost so a lab network full of other ROS nodes stays out of it.
if [ -z "$iface" ]; then
    iface=lo
    echo "run.sh: no -i given, using 'lo' (simulator on this machine)."
    echo "        On the robot this must be the interface that carries LowState, e.g. -i eth0."
fi

uri="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$iface\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"

# --rm because this container holds no state worth keeping: the filter is
# re-seeded at the origin every start, which is also what makes a restart a
# clean way to re-zero it.
run_args=(
    --rm
    --name "$name"
    --network host
    --ipc host
    -e "CYCLONEDDS_URI=$uri"
    -e "ROS_DOMAIN_ID=$domain"
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
)
[ "$detach" = 1 ] && run_args+=(-d) || run_args+=(-it)
[ "$quiet" = 1 ] && run_args+=(--log-driver none)

# Any container already up owns the topic; two estimators publishing /odom is
# not a race the controller can detect.
if docker ps --format '{{.Names}}' | grep -qx "$name"; then
    echo "run.sh: '$name' is already running. Stop it first:  docker stop $name" >&2
    exit 1
fi

echo "estimator: iface=$iface domain=$domain image=$tag"
exec docker run "${run_args[@]}" "$tag" "$@"

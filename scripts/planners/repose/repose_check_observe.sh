#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: repose_check_observe.sh

Offboard bridge client + the live observe check. Sibling of
repose_stream_observe.sh, minus the labelling: it answers "is the palette right
here", not "record 240 frames".

The onboard side is g1_repose_planner_calibration.launch.py, which holds the
controller in its nominal SONIC stand under calibration_lock — no reference
motion, no planner reference, so the robot stands and only the observe block
runs.

    ros2 launch cpp_control g1_repose_planner_calibration.launch.py \
        artifact_dir:=<export> env:=real       # onboard (env:=sim in sim2sim)
    bash repose_check_observe.sh                # offboard

Environment:
  VIBE_BRIDGE_ADDRESS      onboard bridge host
  BRIDGE_ROS_DOMAIN_ID     isolated offboard domain (default 71)
EOF
}

if (($#)) && [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cpp_control_root="$(cd -- "${script_dir}/../../.." && pwd)"
repository_root="$(cd -- "${cpp_control_root}/../../.." && pwd)"
workspace_setup="${repository_root}/cyclonedds_ws/install/setup.bash"

if [[ ! -f "${workspace_setup}" ]]; then
  echo "check_observe: workspace is not built: ${workspace_setup}" >&2
  exit 1
fi

ros_setup=/opt/ros/humble/setup.bash
[[ -f "${ros_setup}" ]] || ros_setup=/opt/ros/foxy/setup.bash
if [[ ! -f "${ros_setup}" ]]; then
  echo "check_observe: no supported ROS setup under /opt/ros" >&2
  exit 1
fi

set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

# The same isolated offboard domain the calibration viewer uses: telemetry only,
# and nothing this process publishes can reach the robot.
export ROS_DOMAIN_ID="${BRIDGE_ROS_DOMAIN_ID:-71}"
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export VIBE_BRIDGE_ADDRESS="${VIBE_BRIDGE_ADDRESS:-gilfoyle-rth}"

bridge_config="$(ros2 pkg prefix cdr_tcp_bridge)/share/cdr_tcp_bridge/config/repose_calibration.yaml"

children=()
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "${children[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  for pid in "${children[@]}"; do wait "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

echo "check_observe: server=${VIBE_BRIDGE_ADDRESS}:5556 domain=${ROS_DOMAIN_ID}"
ros2 run cdr_tcp_bridge bridge_client.sh -p config_path:="${bridge_config}" &
children+=("$!")

ros2 run cpp_control repose_check_observe.py &
children+=("$!")

wait -n "${children[@]}"

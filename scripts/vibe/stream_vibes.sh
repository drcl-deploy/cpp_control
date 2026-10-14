#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: stream_vibes.sh [record]

Starts the offboard CDR/TCP client and the encoder-frame/attention viewer.
Add `record` to write a timestamped bag. Source setup.sh for a real experiment
or setup_local.sh for simulation before running this script.
EOF
}

record=false
while (($#)); do
  case "$1" in
    record)
      record=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "stream_vibes: unknown argument '$1'" >&2
      usage >&2
      exit 2
      ;;
  esac
done

script_dir="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
# Source and installed entry points both find their colcon workspace by ancestry.
workspace="${VIBE_WORKSPACE_ROOT:+${VIBE_WORKSPACE_ROOT}/cyclonedds_ws}"
if [[ -z "$workspace" ]]; then
  candidate="$script_dir"
  while [[ "$candidate" != / ]]; do
    if [[ -f "$candidate/install/local_setup.bash" ]]; then workspace="$candidate"; break; fi
    candidate="$(dirname -- "$candidate")"
  done
fi
[[ -n "$workspace" ]] || { echo 'Source the workspace setup first.' >&2; exit 1; }
repository_root="${VIBE_WORKSPACE_ROOT:-$(dirname -- "$workspace")}"
helper="$script_dir/../ros_environment.sh"
[[ -f "$helper" ]] || helper="$script_dir/../../share/cpp_control/scripts/ros_environment.sh"
source "$helper"
cpp_control_source_ros "$workspace"
cpp_control_check_python cv2 numpy rclpy vision_encoders.msg
if [[ "$record" == true ]]; then cpp_control_check_python rosbag2_py; fi

export ROS_DOMAIN_ID="${BRIDGE_ROS_DOMAIN_ID:-71}"
# The setup files select the experiment DDS interface. This client republishes
# on a separate, local-only domain, so let ROS_LOCALHOST_ONLY select loopback.
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export VIBE_BAG_DIR="${VIBE_BAG_DIR:-${repository_root}/bags}"
export VIBE_BRIDGE_ADDRESS="${VIBE_BRIDGE_ADDRESS:-gilfoyle-rth}"

children=()
cleanup() {
  trap - EXIT INT TERM HUP
  for pid in "${children[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  for pid in "${children[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

echo "stream_vibes: server=${VIBE_BRIDGE_ADDRESS}:5556 client_domain=${ROS_DOMAIN_ID}"
ros2 run cdr_tcp_bridge bridge_client.sh &
children+=("$!")

echo "stream_vibes: starting encoder frame + attention overlay"
/usr/bin/python3 "${script_dir}/attention_overlay_viewer.py" &
children+=("$!")

if [[ "${record}" == true ]]; then
  echo "stream_vibes: recording to ${VIBE_BAG_DIR}"
  ros2 run cpp_control record_vibes.sh &
  children+=("$!")
fi

# Closing the viewer, recorder, or bridge tears down the other children too.
wait -n "${children[@]}"

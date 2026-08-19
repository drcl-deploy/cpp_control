#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: stream_sys1_observe.sh [record] [--samples N] [--negative-samples N]

Starts the isolated offboard CDR/TCP client and Sys1 RGB-D calibration viewer.
Add `record` to save clicked RGB-D snapshots and their labels as a rosbag.
The onboard side must run g1_sys1_observe_calibration.launch.py.

Environment:
  SYS1_OBSERVE_BAG_DIR   bag root (default: ./sys1_observe_bags)
  VIBE_BRIDGE_ADDRESS   onboard bridge host
EOF
}

record=false
samples=30
negative_samples=60
while (($#)); do
  case "$1" in
    record)
      record=true
      shift
      ;;
    --samples)
      if (($# < 2)); then
        echo "stream_sys1_observe: --samples needs a value" >&2
        exit 2
      fi
      samples="$2"
      shift 2
      ;;
    --negative-samples)
      if (($# < 2)); then
        echo "stream_sys1_observe: --negative-samples needs a value" >&2
        exit 2
      fi
      negative_samples="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "stream_sys1_observe: unknown argument '$1'" >&2
      usage >&2
      exit 2
      ;;
  esac
done
if ! [[ "${samples}" =~ ^[1-9][0-9]*$ ]]; then
  echo "stream_sys1_observe: --samples must be a positive integer" >&2
  exit 2
fi
if ! [[ "${negative_samples}" =~ ^[1-9][0-9]*$ ]]; then
  echo "stream_sys1_observe: --negative-samples must be a positive integer" >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cpp_control_root="$(cd -- "${script_dir}/../.." && pwd)"
repository_root="$(cd -- "${cpp_control_root}/../../.." && pwd)"
workspace_setup="${repository_root}/cyclonedds_ws/install/setup.bash"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "stream_sys1_observe: /opt/ros/humble/setup.bash not found" >&2
  exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
  echo "stream_sys1_observe: workspace is not built: ${workspace_setup}" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace_setup}"
set -u

export ROS_DOMAIN_ID="${BRIDGE_ROS_DOMAIN_ID:-71}"
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export VIBE_BRIDGE_ADDRESS="${VIBE_BRIDGE_ADDRESS:-gilfoyle-rth}"

bridge_share="$(ros2 pkg prefix cdr_tcp_bridge)/share/cdr_tcp_bridge"
bridge_config="${bridge_share}/config/sys1_observe.yaml"

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

echo "stream_sys1_observe: server=${VIBE_BRIDGE_ADDRESS}:5556 samples/color=${samples} negatives=${negative_samples}"
ros2 run cdr_tcp_bridge bridge_client.sh \
  -p config_path:="${bridge_config}" &
children+=("$!")

echo "stream_sys1_observe: starting RGB + depth calibration viewer"
ros2 run cpp_control sys1_observe_viewer.py \
  --samples "${samples}" --negative-samples "${negative_samples}" &
children+=("$!")

if [[ "${record}" == true ]]; then
  echo "stream_sys1_observe: recording enabled"
  ros2 run cpp_control record_sys1_observe.sh &
  children+=("$!")
fi

# Closing the viewer, recorder, or bridge tears down the other children too.
wait -n "${children[@]}"

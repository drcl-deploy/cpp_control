#!/usr/bin/env bash
set -euo pipefail

# Runs in the isolated offboard domain prepared by repose_stream_observe.sh.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cpp_control_root="$(cd -- "${script_dir}/../../.." && pwd)"
repository_root="$(cd -- "${cpp_control_root}/../../.." && pwd)"
bag_root="${REPOSE_OBSERVE_BAG_DIR:-${repository_root}/bags/repose_observe}"
mkdir -p "${bag_root}"

stamp="$(LC_TIME=C date '+%d%b%Y_%H_%M')"
bag_path="${bag_root}/${stamp}"
suffix=2
while [[ -e "${bag_path}" ]]; do
  bag_path="${bag_root}/${stamp}_${suffix}"
  suffix=$((suffix + 1))
done

bridge_share="$(ros2 pkg prefix cdr_tcp_bridge)/share/cdr_tcp_bridge"
qos_path="${bridge_share}/config/record_qos.yaml"
topics=(
  /vibe/planner/calibration/color/compressed
  /vibe/planner/calibration/depth
  /vibe/planner/calibration/camera_info
  /vibe/planner/calibration/labels
  /vibe/planner/calibration/observation
  /vibe/planner/calibration/state
  /vibe/planner/calibration/click
)

storage_args=(-s sqlite3)
if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  storage_args=(
    -s mcap
    --storage-config-file "${bridge_share}/config/mcap_writer.yaml"
  )
fi

echo "record_observe: ${bag_path}"
exec ros2 bag record \
  "${storage_args[@]}" \
  --qos-profile-overrides-path "${qos_path}" \
  -o "${bag_path}" \
  "${topics[@]}"

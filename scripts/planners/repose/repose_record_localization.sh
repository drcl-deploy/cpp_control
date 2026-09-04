#!/usr/bin/env bash
set -euo pipefail

# Continuous v8 localization evidence. Unlike the colour-calibration recorder,
# this captures every paired live read and needs no labels or button presses.
script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
script_dir="$(cd -- "$(dirname -- "${script_path}")" && pwd)"
cpp_control_root="$(cd -- "${script_dir}/../../.." && pwd)"
repository_root="$(cd -- "${cpp_control_root}/../../.." && pwd)"
bag_root="${REPOSE_LOCALIZATION_BAG_DIR:-${repository_root}/bags/repose_localization}"
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
  /vibe/planner/observe/color/compressed
  /vibe/planner/observe/depth
  /vibe/planner/observe/camera_info
  /vibe/planner/observe/labels
  /vibe/planner/observe/observation
  /vibe/controller/status
)

# Keep sqlite3 deliberately: repose_export_observe_bag.py reads it without
# depending on the installed ROS message schema and turns it into replay NPZs.
echo "record_localization: ${bag_path}"
exec ros2 bag record \
  -s sqlite3 \
  --qos-profile-overrides-path "${qos_path}" \
  -o "${bag_path}" \
  "${topics[@]}"

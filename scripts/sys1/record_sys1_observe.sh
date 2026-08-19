#!/usr/bin/env bash
set -euo pipefail

# Runs in the isolated offboard domain prepared by stream_sys1_observe.sh.
bag_root="${SYS1_OBSERVE_BAG_DIR:-${PWD}/sys1_observe_bags}"
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
  /vibe/sys1/calibration/color/compressed
  /vibe/sys1/calibration/depth
  /vibe/sys1/calibration/camera_info
  /vibe/sys1/calibration/labels
  /vibe/sys1/calibration/observation
  /vibe/sys1/calibration/state
  /vibe/sys1/calibration/click
)

storage_args=(-s sqlite3)
if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  storage_args=(
    -s mcap
    --storage-config-file "${bridge_share}/config/mcap_writer.yaml"
  )
fi

echo "record_sys1_observe: ${bag_path}"
exec ros2 bag record \
  "${storage_args[@]}" \
  --qos-profile-overrides-path "${qos_path}" \
  -o "${bag_path}" \
  "${topics[@]}"

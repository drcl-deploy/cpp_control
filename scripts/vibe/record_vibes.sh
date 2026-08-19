#!/usr/bin/env bash
set -euo pipefail

# This helper runs inside the isolated offboard domain prepared by
# stream_vibes.sh. Override the destination without editing the script:
#   VIBE_BAG_DIR=/data/vibe_bags ros2 run cpp_control record_vibes.sh

bag_root="${VIBE_BAG_DIR:-${PWD}/bags}"
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
# The last three are sys1's; they stay silent on an open-loop run, and a bag is
# self-describing, so the replay viewer shows the planner pane only when they
# actually carried something.
topics=(
  /enc/frame
  /enc/tokens
  /vibe/sonic/attention_mask
  /lowstate
  /lowcmd
  /vibe/sys1/status
  /vibe/sys0/status
  /vibe/sonic/goal_color
)

storage_args=(-s sqlite3)
if ros2 pkg prefix rosbag2_storage_mcap >/dev/null 2>&1; then
  storage_args=(
    -s mcap
    --storage-config-file "${bridge_share}/config/mcap_writer.yaml"
  )
else
  echo "record_vibes: MCAP plugin not found; using sqlite3"
fi

echo "record_vibes: ${bag_path}"
exec ros2 bag record \
  "${storage_args[@]}" \
  --qos-profile-overrides-path "${qos_path}" \
  -o "${bag_path}" \
  "${topics[@]}"

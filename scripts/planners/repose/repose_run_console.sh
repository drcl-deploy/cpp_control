#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: repose_run_console.sh [--color N]

Opens the planner console: the planner's state pane, and the one place the target
colour is set. Unlike the viewers, this joins the EXPERIMENT domain — the goal
colour travels upstream to the robot and the telemetry bridge is one-way — so
source setup.sh (real) or setup_local.sh (simulation) before running it, the
same file the task launch was started with.

Live viewing and recording stay where they were: scripts/vibe/stream_vibes.sh
carries the planner already.
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

ros_setup=""
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  ros_setup=/opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  ros_setup=/opt/ros/foxy/setup.bash
fi
if [[ -z "${ros_setup}" ]]; then
  echo "run_console: no supported ROS setup found under /opt/ros" >&2
  exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
  echo "run_console: workspace is not built: ${workspace_setup}" >&2
  exit 1
fi

# ROS-generated setup files are not nounset-safe.
set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

# No domain override here on purpose: the console must sit where the controller
# is, which is whatever the sourced experiment setup selected.
echo "run_console: console on ROS domain ${ROS_DOMAIN_ID:-0}"
exec python3 "${script_dir}/repose_console.py" "$@"

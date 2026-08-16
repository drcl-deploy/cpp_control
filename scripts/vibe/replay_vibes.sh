#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: replay_vibes.sh [BAG_DIRECTORY]

Opens a synchronized, read-only Vibe bag viewer. If BAG_DIRECTORY is omitted,
the newest completed bag under $VIBE_BAG_DIR (default: ~/unitree_ros2/bags) is
opened. The script prepares ROS and the workspace in a fresh terminal.
EOF
}

if (($# > 1)); then
  usage >&2
  exit 2
fi
if (($# == 1)) && [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cpp_control_root="$(cd -- "${script_dir}/../.." && pwd)"
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
  echo "replay_vibes: no supported ROS setup found under /opt/ros" >&2
  exit 1
fi
if [[ ! -f "${workspace_setup}" ]]; then
  echo "replay_vibes: workspace is not built: ${workspace_setup}" >&2
  exit 1
fi

# ROS-generated setup files are not nounset-safe.
set +u
source "${ros_setup}"
source "${workspace_setup}"
set -u

export VIBE_BAG_DIR="${VIBE_BAG_DIR:-${repository_root}/bags}"
export QT_IM_MODULE="${VIBE_QT_IM_MODULE:-compose}"
export PYTHONWARNINGS="${PYTHONWARNINGS:+${PYTHONWARNINGS},}ignore:Unable to import Axes3D:UserWarning:matplotlib.projections"
exec python3 "${script_dir}/replay_vibes.py" "$@"

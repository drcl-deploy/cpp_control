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
cpp_control_check_python cv2 numpy rclpy rosbag2_py PyQt5.QtCore matplotlib.backends.backend_qt5agg

export VIBE_BAG_DIR="${VIBE_BAG_DIR:-${repository_root}/bags}"
export QT_IM_MODULE="${VIBE_QT_IM_MODULE:-compose}"
export PYTHONWARNINGS="${PYTHONWARNINGS:+${PYTHONWARNINGS},}ignore:Unable to import Axes3D:UserWarning:matplotlib.projections"
exec /usr/bin/python3 "${script_dir}/replay_vibes.py" "$@"

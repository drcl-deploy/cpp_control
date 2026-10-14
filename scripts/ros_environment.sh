#!/usr/bin/env bash
# Internal environment setup for the viewing tools; does not start ROS nodes.
cpp_control_source_ros() {
  local workspace="$1" ros_root="${2:-/opt/ros}" distro="${ROS_DISTRO:-}" candidate
  local -a installed=()
  if [[ -z "$distro" ]]; then
    for candidate in foxy humble jazzy; do
      [[ ! -f "$ros_root/$candidate/setup.bash" ]] || installed+=("$candidate")
    done
    if ((${#installed[@]} != 1)); then
      echo 'Set ROS_DISTRO to the intended installed distribution (foxy, humble, jazzy).' >&2
      return 1
    fi
    distro="${installed[0]}"
  fi
  case "$distro" in foxy|humble|jazzy) ;;
    *) echo "Unsupported ROS_DISTRO: $distro" >&2; return 1 ;;
  esac
  if [[ ! -f "$ros_root/$distro/setup.bash" || ! -f "$workspace/install/local_setup.bash" ]]; then
    echo "ROS $distro or workspace overlay missing: $workspace/install" >&2
    return 1
  fi
  /usr/bin/python3 - "$workspace" "$distro" <<'PY' || return 1
import json
from pathlib import Path
import platform
import re
import sys
workspace, distro = Path(sys.argv[1]), sys.argv[2]
setup = workspace / 'install/setup.bash'
parents = set(re.findall(r'/opt/ros/([^/\s"\']+)', setup.read_text())) if setup.exists() else set()
if parents and parents != {distro}:
    raise SystemExit('Overlay belongs to another ROS distribution; rebuild it before using these tools.')
marker = workspace / 'install/vibe_build.json'
if marker.exists():
    metadata = json.loads(marker.read_text())
    expected = {'ros_distro': distro, 'architecture': platform.machine(),
                'python': '{}.{}'.format(*sys.version_info[:2])}
    if any(metadata.get(key) != value for key, value in expected.items()):
        raise SystemExit('Overlay ROS/Python/architecture differs; rebuild it before using these tools.')
PY
  local nounset=0 status=0
  [[ $- != *u* ]] || nounset=1
  set +u
  source "$ros_root/$distro/setup.bash" || status=$?
  if ((status == 0)); then
    source "$workspace/install/local_setup.bash" || status=$?
  fi
  ((nounset == 0)) || set -u
  return "$status"
}

cpp_control_check_python() {
  /usr/bin/python3 - "$@" <<'PY'
import importlib
import sys
try:
    for name in sys.argv[1:]:
        importlib.import_module(name)
except ImportError as error:
    raise SystemExit('ROS Python dependency unavailable: {}. Install the package runtime '
                     'dependencies (vibe_ws: ./install.sh --system).'.format(error))
PY
}

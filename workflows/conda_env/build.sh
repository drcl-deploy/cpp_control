#!/usr/bin/env bash
# Build unitree_ros2/cyclonedds_ws in the drclros conda env.
# Run under bash: zsh does not word-split unquoted variables, which silently
# folds every cmake flag into CMAKE_BUILD_TYPE.
#
#   bash workflows/conda_env/build.sh --packages-select unitree_go unitree_hg unitree_api
#   bash workflows/conda_env/build.sh --packages-select cpp_control
#
# CMP0094=NEW: several packages declare cmake_minimum_required(VERSION 3.8),
# which leaves FindPython on the pre-3.15 VERSION lookup strategy -- it then
# picks the highest Python on the machine rather than the one in this env, and
# fails on the missing dev headers/NumPy.
#
# -include cstdint: this env ships a much newer GCC than ROS 2 Humble targets,
# and GCC 13 stopped pulling <cstdint> in transitively. Humble's generated
# message headers use uint8_t without including it, so `messages` fails to
# compile. Forcing the include touches no generated or vendored source.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE"/env.sh

# Preflight. A later `conda install` can quietly pull empy back to 4.x or cmake
# to 4.x, and the resulting failures point at rosidl templates rather than at
# the environment.
preflight_fail=0
note() { printf '  %-6s %s\n' "$1" "$2"; }
empy_v=$(python -c 'import em; print(getattr(em, "__version__", "?"))' 2>/dev/null || echo missing)
if [ "$empy_v" != "3.3.4" ]; then
  note FAIL "empy is $empy_v, need 3.3.4  ->  pip install 'empy==3.3.4'"
  preflight_fail=1
fi
cmake_major=$(cmake --version | head -1 | sed 's/[^0-9]*\([0-9]*\).*/\1/')
if [ "$cmake_major" != "3" ]; then
  note FAIL "cmake is $(cmake --version | head -1), need 3.x  ->  conda install 'cmake<4'"
  preflight_fail=1
fi
if ! ls "$HERE"/../../thirdparty/onnxruntime-linux-*/lib/libonnxruntime.so >/dev/null 2>&1; then
  note FAIL "no onnxruntime under cpp_control/thirdparty  ->  bash $HERE/deps.sh"
  preflight_fail=1
fi
if [ "$preflight_fail" -ne 0 ]; then
  echo "Environment is not in the state build.sh expects; re-run mkenv.sh and deps.sh."
  exit 1
fi

# colcon puts a dependency's install prefix on the path only if the package
# DECLARES the dependency, and cpp_control deliberately leaves `unitree_hg` and
# `messages` out of its package.xml (CMake picks whichever is present at
# configure time). Without help it then configures with neither backend and
# stops at "At least one of unitree_hg or messages is required" -- and it does
# so even now that unitree_hg is a sibling in this very workspace, because the
# undeclared dependency is the reason, not the distance.
#
# So add every sibling install prefix explicitly. That is the targeted version
# of the workflow doc's `source install/setup.sh` step, and it keeps this script
# free of the workspace's own overlay -- running colcon build with install/
# sourced causes its own trouble.
for _d in "$WS"/install/*/; do
  [ -d "$_d" ] || continue
  case "$(basename "$_d")" in cpp_control|COLCON_IGNORE) continue ;; esac
  [ -d "$_d/share" ] || continue
  CMAKE_PREFIX_PATH="${_d%/}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
done

# unitree_hg is what compiles the LowState/LowCmd backend in -- the message path
# the robot actually speaks. Say whether it is there, because
# "Requested workflow backend not compiled" at startup is the only other place
# this gets noticed, and by then you are in front of a robot.
if [ -d "$WS/install/unitree_hg" ]; then
  echo "unitree backend: unitree_hg from $WS/install"
else
  echo "unitree backend: NOT available -- build the message packages first:"
  echo "                   bash $HERE/build.sh --packages-select unitree_go unitree_hg unitree_api"
  echo "                 without them cpp_control configures with no backend and stops."
fi
export CMAKE_PREFIX_PATH

cd "$WS"
colcon build "$@" \
  --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_POLICY_DEFAULT_CMP0094=NEW \
      "-DCMAKE_CXX_FLAGS=-include cstdint" \
      "-DEIGEN3_INCLUDE_DIR=$PFX/include/eigen3" \
  --symlink-install --parallel-workers 12

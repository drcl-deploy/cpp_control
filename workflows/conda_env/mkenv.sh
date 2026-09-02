#!/usr/bin/env bash
# Create the 'drclros' conda environment: ROS 2 Humble (RoboStack) plus the
# toolchain this workspace needs, without root.
#
# The documented setup wants ROS 2 Humble from apt. On a machine whose system
# ROS is a different distro (this one has /opt/ros/jazzy), or without sudo,
# this builds the whole toolchain into a conda environment instead.
#
# Re-running this is safe; it converges the environment onto the pinned
# versions. No -u: RoboStack's activation scripts reference unbound vars.
set -eo pipefail

ENV_NAME=${ENV_NAME:-drclros}

source "$(conda info --base)/etc/profile.d/conda.sh"
export CONDA_ALWAYS_YES=true

if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  conda create -n "$ENV_NAME" python=3.11 -c conda-forge --override-channels
fi
conda activate "$ENV_NAME"

conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --set channel_priority flexible

# cmake<4: several packages here still declare cmake_minimum_required(<3.5),
#   which CMake 4 refuses.
# numpy: rosidl_generator_py does find_package(Python ... NumPy) for `messages`.
# eigen: the difftrack observation builder's only maths dependency.
# yaml-cpp / nlohmann_json / zlib: cpp_control's config, manifest and cnpy.
conda install -c robostack-staging -c conda-forge \
  ros-humble-ros-base \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-ament-lint-auto ros-humble-ament-lint-common \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-joy \
  colcon-common-extensions \
  compilers 'cmake<4' ninja pkg-config make numpy \
  eigen yaml-cpp nlohmann_json zlib

# empy MUST be 3.3.4. RoboStack pulls 4.x, whose Interpreter API is different:
# rosidl_adapter then dies with "module 'em' has no attribute 'BUFFERED_OPT'"
# followed by "'NoneType' object has no attribute 'shutdown'". This has to run
# AFTER the conda install, because that step is what installs 4.x.
pip install --quiet 'empy==3.3.4'

# mj_sim is an ament_python package: MuJoCo and PyYAML have to be importable
# from THIS env's python, which is what `ros2 run mj_sim main` will use.
#
# setuptools<80: colcon builds an ament_python package with
# `setup.py develop --editable` under --symlink-install, and setuptools 80
# removed the `develop` command outright. The failure reads
# "error: option --editable not recognized", which points at mj_sim's setup.py
# rather than at setuptools.
pip install --quiet 'mujoco==3.3.6' pyyaml 'setuptools<80'

# ---------------------------------------------------------------------------
# Verify, rather than discover any of this three minutes into a build.
# ---------------------------------------------------------------------------
fail=0
check() {  # check <label> <got> <want>
  if [ "$2" = "$3" ]; then printf '  ok    %-16s %s\n' "$1" "$2"
  else printf '  FAIL  %-16s got %s, need %s\n' "$1" "$2" "$3"; fail=1; fi
}
present() {  # present <label> <import-or-path test>
  if eval "$2" >/dev/null 2>&1; then printf '  ok    %-16s present\n' "$1"
  else printf '  FAIL  %-16s missing\n' "$1"; fail=1; fi
}
echo
echo "environment check ($ENV_NAME):"
check empy "$(python -c 'import em; print(em.__version__)' 2>/dev/null)" "3.3.4"
present "em.Interpreter" 'python -c "import em; assert hasattr(em, \"Interpreter\")"'
check "cmake major" "$(cmake --version | head -1 | sed 's/[^0-9]*\([0-9]*\).*/\1/')" "3"
present numpy   'python -c "import numpy"'
present mujoco  'python -c "import mujoco"'
present rclpy   'python -c "import rclpy"'
present eigen   '[ -d "$CONDA_PREFIX/include/eigen3" ]'
check setuptools "$(python -c 'import setuptools; print(int(setuptools.__version__.split(".")[0]) < 80)')" "True"
present joy     '[ -d "$CONDA_PREFIX/share/joy" ]'

echo
if [ "$fail" -ne 0 ]; then
  echo "ENV BUILD INCOMPLETE -- see the failures above."
  exit 1
fi
echo "ENV BUILD OK -- next: bash deps.sh"

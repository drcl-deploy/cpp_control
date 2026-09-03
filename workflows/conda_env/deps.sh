#!/usr/bin/env bash
# Install the non-conda dependencies for the drcl_deploy workflow, without root:
#
#   * ONNX Runtime 1.22, symlinked under cpp_control/thirdparty/ where the
#     package's CMakeLists globs for it (readme step 2).
#   * the `assets` package (retired drcl_deploy plant; installed only if present) into the
#     drclros env, and SIM_ASSETS_PATH pointing at its asset root.
#
# Re-running this is safe; it skips what is already unpacked.
# No -u: RoboStack's activation scripts reference unbound vars.
set -eo pipefail

ENV_NAME=${ENV_NAME:-drclros}
ORT_VERSION=${ORT_VERSION:-1.22.0}

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/../.." && pwd)"          # cpp_control/
WS="${WS:-$(cd "$PKG/.." && pwd)}"        # the colcon workspace root

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate "$ENV_NAME"
PFX=$CONDA_PREFIX

# --- ONNX Runtime -----------------------------------------------------------
# CMakeLists globs thirdparty/onnxruntime-linux-x64-* and takes the highest
# version, so the layout matters more than the location: unpack into the env
# prefix (it is a toolchain, not source) and symlink it in, which is exactly
# what the readme's `ln -s` step does by hand.
ORT_DIR="$PFX/onnxruntime-linux-x64-${ORT_VERSION}"
if [ ! -d "$ORT_DIR" ]; then
  D=$(mktemp -d); trap 'rm -rf "$D"' EXIT
  ( cd "$D"
    wget -q "https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz"
    tar -xzf "onnxruntime-linux-x64-${ORT_VERSION}.tgz"
    mv "onnxruntime-linux-x64-${ORT_VERSION}" "$ORT_DIR" )
fi
mkdir -p "$PKG/thirdparty"
ln -sfn "$ORT_DIR" "$PKG/thirdparty/onnxruntime-linux-x64-${ORT_VERSION}"

# --- assets (retired drcl_deploy plant) --------------------------------------
# `assets` owns the MuJoCo scenes mj_sim loads, and mj_sim resolves
# cfg['sim']['model_path'] against $SIM_ASSETS_PATH. Both belong to the
# drcl_deploy workspace this package no longer lives in, so this is INFORMATIONAL
# now, not a requirement: the unitree plant brings its own model
# (unitree_mujoco/unitree_robots/g1) and scripts/make_sim2sim_scene.py
# --flavor unitree needs nothing but the `mujoco` python package.
if [ -d "$WS/src/assets" ]; then
  pip install --quiet -e "$WS/src/assets"
  echo "  note: drcl_deploy assets found and installed."
fi

# ---------------------------------------------------------------------------
fail=0
echo
echo "dependency check ($ENV_NAME):"
for f in "$PKG/thirdparty/onnxruntime-linux-x64-${ORT_VERSION}/include/onnxruntime_cxx_api.h" \
         "$PKG/thirdparty/onnxruntime-linux-x64-${ORT_VERSION}/lib/libonnxruntime.so"; do
  if [ -e "$f" ]; then printf '  ok    %s\n' "${f#$PKG/}"
  else printf '  FAIL  %s\n' "${f#$PKG/}"; fail=1; fi
done
# mujoco: scripts/make_sim2sim_scene.py --flavor unitree builds the sim2sim
# scene through MjSpec. This one IS required -- without it there is no scene and
# the sim2sim runner stops before it starts anything.
if python -c 'import mujoco' 2>/dev/null; then
  printf '  ok    %-24s %s\n' "mujoco" "$(python -c 'import mujoco; print(mujoco.__version__)')"
else
  printf '  FAIL  %-24s not importable  ->  pip install mujoco\n' mujoco; fail=1
fi
# assets belongs to the retired drcl_deploy plant: reported, never required.
if python -c 'import assets' 2>/dev/null; then
  printf '  ok    %-24s %s\n' "assets (drcl, optional)" "installed"
fi

echo
if [ "$fail" -ne 0 ]; then
  echo "DEPS INCOMPLETE -- see the failures above."
  exit 1
fi
echo "DEPS OK -- next: bash build.sh --packages-select unitree_go unitree_hg unitree_api"
echo "                 bash build.sh --packages-select cpp_control"

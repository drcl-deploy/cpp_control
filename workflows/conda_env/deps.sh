#!/usr/bin/env bash
# Install the non-conda dependencies for the drcl_deploy workflow, without root:
#
#   * ONNX Runtime 1.22, symlinked under cpp_control/thirdparty/ where the
#     package's CMakeLists globs for it (readme step 2).
#   * the `assets` package, which owns the MuJoCo scenes mj_sim loads, into the
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

# --- assets -----------------------------------------------------------------
# mj_sim resolves cfg['sim']['model_path'] against $SIM_ASSETS_PATH, and dies
# with a TypeError on None rather than a message if it is unset.
if [ -d "$WS/src/assets" ]; then
  pip install --quiet -e "$WS/src/assets"
else
  echo "  note: $WS/src/assets not cloned -- mj_sim will have no scenes."
  echo "        git clone https://github.com/drcl-deploy/assets $WS/src/assets"
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
if python -c 'import assets' 2>/dev/null; then
  printf '  ok    %-24s %s\n' "assets" "$(python -c 'from assets.paths import asset_path; print(asset_path("g1", "scene_flat.xml"))' 2>/dev/null || echo installed)"
else
  printf '  FAIL  %-24s not importable\n' assets; fail=1
fi

echo
if [ "$fail" -ne 0 ]; then
  echo "DEPS INCOMPLETE -- see the failures above."
  exit 1
fi
echo "DEPS OK -- next: bash build.sh --packages-up-to cpp_control"

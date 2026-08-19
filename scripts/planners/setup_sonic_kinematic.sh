#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
PACKAGE_DIR="$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)"
# CMake installs scripts flat under lib/cpp_control. Keep `ros2 run` useful by
# redirecting that layout to the package share directory used by launch files.
INSTALLED_SHARE="${SCRIPT_DIR}/../../share/cpp_control"
if [[ ! -d "${PACKAGE_DIR}/models/planner" && -d "${INSTALLED_SHARE}/models" ]]; then
  PACKAGE_DIR="$(CDPATH= cd -- "${INSTALLED_SHARE}" && pwd)"
fi
MODEL_DIR="${PACKAGE_DIR}/models/planner/sonic_kinematic"
MODEL_PATH="${MODEL_DIR}/planner_sonic.onnx"

if ! command -v hf >/dev/null 2>&1; then
  echo "error: Hugging Face CLI 'hf' is not installed" >&2
  echo "install it with: python3 -m pip install --user huggingface_hub" >&2
  exit 1
fi

mkdir -p "${MODEL_DIR}"

if [[ -s "${MODEL_PATH}" && "${1:-}" != "--force" ]]; then
  echo "SONIC kinematic model already present:"
  echo "  ${MODEL_PATH}"
  echo "Use --force to download it again."
  exit 0
fi

echo "Downloading nvidia/GEAR-SONIC:planner_sonic.onnx ..."
download_args=(
  nvidia/GEAR-SONIC
  planner_sonic.onnx
  --local-dir "${MODEL_DIR}"
)
if [[ "${1:-}" == "--force" ]]; then
  download_args+=(--force-download)
elif [[ $# -gt 0 ]]; then
  echo "usage: $0 [--force]" >&2
  exit 2
fi
hf download "${download_args[@]}"

if [[ ! -s "${MODEL_PATH}" ]]; then
  echo "error: download completed without ${MODEL_PATH}" >&2
  exit 1
fi

echo "SONIC kinematic model ready:"
echo "  ${MODEL_PATH}"
du -h "${MODEL_PATH}"

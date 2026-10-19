#!/usr/bin/env bash
# Build the archived Python binding against an installed PC Service SDK.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="$SCRIPT_DIR/xrobotoolkit-pybind"
PC_SERVICE_DIR="${XRTOOLKIT_PC_SERVICE_DIR:-/opt/apps/roboticsservice}"
VENV_DIR="${1:-$SCRIPT_DIR/.venv}"

if [ ! -f "$PC_SERVICE_DIR/SDK/x64/libPXREARobotSDK.so" ]; then
  echo "Missing PC Service SDK: $PC_SERVICE_DIR/SDK/x64/libPXREARobotSDK.so" >&2
  exit 1
fi

if [ ! -x "$VENV_DIR/bin/python" ]; then
  uv venv --python 3.11 "$VENV_DIR"
fi

"$VENV_DIR/bin/python" -m ensurepip --upgrade
"$VENV_DIR/bin/python" -m pip install --upgrade pip setuptools wheel pybind11

pybind_prefix="$("$VENV_DIR/bin/python" -c 'import pathlib, pybind11; print(pathlib.Path(pybind11.get_cmake_dir()).parents[2])')"
export CMAKE_PREFIX_PATH="${pybind_prefix}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PC_SERVICE_DIR/SDK/x64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

work_dir="$(mktemp -d)"
trap 'rm -rf "$work_dir"' EXIT
cp -a "$SOURCE_DIR/." "$work_dir/"
mkdir -p "$work_dir/lib" "$work_dir/include"
cp "$PC_SERVICE_DIR/SDK/x64/libPXREARobotSDK.so" "$work_dir/lib/"
cp "$PC_SERVICE_DIR/SDK/include/PXREARobotSDK.h" "$work_dir/include/"

pushd "$work_dir" >/dev/null
"$VENV_DIR/bin/python" -m pip install .
popd >/dev/null

"$VENV_DIR/bin/python" - <<'PY'
import xrobotoolkit_sdk
print(f"xrobotoolkit_sdk import succeeded: {xrobotoolkit_sdk.__file__}")
PY

echo "Built SDK environment: $VENV_DIR"
echo "Set RLINF_PICO_SDK_VENV=$VENV_DIR to use it with Release."

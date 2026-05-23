#!/usr/bin/env bash
set -euo pipefail

PYTHON_BIN="${PYTHON_BIN:-/data/tianang/miniconda/envs/aero_sim/bin/python}"
GRADIENT_EPS="${GRADIENT_EPS:-5e-5}"
MUJOCO_DIR="$("$PYTHON_BIN" - <<'PY'
import pathlib
import mujoco
print(pathlib.Path(mujoco.__file__).resolve().parent)
PY
)"
MUJOCO_VERSION="$("$PYTHON_BIN" - <<'PY'
import mujoco
print(mujoco.mj_versionString())
PY
)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_AUTOBIO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT="$REPO_AUTOBIO_DIR/libmjlab_thread.so.$MUJOCO_VERSION"

g++ -std=c++17 -O2 -fPIC -shared \
  -DMJLAB_THREAD_GRADIENT_EPS="$GRADIENT_EPS" \
  -I"$MUJOCO_DIR/include" \
  "$SCRIPT_DIR/thread.cc" \
  "$MUJOCO_DIR/libmujoco.so.$MUJOCO_VERSION" \
  -Wl,-rpath,"$MUJOCO_DIR" \
  -o "$OUT"

echo "$OUT"

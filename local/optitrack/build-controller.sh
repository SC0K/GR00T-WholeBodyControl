#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/controller-env.sh"
"$MOCAP_ROOT/.venv_teleop/bin/cmake" -S "$MOCAP_ROOT/gear_sonic_deploy" \
    -B "$MOCAP_ROOT/.venv_teleop/mocap/native/build" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DTENSORRT_LIBRARY_DIR="$TensorRT_ROOT/targets/x86_64-linux-gnu/lib"
"$MOCAP_ROOT/.venv_teleop/bin/cmake" --build "$MOCAP_ROOT/.venv_teleop/mocap/native/build" \
    --target g1_deploy_onnx_ref -j4

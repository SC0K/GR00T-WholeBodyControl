#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/controller-env.sh"
cd "$SONIC_ROOT"
cmake -S gear_sonic_deploy -B gear_sonic_deploy/build \
 -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$SONIC_ROOT/.native" \
 -DTENSORRT_LIBRARY_DIR="$TensorRT_ROOT/lib" \
 -Donnxruntime_INCLUDE_DIR="$SONIC_ROOT/.local/onnxruntime-linux-x64-1.16.3/include" \
 -Donnxruntime_LIBRARY="$SONIC_ROOT/.local/onnxruntime-linux-x64-1.16.3/lib/libonnxruntime.so"
cmake --build gear_sonic_deploy/build --target g1_deploy_onnx_ref -j4

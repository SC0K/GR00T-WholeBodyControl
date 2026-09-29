#!/usr/bin/env bash
SONIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TensorRT_ROOT="$SONIC_ROOT/.local/TensorRT"
export CUDAToolkit_ROOT="$SONIC_ROOT/.native"
export HAS_ROS2=0
export PATH="$SONIC_ROOT/.native/bin:$PATH"
export LD_LIBRARY_PATH="$TensorRT_ROOT/lib:$SONIC_ROOT/.local/onnxruntime-linux-x64-1.16.3/lib:$SONIC_ROOT/.native/targets/x86_64-linux/lib:$SONIC_ROOT/.native/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

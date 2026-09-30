#!/usr/bin/env bash
MOCAP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export TensorRT_ROOT="$MOCAP_ROOT/.venv_teleop/mocap/native/TensorRT-10.13.3.9"
export CUDAToolkit_ROOT="$MOCAP_ROOT/.venv_teleop/mocap/native/cuda"
export HAS_ROS2=0
export LD_LIBRARY_PATH="$MOCAP_ROOT/gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/x86_64:$TensorRT_ROOT/targets/x86_64-linux-gnu/lib:$CUDAToolkit_ROOT/lib:/opt/onnxruntime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

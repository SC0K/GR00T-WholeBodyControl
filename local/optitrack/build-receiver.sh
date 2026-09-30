#!/usr/bin/env bash
set -euo pipefail
MOCAP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MOCAP_SDK="${NATNET_SDK_ROOT:-/home/sitongchen/keyLM_ros2_ws/src/crl-humanoid-ros/crl_optitrack_ros/optitrack_adaptor/ext/NatNetSDK_linux}"
MOCAP_PY="$MOCAP_ROOT/.venv_teleop/bin/python"
read -ra MOCAP_INCLUDES <<< "$("$MOCAP_PY" -m pybind11 --includes)"
MOCAP_SUFFIX="$("$MOCAP_PY" -c 'import sysconfig; print(sysconfig.get_config_var("EXT_SUFFIX"))')"
mkdir -p "$MOCAP_ROOT/.venv_teleop/mocap"
g++ -O2 -shared -std=c++17 -fPIC -fvisibility=hidden -pthread "${MOCAP_INCLUDES[@]}" \
    -I "$MOCAP_SDK/include" "$MOCAP_ROOT/local/optitrack/natnet_bindings.cpp" \
    -L "$MOCAP_SDK/lib" -Wl,-rpath,"$MOCAP_SDK/lib" -lNatNet \
    -o "$MOCAP_ROOT/.venv_teleop/mocap/_natnet$MOCAP_SUFFIX"

#!/usr/bin/env bash
# Receive-only NatNet diagnostic. Does not publish robot commands.
set -euo pipefail
MOCAP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MOCAP_SDK="${NATNET_SDK_ROOT:-/home/sitongchen/keyLM_ros2_ws/src/crl-humanoid-ros/crl_optitrack_ros/optitrack_adaptor/ext/NatNetSDK_linux}"
if [[ ! -f "$MOCAP_SDK/include/NatNetClient.h" || ! -f "$MOCAP_SDK/lib/libNatNet.so" ]]; then
    echo 'Set NATNET_SDK_ROOT to the installed Linux NatNet SDK directory.' >&2
    exit 2
fi
if [[ ! -x "$MOCAP_ROOT/.venv_teleop/bin/python" ]]; then
    echo 'Create .venv_teleop first; see local/TELEOP_ENVIRONMENT.md.' >&2
    exit 2
fi
mapfile -t MOCAP_IPS < <("$MOCAP_ROOT/.venv_teleop/bin/python" - "$MOCAP_ROOT/local/optitrack/setup.json" <<'PY'
import ipaddress
import json
import sys
with open(sys.argv[1]) as stream:
    source = json.load(stream)['source']
for key in ('server_ip', 'client_interface_ip'):
    print(ipaddress.IPv4Address(source[key]))
PY
)
if [[ "${#MOCAP_IPS[@]}" != 2 ]]; then
    echo 'Set valid server_ip and client_interface_ip in setup.json.' >&2
    exit 2
fi
mkdir -p "$MOCAP_ROOT/.venv_teleop/mocap"
g++ -std=c++17 -O2 -pthread -I "$MOCAP_SDK/include" \
    "$MOCAP_ROOT/local/optitrack/probe.cpp" \
    -L "$MOCAP_SDK/lib" -Wl,-rpath,"$MOCAP_SDK/lib" -lNatNet \
    -o "$MOCAP_ROOT/.venv_teleop/mocap/probe"
"$MOCAP_ROOT/.venv_teleop/mocap/probe" "${MOCAP_IPS[@]}" \
    | tee "$MOCAP_ROOT/.venv_teleop/mocap/probe.log"

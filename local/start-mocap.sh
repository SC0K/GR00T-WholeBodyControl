#!/usr/bin/env bash
set -euo pipefail
MOCAP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MOCAP_ROOT"
exec .venv_teleop/bin/python local/optitrack/bridge.py "$@"

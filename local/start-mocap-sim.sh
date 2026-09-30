#!/usr/bin/env bash
set -euo pipefail
MOCAP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$MOCAP_ROOT"
exec .venv_teleop/bin/python gear_sonic/scripts/run_sim_loop.py \
    --interface sim --auto-release-suspension "$@"

#!/usr/bin/env bash
set -euo pipefail
SONIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SONIC_ROOT"
exec .venv_sim/bin/python gear_sonic/scripts/run_sim_loop.py --interface sim --auto-release-suspension "$@"

#!/usr/bin/env bash
set -euo pipefail
SONIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SONIC_ROOT"
exec .venv_sim/bin/python -u local/quest/bridge.py --cert .local/quest-certs/cert.pem --key .local/quest-certs/key.pem "$@"

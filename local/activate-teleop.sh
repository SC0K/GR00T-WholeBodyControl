#!/usr/bin/env bash
# Source this file: source local/activate-teleop.sh
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo 'Use: source local/activate-teleop.sh' >&2
    exit 1
fi

_TELEOP_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$_TELEOP_REPO_ROOT/.venv_teleop/bin/activate" ]]; then
    echo 'Missing .venv_teleop; see local/TELEOP_ENVIRONMENT.md.' >&2
    unset _TELEOP_REPO_ROOT
    return 1
fi
if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi
source "$_TELEOP_REPO_ROOT/.venv_teleop/bin/activate"
unset _TELEOP_REPO_ROOT

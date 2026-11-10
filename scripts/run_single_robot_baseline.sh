#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
if ! command -v pixi >/dev/null 2>&1; then
    export PATH="$ROOT/../tools/bin:$PATH"
fi
command -v pixi >/dev/null 2>&1 || { echo 'Install Pixi and add it to PATH.' >&2; exit 1; }
[[ -f install/setup.bash ]] || { echo 'Run: pixi run colcon build --symlink-install' >&2; exit 1; }
eval "$(pixi shell-hook)"
source install/setup.bash
# Separate discovery from other robot sessions, including physical robots.
export ROS_DOMAIN_ID=71
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="edgeai_baseline_${USER}_$$"
exec python scripts/single_robot_baseline.py "$@"

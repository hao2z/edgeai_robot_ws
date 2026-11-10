#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/edgeai_autonomy_${UID}.lock"
flock -n 9 || { echo 'Another autonomy demo is already running.' >&2; exit 1; }
if ! command -v pixi >/dev/null 2>&1; then export PATH="$ROOT/../tools/bin:$PATH"; fi
command -v pixi >/dev/null 2>&1 || { echo 'Install Pixi and add it to PATH.' >&2; exit 1; }
eval "$(pixi shell-hook)"
[[ -f install/setup.bash ]] || { echo 'Build the workspace first.' >&2; exit 1; }
source install/setup.bash
export ROS_DOMAIN_ID=72
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="edgeai_autonomy_${USER}_$$"
exec ros2 run edgeai_autonomy demo "$@"

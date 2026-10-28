#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cache=${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}
if [[ ! -f "$cache/env/pixi.toml" || ! -f "$cache/frontier-install/local_setup.bash" ]]; then
    echo 'Nav2/frontier explorer overlay is missing. Run scripts/setup/prepare-open-source.sh first.' >&2
    exit 1
fi
export EDGEAI_GUI=true EDGEAI_RVIZ=true
export EDGEAI_ROS_DOMAIN=${4:-86}
exec "$HOME/.pixi/bin/pixi" run --manifest-path "$cache/env/pixi.toml" \
    bash "$check_dir/run-open-source-exploration.sh" "${1:-300}" "${2:-}" dwb "${3:-competition}" rectangle

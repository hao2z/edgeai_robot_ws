#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -eo pipefail
export COLUMNS=120 LINES=40
export PATH="$HOME/.pixi/bin:$PATH"
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
cd "$PROJECT_ROOT"
if [[ ${EDGEAI_ROS_ENV_ACTIVE:-false} != true ]]; then
    exec bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" env EDGEAI_ROS_ENV_ACTIVE=true bash "$0" "$@"
fi
if [[ ! -f "$EDGEAI_BUILD_ROOT/install/local_setup.bash" ]]; then
    echo 'Build this repository first: bash scripts/setup/build-workspace.sh' >&2
    exit 1
fi
source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
exec "$@"

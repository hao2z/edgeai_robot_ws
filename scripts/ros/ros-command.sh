#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -eo pipefail
export COLUMNS=120 LINES=40
export PATH="$HOME/.pixi/bin:$PATH"
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
cd "$PROJECT_ROOT"
eval "$(pixi shell-hook --manifest-path "$EDGEAI_ENV_MANIFEST")"
if [[ ! -f "$EDGEAI_BUILD_ROOT/install/local_setup.bash" ]]; then
    echo 'Build this repository first: bash scripts/setup/build-workspace.sh' >&2
    exit 1
fi
source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
exec "$@"

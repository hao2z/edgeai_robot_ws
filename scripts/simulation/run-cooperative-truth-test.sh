#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../ros/workspace-env.sh"
source "$PROJECT_ROOT/scripts/display/configure-desktop.sh"
source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
source "$EDGEAI_RUNTIME_CACHE/frontier-install/local_setup.bash"
exec python3 "$PROJECT_ROOT/scripts/simulation/cooperative_truth_test.py" "$@"

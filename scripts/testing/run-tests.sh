#!/usr/bin/env bash
set -eo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$project_root"
# Listing profiles needs no ROS installation or workspace setup.
for option in "$@"; do
    if [[ $option == --list || $option == --help || $option == -h ]]; then
        exec python3 tests/run.py "$@"
    fi
done
if [[ ${EDGEAI_TEST_ENV_ACTIVE:-false} != true ]]; then
    exec bash scripts/ros/ros-command.sh env EDGEAI_TEST_ENV_ACTIVE=true bash scripts/testing/run-tests.sh "$@"
fi
# Some authority tests use the external frontier service definitions.
if [[ -f "$EDGEAI_RUNTIME_CACHE/frontier-install/setup.bash" ]]; then
    source "$EDGEAI_RUNTIME_CACHE/frontier-install/setup.bash"
    source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
fi
exec python3 tests/run.py "$@"

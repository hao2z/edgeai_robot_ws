#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../ros/workspace-env.sh"
mkdir -p "$EDGEAI_BUILD_ROOT"
if [[ -f "$EDGEAI_RUNTIME_CACHE/frontier-install/setup.bash" ]]; then
    set +u
    source "$EDGEAI_RUNTIME_CACHE/frontier-install/setup.bash"
fi
bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" env ROS_DISTRO=jazzy \
    colcon --log-base "$EDGEAI_BUILD_ROOT/log" build \
    --base-paths "$PROJECT_ROOT/src" --build-base "$EDGEAI_BUILD_ROOT/build" \
    --install-base "$EDGEAI_BUILD_ROOT/install" --symlink-install "$@"

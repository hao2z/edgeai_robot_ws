#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../ros/workspace-env.sh"
mkdir -p "$EDGEAI_BUILD_ROOT"
pixi run --manifest-path "$EDGEAI_ENV_MANIFEST" env ROS_DISTRO=jazzy \
    colcon --log-base "$EDGEAI_BUILD_ROOT/log" build \
    --base-paths "$PROJECT_ROOT/src" --build-base "$EDGEAI_BUILD_ROOT/build" \
    --install-base "$EDGEAI_BUILD_ROOT/install" --symlink-install "$@"

#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Prepare a separate, persistent Pixi overlay; the existing workspace is not changed.
set -euo pipefail
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
if [[ $EDGEAI_ROS_BACKEND == native ]]; then
    bash "$PROJECT_ROOT/scripts/setup/build-frontier-runtime.sh"
    bash "$PROJECT_ROOT/scripts/setup/build-workspace.sh"
    exit 0
fi
cache=${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}
pixi="$HOME/.pixi/bin/pixi"
check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$cache/env"
if [[ ! -f "$cache/env/pixi.toml" ]]; then
    if [[ -f "$PROJECT_ROOT/src/edgeai_navigation/env/pixi.lock" ]]; then
        cp "$PROJECT_ROOT/src/edgeai_navigation/env/pixi.toml" "$PROJECT_ROOT/src/edgeai_navigation/env/pixi.lock" "$cache/env/"
    else
        cp "$PROJECT_ROOT/pixi.toml" "$PROJECT_ROOT/pixi.lock" "$cache/env/"
    fi
fi
if ! grep -q '^ros-jazzy-navigation2[[:space:]]*=' "$cache/env/pixi.toml"; then
    "$pixi" add --manifest-path "$cache/env/pixi.toml" ros-jazzy-navigation2 ros-jazzy-nav2-bringup
else
    "$pixi" install --manifest-path "$cache/env/pixi.toml" --locked
fi
if ! grep -q '^ros-jazzy-topic-tools[[:space:]]*=' "$cache/env/pixi.toml"; then
    "$pixi" add --manifest-path "$cache/env/pixi.toml" ros-jazzy-topic-tools
fi
frontier_rev=11864e4ac6789dc81fd71bad51e43339c01deca2
if [[ ! -d "$cache/frontier-upstream/.git" ]]; then
    git clone https://github.com/mertgulerx/frontier_exploration_ros2.git "$cache/frontier-upstream"
fi
git -C "$cache/frontier-upstream" checkout --detach "$frontier_rev"
bash "$PROJECT_ROOT/scripts/setup/build-frontier-runtime.sh"
echo "Prepared frontier_exploration_ros2 at $frontier_rev"
# Navigation/frontier parameters and RViz layouts are maintained in the repository.
# Environment setup must not regenerate or overwrite them.
EDGEAI_ENV_MANIFEST="$cache/env/pixi.toml" bash "$PROJECT_ROOT/scripts/setup/build-workspace.sh"

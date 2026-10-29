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
upstream_rev=326cf8a0b487c34246bb8f3326afbcd69576dc60
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
if [[ ! -d "$cache/upstream/.git" ]]; then
    git clone https://github.com/robo-friends/m-explore-ros2.git "$cache/upstream"
fi
git -C "$cache/upstream" checkout --detach "$upstream_rev"
if [[ $(git -C "$cache/upstream" rev-parse HEAD) != "$upstream_rev" ]]; then
    echo 'Explore Lite revision mismatch.' >&2
    exit 1
fi
"$pixi" run --manifest-path "$cache/env/pixi.toml" env ROS_DISTRO=jazzy \
    colcon --log-base "$cache/build-log" build \
    --base-paths "$cache/upstream/explore" "$cache/upstream/explore_lite_msgs" \
    --build-base "$cache/build" --install-base "$cache/install" \
    --cmake-args -DBUILD_TESTING=OFF
echo "Prepared pinned Explore Lite and Nav2 in $cache"
frontier_rev=11864e4ac6789dc81fd71bad51e43339c01deca2
if [[ ! -d "$cache/frontier-upstream/.git" ]]; then
    git clone https://github.com/mertgulerx/frontier_exploration_ros2.git "$cache/frontier-upstream"
fi
git -C "$cache/frontier-upstream" checkout --detach "$frontier_rev"
bash "$PROJECT_ROOT/scripts/setup/build-frontier-runtime.sh"
"$pixi" run --manifest-path "$cache/env/pixi.toml" python3 "$PROJECT_ROOT/src/edgeai_exploration/tools/prepare_frontier_config.py" \
    "$cache/frontier-upstream/config/params.yaml" \
    "$PROJECT_ROOT/src/edgeai_exploration/config/frontier_exploration.yaml"
echo "Prepared frontier_exploration_ros2 at $frontier_rev"
"$pixi" run --manifest-path "$cache/env/pixi.toml" python3 "$PROJECT_ROOT/src/edgeai_navigation/tools/prepare_nav2_config.py" \
    "$cache/env/.pixi/envs/default/share/nav2_bringup/params/nav2_params.yaml" \
    "$PROJECT_ROOT/src/edgeai_navigation/config/nav2_exploration.yaml"
EDGEAI_ENV_MANIFEST="$cache/env/pixi.toml" bash "$PROJECT_ROOT/scripts/setup/build-workspace.sh"

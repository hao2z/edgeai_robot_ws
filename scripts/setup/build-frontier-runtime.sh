#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../ros/workspace-env.sh"
cache=$EDGEAI_RUNTIME_CACHE
mkdir -p "$cache"
if [[ ! -d "$cache/frontier-upstream" ]]; then
    if [[ -d "$HOME/.cache/edgeai-open-source/frontier-upstream/.git" ]]; then
        git clone --no-hardlinks "$HOME/.cache/edgeai-open-source/frontier-upstream" "$cache/frontier-upstream"
    else
        git clone https://github.com/mertgulerx/frontier_exploration_ros2.git "$cache/frontier-upstream"
        git -C "$cache/frontier-upstream" checkout --detach 11864e4ac6789dc81fd71bad51e43339c01deca2
    fi
fi
revision=11864e4ac6789dc81fd71bad51e43339c01deca2
[[ $(git -C "$cache/frontier-upstream" rev-parse HEAD) == "$revision" ]] || {
    echo 'Frontier upstream revision mismatch.' >&2; exit 1;
}
python3 "$PROJECT_ROOT/src/edgeai_exploration/tools/prepare_frontier_runtime.py" \
    "$cache/frontier-upstream" "$cache/frontier-coordinated-src"
bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" env ROS_DISTRO=jazzy CMAKE_BUILD_PARALLEL_LEVEL=2 \
    colcon --log-base "$cache/frontier-log" build --base-paths "$cache/frontier-coordinated-src" \
    --build-base "$cache/frontier-coordinated-build" --install-base "$cache/frontier-install" \
    --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
printf '%s\n' "$revision repeat-gain-v3" > "$cache/frontier-install/edgeai-recovery-owner"

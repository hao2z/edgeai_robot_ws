#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../ros/workspace-env.sh"
if [[ $EDGEAI_ROS_BACKEND == native ]]; then
    cache=$EDGEAI_RUNTIME_CACHE
    revision=d5d768b7b38a05e7cfd2b6c693b786b402161bed
    mkdir -p "$cache"
    upstream_source="$HOME/.cache/edgeai-open-source/nav2-primitives"
    [[ $(git -C "$upstream_source" rev-parse HEAD) == "$revision" ]]
    source_path="$cache/mppi-fixed-source"
    python3 "$PROJECT_ROOT/scripts/setup/prepare-mppi-backport.py" \
        "$upstream_source/nav2_mppi_controller" "$source_path"
    bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" env MAKEFLAGS='-j1 -l2' CMAKE_BUILD_PARALLEL_LEVEL=1 \
        colcon --log-base "$cache/mppi-log" build --base-paths "$source_path" \
        --build-base "$cache/mppi-fixed-build" --install-base "$cache/mppi-install" \
        --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
        "-DEDGEAI_BACKPORT_TEST_SOURCE=$PROJECT_ROOT/tests/exploration/mppi_backport_check.cpp"
    bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" bash -c \
        'source "$1/local_setup.bash"; exec "$2"' -- "$cache/mppi-install" \
        "$cache/mppi-fixed-build/nav2_mppi_controller/edgeai_mppi_backport_check"
    printf '%s\n' "$revision" > "$cache/mppi-install/edgeai-mppi-revision"
    cp "$source_path/edgeai-backport-revision" "$cache/mppi-install/edgeai-mppi-backport"
    exit 0
fi
cache=${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}
revision=d5d768b7b38a05e7cfd2b6c693b786b402161bed
[[ $(git -C "$cache/nav2-primitives" rev-parse HEAD) == "$revision" ]]
git -C "$cache/nav2-primitives" sparse-checkout add nav2_mppi_controller
deps="$cache/mppi-deps"
mkdir -p "$deps/src"
# Use a matching header-only dependency set, separate from the installed ROS
# environment, whose xtensor/xsimd headers are incompatible for a source build.
for spec in 'xtl 0.7.7' 'xsimd 10.0.0' 'xtensor 0.24.7'; do
    read -r name version <<< "$spec"
    source_dir="$deps/src/$name-$version"
    if [[ ! -d "$source_dir/.git" ]]; then
        git clone --depth 1 --branch "$version" "https://github.com/xtensor-stack/$name.git" "$source_dir"
    fi
    pixi run --manifest-path "$EDGEAI_ENV_MANIFEST" cmake -S "$source_dir" -B "$deps/build/$name" \
        -DCMAKE_INSTALL_PREFIX="$deps/install" -DCMAKE_PREFIX_PATH="$deps/install" -DBUILD_TESTS=OFF \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    pixi run --manifest-path "$EDGEAI_ENV_MANIFEST" cmake --install "$deps/build/$name"
done
source_path="$cache/mppi-fixed-source"
python3 "$PROJECT_ROOT/scripts/setup/prepare-mppi-backport.py" \
    "$cache/nav2-primitives/nav2_mppi_controller" "$source_path"
build_path="$cache/mppi-fixed-build"
if [[ ${EDGEAI_MPPI_SCORE_PROBE_BUILD:-false} == true ]]; then
    python3 "$PROJECT_ROOT/tests/exploration/prepare-mppi-score-probe.py" \
        "$source_path" "$cache/mppi-probe-source"
    source_path="$cache/mppi-probe-source"
    build_path="$cache/mppi-probe-build"
fi
pixi run --manifest-path "$EDGEAI_ENV_MANIFEST" env ROS_DISTRO=jazzy MAKEFLAGS='-j1 -l2' CMAKE_BUILD_PARALLEL_LEVEL=1 \
    colcon --log-base "$cache/mppi-log" build --base-paths "$source_path" \
    --build-base "$build_path" --install-base "$cache/mppi-install" \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
    "-DEDGEAI_BACKPORT_TEST_SOURCE=$PROJECT_ROOT/tests/exploration/mppi_backport_check.cpp" \
    "-DCMAKE_PREFIX_PATH=$deps/install" "-DCMAKE_CXX_FLAGS=-I$deps/install/include"
pixi run --manifest-path "$EDGEAI_ENV_MANIFEST" "$build_path/nav2_mppi_controller/edgeai_mppi_backport_check"
printf '%s\n' "$revision" > "$cache/mppi-install/edgeai-mppi-revision"
cp "$source_path/edgeai-backport-revision" "$cache/mppi-install/edgeai-mppi-backport"

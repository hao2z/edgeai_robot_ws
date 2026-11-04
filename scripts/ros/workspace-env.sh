#!/usr/bin/env bash
# Source this helper; the repository is the only editable source tree.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PROJECT_ROOT
export EDGEAI_ROS_BACKEND=${EDGEAI_ROS_BACKEND:-$(cat "$PROJECT_ROOT/runtime/ros-backend" 2>/dev/null || echo native)}
if [[ $EDGEAI_ROS_BACKEND == native ]]; then
    export EDGEAI_BUILD_ROOT=${EDGEAI_BUILD_ROOT:-$HOME/.cache/edgeai-native-build}
    export EDGEAI_RUNTIME_CACHE=${EDGEAI_NATIVE_CACHE:-$HOME/.cache/edgeai-native}
    export EDGEAI_GZ_GUI_CONFIG=/usr/share/gz/gz-sim8/gui/gui.config
elif [[ $EDGEAI_ROS_BACKEND == pixi ]]; then
    export EDGEAI_BUILD_ROOT=${EDGEAI_BUILD_ROOT:-$HOME/.cache/edgeai-robot-build}
    export EDGEAI_RUNTIME_CACHE=${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}
    export EDGEAI_GZ_GUI_CONFIG="$EDGEAI_RUNTIME_CACHE/env/.pixi/envs/default/share/gz/gz-sim8/gui/gui.config"
else
    echo 'EDGEAI_ROS_BACKEND must be native or pixi.' >&2; return 1
fi
export PATH="$HOME/.pixi/bin:$PATH"
if [[ -z ${EDGEAI_ENV_MANIFEST:-} ]]; then
    navigation_manifest="${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}/env/pixi.toml"
    if [[ -f $navigation_manifest ]]; then
        export EDGEAI_ENV_MANIFEST="$navigation_manifest"
    else
        export EDGEAI_ENV_MANIFEST="$PROJECT_ROOT/pixi.toml"
    fi
fi

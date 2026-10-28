#!/usr/bin/env bash
# Source this helper; the repository is the only editable source tree.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PROJECT_ROOT
export EDGEAI_BUILD_ROOT=${EDGEAI_BUILD_ROOT:-$HOME/.cache/edgeai-robot-build}
export PATH="$HOME/.pixi/bin:$PATH"
if [[ -z ${EDGEAI_ENV_MANIFEST:-} ]]; then
    navigation_manifest="${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}/env/pixi.toml"
    if [[ -f $navigation_manifest ]]; then
        export EDGEAI_ENV_MANIFEST="$navigation_manifest"
    else
        export EDGEAI_ENV_MANIFEST="$PROJECT_ROOT/pixi.toml"
    fi
fi

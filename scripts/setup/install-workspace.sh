#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PATH="$HOME/.pixi/bin:$PATH"
if ! command -v pixi >/dev/null; then
    curl -fsSL https://pixi.sh/install.sh -o /tmp/edgeai-install-pixi.sh
    PIXI_NO_TELEMETRY=1 bash /tmp/edgeai-install-pixi.sh
fi
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
pixi install --manifest-path "$EDGEAI_ENV_MANIFEST" --locked
exec bash "$PROJECT_ROOT/scripts/setup/build-workspace.sh"

#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
export COLUMNS=120 LINES=40
export PATH="$HOME/.pixi/bin:$PATH"
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
cd "$PROJECT_ROOT"
mkdir -p "$PROJECT_ROOT/runtime"
exec bash "$PROJECT_ROOT/scripts/simulation/run-simulation.sh" wslg

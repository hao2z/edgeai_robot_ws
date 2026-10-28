#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
CHECK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$PROJECT_ROOT/runtime/simulation.pid" ]]; then
    sim_pid=$(cat "$PROJECT_ROOT/runtime/simulation.pid")
    if [[ -r /proc/$sim_pid/cmdline ]] && grep -zq 'run-simulation.sh' "/proc/$sim_pid/cmdline"; then
        kill -TERM "$sim_pid"
        for _ in {1..350}; do
            [[ -r /proc/$sim_pid/cmdline ]] && grep -zq 'run-simulation.sh' "/proc/$sim_pid/cmdline" || exit 0
            sleep .1
        done
        echo 'Previous simulator did not finish cleanup; refusing to start another.' >&2
        exit 1
    fi
fi

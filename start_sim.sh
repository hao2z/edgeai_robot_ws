#!/usr/bin/env bash
set -e

# Workspace directory
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# Activate Pixi environment
source "$WS_DIR/scripts/ros/workspace-env.sh"
if [[ ${EDGEAI_ROS_ENV_ACTIVE:-false} != true ]]; then
    exec bash "$WS_DIR/scripts/ros/with-ros-env.sh" env EDGEAI_ROS_ENV_ACTIVE=true bash "$0" "$@"
fi

# Source ROS 2 workspace
if [[ ! -f "$EDGEAI_BUILD_ROOT/install/local_setup.bash" ]]; then
    echo "Workspace has not been built."
    echo "Run: bash scripts/setup/build-workspace.sh"
    exit 1
fi

source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"

# Register cleanup before starting any background processes.
CHILD_PIDS=()
cleanup() {
    trap - EXIT INT TERM
    if (( ${#CHILD_PIDS[@]} )); then
        kill "${CHILD_PIDS[@]}" 2>/dev/null || true
        wait "${CHILD_PIDS[@]}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting EdgeAI realistic robot simulation..."

ros2 launch edgeai_simulation simulation_real.launch.py rviz:=true "$@" &
SIM_PID=$!
CHILD_PIDS+=("$SIM_PID")

wait "$SIM_PID"

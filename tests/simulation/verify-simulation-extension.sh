#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -eo pipefail
export PATH="$HOME/.pixi/bin:$PATH"
export COLUMNS=120 LINES=40
CHECK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Use native WSLg display for sensor rendering, as requested.
source "$PROJECT_ROOT/scripts/display/configure-desktop.sh"
if [[ ${EDGEAI_TEST_DISPLAY:-wslg} == vcxsrv ]]; then
    export DISPLAY="$(ip -4 route show default | awk 'NR==1 {print $3}'):1"
    export XAUTHORITY="$PROJECT_ROOT/runtime/vcxsrv.xauth" QT_X11_NO_MITSHM=1
    unset WAYLAND_DISPLAY
fi
timeout 5s xdpyinfo >/dev/null
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
cd "$PROJECT_ROOT"
if [[ ${EDGEAI_ROS_ENV_ACTIVE:-false} != true ]]; then
    exec bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" env EDGEAI_ROS_ENV_ACTIVE=true bash "$0" "$@"
fi
source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
mkdir -p "$HOME/.cache" "$PROJECT_ROOT/tests/results/simulation-extension"
exec 9>"$HOME/.cache/edgeai-simulation.lock"
flock -n 9 || { echo 'Another simulation is running'; exit 1; }
count=${1:-2}
mode=${2:-multi}
multi=true
extras=(--count "$count" --maps)
if [[ $mode == single ]]; then multi=false; extras+=(--single); fi
if [[ $mode == motion ]]; then extras+=(--motion); fi
if [[ ! $count =~ ^[1-4]$ || ! $mode =~ ^(single|multi|motion)$ ]]; then
    echo 'Usage: verify-simulation-extension.sh [1..4] [single|multi|motion]' >&2; exit 2
fi
# Development sensor fixture is intentional: this is not the midterm mission.
sim_share="$PROJECT_ROOT/src/edgeai_simulation"
setsid ros2 launch edgeai_simulation simulation_real.launch.py \
  multi_robot:="$multi" robot_count:="$count" gui:=false rviz:=false slam:=true \
  robots_file:="$sim_share/config/robots.yaml" targets_file:="$sim_share/config/targets.yaml" \
  world_file:="$sim_share/worlds/empty.sdf" \
  >"$PROJECT_ROOT/tests/results/simulation-extension/launch-$mode-$count.log" 2>&1 &
sim_pid=$!
cleanup() {
    trap - EXIT INT TERM
    kill -INT -- "-$sim_pid" 2>/dev/null || true
    for _ in {1..30}; do
        if ! kill -0 "$sim_pid" 2>/dev/null; then break; fi
        sleep 0.2
    done
    kill -TERM -- "-$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
timeout --signal=INT --kill-after=5s 120s python3 "$CHECK_DIR/verify_simulation_extension.py" "${extras[@]}" \
  --output "$PROJECT_ROOT/tests/results/simulation-extension/result-$mode-$count.json"

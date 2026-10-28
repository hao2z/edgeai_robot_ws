#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
export PATH="$HOME/.pixi/bin:$PATH"
export COLUMNS=120 LINES=40
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
CHECK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ${1:-vcxsrv} == vcxsrv ]]; then
    HOST_IP=$(ip -4 route show default | awk 'NR==1 {print $3}')
    export DISPLAY="$HOST_IP:1"
    export XAUTHORITY="$PROJECT_ROOT/runtime/vcxsrv.xauth"
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    unset WAYLAND_DISPLAY
    timeout 10s xdpyinfo >/dev/null
else
    export DISPLAY=:0 QT_QPA_PLATFORM=xcb
    unset XAUTHORITY
    if ! timeout 5s xdpyinfo >/dev/null 2>&1; then
        echo 'WSLg X11 display :0 is unavailable. Restore WSLg, or explicitly use -DisplayServer VcXsrv.' >&2
        exit 1
    fi
fi
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
cd "$PROJECT_ROOT"
mkdir -p "$PROJECT_ROOT/runtime"
mkdir -p "$HOME/.cache"
exec 9>"$HOME/.cache/edgeai-simulation.lock"
flock -n 9 || { echo 'An EdgeAI simulation is already running.'; exit 1; }
echo $$ > "$PROJECT_ROOT/runtime/simulation.pid"
session=${EDGEAI_SESSION_DIR:-$PROJECT_ROOT/tests/results/runs/$(date +%Y%m%d-%H%M%S)-simulation-$$}
mkdir -p "$session"
export ROS_LOG_DIR="$session/ros" PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
printf '%s\n' "$session" > "$PROJECT_ROOT/runtime/latest-session.txt"
for name in sim nav explore rviz rosout watch; do touch "$session/$name.log"; done
python3 -u "$PROJECT_ROOT/scripts/ros/stream_session_logs.py" "$session" 2>"$session/stream-errors.log" &
STREAM_PID=$!
SIM_GROUP=''
cleanup() {
    result=$?
    trap - EXIT INT TERM
    touch "$session/final-capture"
    if [[ ${EDGEAI_CAPTURE_ACK:-false} == true ]]; then
        for _ in {1..20}; do
            [[ ! -f "$session/capture-finished" ]] || break
            sleep 1
        done
    fi
    if [[ -n $SIM_GROUP ]]; then
        kill -TERM -- "-$SIM_GROUP" 2>/dev/null || true
        wait "$SIM_GROUP" 2>/dev/null || true
    fi
    touch "$session/stream.stop"
    wait "$STREAM_PID" 2>/dev/null || true
    printf '%s\n' "$result" > "$session/exit-code.txt"
    rm -f "$PROJECT_ROOT/runtime/simulation.pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 "$PROJECT_ROOT/simulation/tools/prepare_observation_world.py" \
    "$PROJECT_ROOT/src/edgeai_robot_description/worlds/empty.sdf" "$session/observation.sdf" \
    "$(dirname "$EDGEAI_ENV_MANIFEST")/.pixi/envs/default/share/gz/gz-sim8/gui/gui.config"
setsid bash ./start_sim.sh "${@:2}" "world_file:=$session/observation.sdf" >"$session/sim.log" 2>&1 &
SIM_GROUP=$!
wait "$SIM_GROUP"

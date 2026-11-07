#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
export PATH="$HOME/.pixi/bin:$PATH"
export COLUMNS=120 LINES=40
CHECK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ${1:-auto} == vcxsrv ]]; then
    HOST_IP=$(ip -4 route show default | awk 'NR==1 {print $3}')
    export DISPLAY="$HOST_IP:1"
    export XAUTHORITY="$PROJECT_ROOT/runtime/vcxsrv.xauth"
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    unset WAYLAND_DISPLAY
    timeout 10s xdpyinfo >/dev/null
else
    source "$PROJECT_ROOT/scripts/display/configure-desktop.sh"
    if ! timeout 5s xdpyinfo >/dev/null 2>&1; then
        echo 'X11 display is unavailable. Check DISPLAY/XAUTHORITY and the desktop session, or select VcXsrv explicitly on Windows.' >&2
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
export EDGEAI_TRUTH_DIRECTORY="$session"
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
sim_share="$PROJECT_ROOT/src/edgeai_simulation"
arena_args=("targets_file:=$sim_share/config/targets.yaml"
            "robots_file:=$sim_share/config/robots.yaml" 'use_robot_pose:=false')
world_source="$sim_share/worlds/empty.sdf"
camera_pose='3.8 -4.35 6 0 0.80 2.288'
case ${EDGEAI_SIM_ARENA:-midterm} in
    simple) ;;
    midterm)
        world_source="$sim_share/worlds/midterm.sdf"
        camera_pose='2.5 -3.8 7 0 0.85 1.57079632679'
        arena_args=("targets_file:=$sim_share/config/targets_midterm.yaml"
                    "robots_file:=$sim_share/config/robots_midterm.yaml" 'use_robot_pose:=true')
        ;;
    *) echo 'EDGEAI_SIM_ARENA must be simple or midterm.' >&2; exit 2 ;;
esac
python3 "$sim_share/tools/prepare_observation_world.py" \
    "$world_source" "$session/observation.sdf" "$EDGEAI_GZ_GUI_CONFIG" "$camera_pose"
setsid bash ./start_sim.sh "${@:2}" "${arena_args[@]}" "world_file:=$session/observation.sdf" >"$session/sim.log" 2>&1 &
SIM_GROUP=$!
wait "$SIM_GROUP"

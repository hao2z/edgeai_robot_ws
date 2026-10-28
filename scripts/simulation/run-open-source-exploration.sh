#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Run with the isolated Pixi environment activated; no known arena map is loaded.
set -euo pipefail
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=${EDGEAI_ROS_DOMAIN:-87}
exec 9>"$HOME/.cache/edgeai-autonomous-$ROS_DOMAIN_ID.lock"
flock -n 9 || { echo 'An exploration session already uses this ROS domain.' >&2; exit 1; }
export GZ_PARTITION="edgeai_nav2_${ROS_DOMAIN_ID}_$$"
export DISPLAY=:0 QT_QPA_PLATFORM=xcb
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset XAUTHORITY
set +u  # colcon-generated setup scripts expect some variables to be unset.
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
backend=${EDGEAI_EXPLORER:-frontier}
cache=${EDGEAI_NAV_CACHE:-$HOME/.cache/edgeai-open-source}
if [[ $backend == frontier ]]; then
    source "$cache/frontier-install/local_setup.bash"
elif [[ $backend == lite ]]; then
    source "$cache/install/local_setup.bash"
else
    echo "Unknown exploration backend: $backend" >&2; exit 2
fi
set -u
duration=${1:-120}
map_basename=${2:-}
controller=${3:-dwb}
arena=${4:-simple}
profile=${5:-baseline}
gui=${EDGEAI_GUI:-true}
rviz=${EDGEAI_RVIZ:-true}
check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_dir="$PROJECT_ROOT/src/edgeai_robot_description/config"
arena_args=()
if [[ $arena == competition ]]; then
    arena_args=("targets_file:=$config_dir/targets_competition.yaml")
elif [[ $arena != simple ]]; then
    echo "Unknown arena: $arena" >&2; exit 2
fi
case "$controller" in
    mppi) nav_config="$config_dir/nav2_exploration.yaml" ;;
    dwb) nav_config="$config_dir/nav2_exploration_dwb.yaml" ;;
    *) echo "Unknown controller: $controller" >&2; exit 2 ;;
esac
explore_config="$config_dir/explore_lite.yaml"
if [[ $profile == rectangle ]]; then
    nav_config="$config_dir/nav2_exploration_rectangle.yaml"
elif [[ $profile == conservative ]]; then
    nav_config="$config_dir/nav2_exploration_dwb_safe.yaml"
elif [[ $profile == safe ]]; then
    nav_config="$config_dir/nav2_exploration_dwb_safe.yaml"
    explore_config="$config_dir/explore_lite_safe.yaml"
elif [[ $profile != baseline ]]; then
    echo "Unknown profile: $profile" >&2; exit 2
fi
temp_dir=${EDGEAI_SESSION_DIR:-$PROJECT_ROOT/tests/results/runs/$(date +%Y%m%d-%H%M%S)-domain$ROS_DOMAIN_ID-$$}
mkdir -p "$temp_dir" "$PROJECT_ROOT/runtime"
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
export ROS_LOG_DIR="$temp_dir/ros"
python - "$nav_config" "$temp_dir/nav2-runtime.yaml" <<'PY'
from pathlib import Path
import sys, yaml
source, output = map(Path, sys.argv[1:])
config = yaml.safe_load(source.read_text())
planner = config['planner_server']['ros__parameters']['GridBased']
if 'lattice_filepath' in planner:
    planner['lattice_filepath'] = str((source.parent / planner['lattice_filepath']).resolve())
output.write_text(yaml.safe_dump(config, sort_keys=False))
PY
nav_config="$temp_dir/nav2-runtime.yaml"
for name in sim nav explore rviz rosout watch; do touch "$temp_dir/$name.log"; done
event() { printf '%s [session] %s\n' "$(date --iso-8601=seconds)" "$*" | tee -a "$temp_dir/events.log"; }
registration="$PROJECT_ROOT/runtime/exploration-$ROS_DOMAIN_ID.json"
python - "$registration" "$temp_dir" "$$" "$GZ_PARTITION" "$arena" "$profile" "$duration" <<'PY'
import json, pathlib, sys, os
registration, directory, pid, partition, arena, profile, duration = sys.argv[1:]
record = dict(pid=int(pid), start_ticks=pathlib.Path(f'/proc/{pid}/stat').read_text().split()[21],
              directory=directory, partition=partition, domain=os.environ['ROS_DOMAIN_ID'],
              arena=arena, profile=profile, duration=int(duration))
for path in (pathlib.Path(registration), pathlib.Path(directory)/'session.json'):
    path.write_text(json.dumps(record, indent=2))
PY
printf '%s\n' "$temp_dir" > "$PROJECT_ROOT/runtime/latest-session.txt"
python -u "$PROJECT_ROOT/scripts/ros/stream_session_logs.py" "$temp_dir" 2>"$temp_dir/stream-errors.log" &
tail_pid=$!
sim_pid='' nav_pid='' explore_pid='' watch_pid='' rviz_pid='' monitor_pid=''
cleanup() {
    result=$?
    trap - EXIT INT TERM
    if [[ ! -f "$temp_dir/final-capture" && ${EDGEAI_CAPTURE_ACK:-false} == true ]]; then
        touch "$temp_dir/final-capture"
        for _ in {1..15}; do
            [[ ! -f "$temp_dir/capture-finished" ]] || break
            sleep 1
        done
    fi
    touch "$temp_dir/stopping"
    event "正在关闭本次测试的仿真、导航与探索进程（退出码 $result），日志保留在 $temp_dir"
    [[ -z $monitor_pid ]] || kill -INT "$monitor_pid" 2>/dev/null || true
    [[ -z $watch_pid ]] || kill -INT "$watch_pid" 2>/dev/null || true
    [[ -z $watch_pid ]] || wait "$watch_pid" 2>/dev/null || true
    [[ -z $rviz_pid ]] || kill -TERM -- "-$rviz_pid" 2>/dev/null || true
    for pid in "$explore_pid" "$nav_pid" "$sim_pid"; do
        [[ -z $pid ]] || kill -INT -- "-$pid" 2>/dev/null || true
    done
    for _ in {1..30}; do
        alive=false
        for pid in "$explore_pid" "$nav_pid" "$sim_pid"; do
            if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then alive=true; fi
        done
        $alive || break
        sleep .2
    done
    for pid in "$explore_pid" "$nav_pid" "$sim_pid"; do
        [[ -z $pid ]] || kill -TERM -- "-$pid" 2>/dev/null || true
    done
    [[ -z $monitor_pid ]] || kill -TERM "$monitor_pid" 2>/dev/null || true
    touch "$temp_dir/stream.stop"
    wait "$tail_pid" 2>/dev/null || true
    rm -f -- "$registration"
    printf '%s\n' "$result" > "$temp_dir/exit-code.txt"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
event "启动测试：ROS 域 $ROS_DOMAIN_ID，场地 $arena，轮廓 $profile；所有原始输出实时保存"
# Use the installed native GUI plugin configuration; change only the view.
world_source="$config_dir/../worlds/empty.sdf"
[[ $arena != competition ]] || world_source="$config_dir/../worlds/competition_rooms.sdf"
python "$PROJECT_ROOT/simulation/tools/prepare_observation_world.py" "$world_source" "$temp_dir/observation.sdf" \
    "$cache/env/.pixi/envs/default/share/gz/gz-sim8/gui/gui.config"
arena_args+=("world_file:=$temp_dir/observation.sdf")
python -u "$PROJECT_ROOT/scripts/ros/monitor_ros.py" >"$temp_dir/rosout.log" 2>&1 &
monitor_pid=$!

setsid ros2 launch edgeai_robot_description simulation_real.launch.py \
    multi_robot:=false robot_count:=1 gui:="$gui" rviz:=false slam:=true "${arena_args[@]}" >"$temp_dir/sim.log" 2>&1 &
sim_pid=$!
setsid ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true autostart:=true \
    use_composition:=False params_file:="$nav_config" >"$temp_dir/nav.log" 2>&1 &
nav_pid=$!
if [[ $rviz == true ]]; then
    setsid ros2 run rviz2 rviz2 -d "$config_dir/../rviz/exploration.rviz" \
        --ros-args -p use_sim_time:=true >"$temp_dir/rviz.log" 2>&1 &
    rviz_pid=$!
fi
echo "Starting SLAM and Nav2 in isolated ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
ready=false
for _ in {1..60}; do
    if ! kill -0 "$sim_pid" 2>/dev/null || ! kill -0 "$nav_pid" 2>/dev/null; then break; fi
    if timeout 3s ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active '; then
        ready=true
        break
    fi
    sleep 1
done
if ! $ready; then
    echo 'Nav2 did not become active. Recent simulator and navigation output:' >&2
    event '启动失败：Nav2 未进入 active 状态，请查看 sim.log 和 nav.log'
    tail -n 35 "$temp_dir/sim.log" >&2
    tail -n 80 "$temp_dir/nav.log" >&2
    exit 1
fi
echo "Nav2 is active; launching upstream explorer: $backend"
event 'Nav2 已就绪，开始自主探索'
touch "$temp_dir/ready"
if [[ $backend == frontier ]]; then
    python -u "$PROJECT_ROOT/algorithms/exploration/watch_exploration_completion.py" "$temp_dir/completed" >"$temp_dir/watch.log" 2>&1 &
    watch_pid=$!
    setsid ros2 run frontier_exploration_ros2 frontier_explorer --ros-args \
        --params-file "$config_dir/frontier_exploration.yaml" >"$temp_dir/explore.log" 2>&1 &
else
    setsid ros2 run explore_lite explore --ros-args --params-file "$explore_config" >"$temp_dir/explore.log" 2>&1 &
fi
explore_pid=$!
start=$SECONDS
reason=time_limit
while (( SECONDS-start < duration )); do
    if ! kill -0 "$sim_pid" 2>/dev/null || ! kill -0 "$nav_pid" 2>/dev/null || ! kill -0 "$explore_pid" 2>/dev/null; then
        echo 'A required process stopped before the test ended.' >&2
        event '必要进程提前退出，测试失败'
        tail -n 50 "$temp_dir/nav.log" >&2
        exit 1
    fi
    if [[ $backend == frontier && -f "$temp_dir/completed" ]]; then
        reason=upstream_completion_event
        break
    fi
    if [[ $backend == lite ]] && grep -q 'No frontiers found, stopping' "$temp_dir/explore.log"; then
        reason=no_frontiers
        break
    fi
    if [[ $backend == lite ]] && grep -q 'All frontiers traversed/tried out, stopping' "$temp_dir/explore.log"; then
        reason=all_frontiers_tried_including_blacklisted
        break
    fi
    sleep 2
done
event "探索阶段结束：$reason，用时 $((SECONDS-start)) 秒；开始截图与地图评估"
touch "$temp_dir/stopping"
kill -INT -- "-$explore_pid" 2>/dev/null || true
kill -INT -- "-$nav_pid" 2>/dev/null || true
timeout 5s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
touch "$temp_dir/final-capture"
# The Windows observer acknowledges both captures before GUI processes exit.
if [[ ${EDGEAI_CAPTURE_ACK:-false} == true ]]; then
    for _ in {1..30}; do
        [[ ! -f "$temp_dir/capture-finished" ]] || break
        sleep 1
    done
    [[ -f "$temp_dir/capture-finished" ]] || event '最终截图等待超时，详见 screenshots/capture.log'
fi
for _ in {1..20}; do
    kill -0 "$nav_pid" 2>/dev/null || break
    sleep .1
done
kill -TERM -- "-$nav_pid" 2>/dev/null || true
timeout 5s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
echo "Trial stopped: reason=$reason; elapsed=$((SECONDS-start)) seconds; controller=$controller."
if [[ -n $map_basename ]]; then
    mkdir -p "$(dirname "$map_basename")"
    timeout 25s ros2 run nav2_map_server map_saver_cli -f "$map_basename" --ros-args -p use_sim_time:=true >"$temp_dir/map-save.log" 2>&1
    if [[ $arena == competition ]]; then
        python "$PROJECT_ROOT/tests/exploration/evaluate_arena_map.py" "$map_basename.yaml" "$config_dir/../worlds/competition_rooms.sdf" | tee "$temp_dir/evaluation.log"
    else
        python "$PROJECT_ROOT/tests/exploration/evaluate_exploration_map.py" "$map_basename.yaml" | tee "$temp_dir/evaluation.log"
    fi
fi
echo 'Recent navigation events:'
grep -E 'Begin navigating|Goal succeeded|Goal failed|Failed to make progress|Aborting' "$temp_dir/nav.log" | tail -n 25 || true

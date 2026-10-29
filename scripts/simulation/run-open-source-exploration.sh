#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# Run with the isolated Pixi environment activated; no known arena map is loaded.
set -euo pipefail
ulimit -c 0
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=${EDGEAI_ROS_DOMAIN:-87}
exec 9>"$HOME/.cache/edgeai-autonomous-$ROS_DOMAIN_ID.lock"
flock -n 9 || { echo 'An exploration session already uses this ROS domain.' >&2; exit 1; }
export GZ_PARTITION="edgeai_nav2_${ROS_DOMAIN_ID}_$$"
export DISPLAY=:0 QT_QPA_PLATFORM=xcb
export GALLIUM_DRIVER=${EDGEAI_GALLIUM_DRIVER:-d3d12} MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
unset XAUTHORITY
set +u  # colcon-generated setup scripts expect some variables to be unset.
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
source "$EDGEAI_BUILD_ROOT/install/local_setup.bash"
backend=${EDGEAI_EXPLORER:-frontier}
cache=$EDGEAI_RUNTIME_CACHE
if [[ $backend == frontier ]]; then
    if [[ ! -f "$cache/frontier-install/edgeai-recovery-owner" ]] ||
       ! grep -qx '11864e4ac6789dc81fd71bad51e43339c01deca2 repeat-gain-v3' "$cache/frontier-install/edgeai-recovery-owner"; then
        echo 'Build the coordinated explorer first: bash scripts/setup/build-frontier-runtime.sh' >&2
        exit 1
    fi
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
if [[ $controller == mppi && ( $EDGEAI_ROS_BACKEND == pixi || ${EDGEAI_MPPI_USE_SOURCE:-false} == true ) ]]; then
    if ! grep -qx 'ec1c42d36e85f70574aaa21a9f587585d1de3258' "$cache/mppi-install/edgeai-mppi-backport" 2>/dev/null; then
        echo 'MPPI requires the low-acceleration backport: bash scripts/setup/build-mppi-runtime.sh' >&2
        exit 1
    fi
    if ! grep -qx 'strict-math-v1' "$cache/mppi-install/edgeai-mppi-backport"; then
        echo 'Rebuild MPPI with the numerical math fix: bash scripts/setup/build-mppi-runtime.sh' >&2
        exit 1
    fi
    set +u
    source "$cache/mppi-install/local_setup.bash"
    set -u
fi
gui=${EDGEAI_GUI:-true}
rviz=${EDGEAI_RVIZ:-true}
check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_dir="$PROJECT_ROOT/src/edgeai_navigation/config"
sim_dir="$PROJECT_ROOT/src/edgeai_simulation"
exploration_dir="$PROJECT_ROOT/src/edgeai_exploration"
arena_args=()
if [[ $arena == competition ]]; then
    arena_args=("targets_file:=$sim_dir/config/targets_competition.yaml")
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
    [[ $controller != mppi ]] || nav_config="$config_dir/nav2_exploration_rectangle_mppi.yaml"
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
if [[ $controller == mppi && ( $EDGEAI_ROS_BACKEND == pixi || ${EDGEAI_MPPI_USE_SOURCE:-false} == true ) ]]; then
    cp "$cache/mppi-install/edgeai-mppi-revision" "$temp_dir/mppi-source-revision.txt"
    cp "$cache/mppi-install/edgeai-mppi-backport" "$temp_dir/mppi-backport-revision.txt"
fi
printf '%s\n' "$EDGEAI_ROS_BACKEND" > "$temp_dir/ros-backend.txt"
printf '%s\n' "$GALLIUM_DRIVER" > "$temp_dir/render-driver.txt"
ros2 pkg prefix nav2_mppi_controller > "$temp_dir/mppi-prefix.txt"
export PYTHONUNBUFFERED=1 RCUTILS_LOGGING_BUFFERED_STREAM=0
export ROS_LOG_DIR="$temp_dir/ros"
python3 - "$nav_config" "$temp_dir/nav2-runtime.yaml" <<'PY'
from pathlib import Path
import sys, yaml, os
source, output = map(Path, sys.argv[1:])
config = yaml.safe_load(source.read_text())
if os.environ.get('EDGEAI_MOTION_ABLATION') == 'goal_only':
    assert os.environ.get('EDGEAI_TURN_TEST') == 'true'
    follow = config['controller_server']['ros__parameters']['FollowPath']
    assert follow['plugin'] == 'nav2_mppi_controller::MPPIController'
    follow['critics'] = ['ConstraintCritic', 'GoalCritic', 'GoalAngleCritic', 'PreferForwardCritic']
    follow['GoalCritic']['threshold_to_consider'] = 10.0
    print('TEST ABLATION: goal-only MPPI; collision monitor remains enabled. Not a deployment profile.', flush=True)
planner = config['planner_server']['ros__parameters']['GridBased']
if 'lattice_filepath' in planner:
    planner['lattice_filepath'] = str((source.parent / planner['lattice_filepath']).resolve())
bt = config['bt_navigator']['ros__parameters']
bt['navigators'] = ['navigate_to_pose']
bt['default_nav_to_pose_bt_xml'] = str((source.parent/'behavior_trees/explore_recovery.xml').resolve())
output.write_text(yaml.safe_dump(config, sort_keys=False))
PY
nav_config="$temp_dir/nav2-runtime.yaml"
for name in sim nav explore rviz rosout watch alignment authority perception observation; do touch "$temp_dir/$name.log"; done
event() { printf '%s [session] %s\n' "$(date --iso-8601=seconds)" "$*" | tee -a "$temp_dir/events.log"; }
registration="$PROJECT_ROOT/runtime/exploration-$ROS_DOMAIN_ID.json"
python3 - "$registration" "$temp_dir" "$$" "$GZ_PARTITION" "$arena" "$profile" "$duration" <<'PY'
import json, pathlib, sys, os
registration, directory, pid, partition, arena, profile, duration = sys.argv[1:]
record = dict(pid=int(pid), start_ticks=pathlib.Path(f'/proc/{pid}/stat').read_text().split()[21],
              directory=directory, partition=partition, domain=os.environ['ROS_DOMAIN_ID'],
              arena=arena, profile=profile, duration=int(duration))
record['mode'] = 'isolated_turn_check' if os.environ.get('EDGEAI_TURN_TEST') == 'true' else 'exploration'
if os.environ.get('EDGEAI_RECOVERY_TEST') == 'true':
    record['mode'] = 'native_recovery_check'
for path in (pathlib.Path(registration), pathlib.Path(directory)/'session.json'):
    path.write_text(json.dumps(record, indent=2))
PY
printf '%s\n' "$temp_dir" > "$PROJECT_ROOT/runtime/latest-session.txt"
[[ $backend != frontier ]] || touch "$temp_dir/frontier-backend"
python3 -u "$PROJECT_ROOT/scripts/ros/stream_session_logs.py" "$temp_dir" 2>"$temp_dir/stream-errors.log" 9>&- &
tail_pid=$!
sim_pid='' nav_pid='' explore_pid='' watch_pid='' rviz_pid='' monitor_pid='' diagnostic_pid='' authority_pid='' command_probe_pid='' perception_pid='' scan_pid='' observation_pid=''
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
    [[ -z $diagnostic_pid ]] || kill -TERM "$diagnostic_pid" 2>/dev/null || true
    [[ -z $command_probe_pid ]] || kill -TERM "$command_probe_pid" 2>/dev/null || true
    [[ -z $observation_pid ]] || kill -TERM "$observation_pid" 2>/dev/null || true
    [[ -z $observation_pid ]] || wait "$observation_pid" 2>/dev/null || true
    [[ -z $scan_pid ]] || kill -TERM "$scan_pid" 2>/dev/null || true
    [[ -z $scan_pid ]] || wait "$scan_pid" 2>/dev/null || true
    [[ -z $perception_pid ]] || kill -TERM "$perception_pid" 2>/dev/null || true
    [[ -z $perception_pid ]] || wait "$perception_pid" 2>/dev/null || true
    [[ -z $authority_pid ]] || kill -INT "$authority_pid" 2>/dev/null || true
    [[ -z $watch_pid ]] || kill -INT "$watch_pid" 2>/dev/null || true
    [[ -z $watch_pid ]] || wait "$watch_pid" 2>/dev/null || true
    [[ -z $rviz_pid ]] || kill -TERM -- "-$rviz_pid" 2>/dev/null || true
    # Stop the Gazebo update loop through transport before delivering signals.
    # Calling Stop() from a signal handler can race rendering sensor teardown.
    if [[ -n $sim_pid ]] && kill -0 "$sim_pid" 2>/dev/null; then
        event '请求 Gazebo 自身停止接口，等待传感器与更新线程退出'
        timeout 5s gz service -s /server_control --reqtype gz.msgs.ServerControl \
            --reptype gz.msgs.Boolean --timeout 3000 --req 'stop: true' \
            >"$temp_dir/gazebo-stop.log" 2>&1 || true
        sleep 2
    fi
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
    if grep -Eq 'Segmentation fault|exit code (139|-11)' "$temp_dir/sim.log"; then
        event '仿真进程发生段错误：测试标记为失败，请查 sim.log 与崩溃转储'
        [[ $result -ne 0 ]] || result=1
    fi
    [[ -z $monitor_pid ]] || kill -TERM "$monitor_pid" 2>/dev/null || true
    touch "$temp_dir/stream.stop"
    wait "$tail_pid" 2>/dev/null || true
    rm -f -- "$registration"
    printf '%s\n' "$result" > "$temp_dir/exit-code.txt"
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
event "启动测试：ROS 域 $ROS_DOMAIN_ID，场地 $arena，轮廓 $profile；所有原始输出实时保存"
# Use the installed native GUI plugin configuration; change only the view.
world_source="$sim_dir/worlds/empty.sdf"
[[ $arena != competition ]] || world_source="$sim_dir/worlds/competition_rooms.sdf"
python3 "$PROJECT_ROOT/src/edgeai_simulation/tools/prepare_observation_world.py" "$world_source" "$temp_dir/observation.sdf" \
    "$EDGEAI_GZ_GUI_CONFIG"
arena_args+=("world_file:=$temp_dir/observation.sdf")
python3 -u "$PROJECT_ROOT/scripts/ros/monitor_ros.py" >"$temp_dir/rosout.log" 2>&1 9>&- &
monitor_pid=$!
python3 -u "$PROJECT_ROOT/scripts/ros/navigation_diagnostics.py" --ros-args \
    -p use_sim_time:=true -p "log_directory:=$temp_dir" >"$temp_dir/alignment.log" 2>&1 9>&- &
diagnostic_pid=$!

setsid ros2 launch edgeai_simulation simulation_real.launch.py \
    multi_robot:=false robot_count:=1 gui:="$gui" rviz:=false slam:=true "${arena_args[@]}" >"$temp_dir/sim.log" 2>&1 9>&- &
sim_pid=$!
setsid ros2 launch edgeai_navigation managed_navigation.launch.py use_sim_time:=true \
    params_file:="$nav_config" mppi_invert_linear:="${EDGEAI_MPPI_INVERT_LINEAR:-false}" >"$temp_dir/nav.log" 2>&1 9>&- &
nav_pid=$!
if [[ ${EDGEAI_MPPI_INVERT_LINEAR:-false} == true ]]; then
    [[ $controller == mppi && ${EDGEAI_TURN_TEST:-false} == true ]] || exit 2
    python3 -u "$PROJECT_ROOT/tests/exploration/mppi_command_probe.py" --ros-args \
        -p use_sim_time:=true >"$temp_dir/mppi-command-probe.jsonl" 2>&1 9>&- &
    command_probe_pid=$!
    event '实验：仅 MPPI 线速度取反，恢复动作不变；碰撞监控保留'
fi
authority_mode=AUTO
[[ ${EDGEAI_SCAN360:-false} != true ]] || authority_mode=STOPPED
[[ ${EDGEAI_TURN_TEST:-false} != true ]] || authority_mode=MANUAL
[[ ${EDGEAI_RECOVERY_TEST:-false} != true ]] || authority_mode=MANUAL
python3 -u "$PROJECT_ROOT/src/edgeai_navigation/scripts/navigation_authority.py" --directory "$temp_dir" \
    --mode "$authority_mode" --tree "$config_dir/behavior_trees/explore_recovery.xml" \
    --ros-args -p use_sim_time:=true >"$temp_dir/authority.log" 2>&1 9>&- &
authority_pid=$!
if [[ ${EDGEAI_TARGET_SEARCH:-true} == true ]]; then
    if [[ ${EDGEAI_DISABLE_RGBD_CAMERA:-false} == true ]]; then
        echo 'Target search requires RGB-D camera; unset EDGEAI_DISABLE_RGBD_CAMERA or disable EDGEAI_TARGET_SEARCH.' >&2
        exit 2
    fi
    python3 -u "$PROJECT_ROOT/src/edgeai_perception/scripts/aruco_targets.py" --ros-args \
        --params-file "$PROJECT_ROOT/src/edgeai_perception/config/aruco_targets.yaml" \
        -p "output_directory:=$temp_dir" >"$temp_dir/perception.log" 2>&1 9>&- &
    perception_pid=$!
    event '目标寻找已启动：ArUco图像检测、多帧确认和地图定位；详见 perception.log、targets.json'
fi
if [[ $rviz == true ]]; then
    setsid ros2 run rviz2 rviz2 -d "$exploration_dir/rviz/exploration.rviz" \
        --ros-args -p use_sim_time:=true >"$temp_dir/rviz.log" 2>&1 9>&- &
    rviz_pid=$!
fi
ready=false
for _ in {1..60}; do
    if ! kill -0 "$sim_pid" 2>/dev/null || ! kill -0 "$nav_pid" 2>/dev/null; then break; fi
    if timeout 3s ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active '; then
        ready=true
        break
    fi
    sleep 1
done
if [[ -n $perception_pid ]] && ! kill -0 "$perception_pid" 2>/dev/null; then
    echo 'Target perception process exited; inspect perception.log.' >&2
    exit 1
fi
if ! $ready; then
    echo 'Nav2 did not become active. Recent simulator and navigation output:' >&2
    event '启动失败：Nav2 未进入 active 状态，请查看 sim.log 和 nav.log'
    tail -n 35 "$temp_dir/sim.log" >&2
    tail -n 80 "$temp_dir/nav.log" >&2
    exit 1
fi
if [[ ${EDGEAI_RECOVERY_TEST:-false} == true ]]; then
    event 'Nav2 已就绪，开始原生恢复故障注入检查；本轮不启动探索器'
elif [[ ${EDGEAI_TURN_TEST:-false} == true ]]; then
    event "Nav2 已就绪，开始基础运动检查：${EDGEAI_MOTION_TEST:-turn}；本轮不启动探索器"
elif [[ ${EDGEAI_SCAN360:-false} == true ]]; then
    event 'Nav2 已就绪，等待启动360度观察任务'
else
    event 'Nav2 已就绪，开始自主探索'
fi
touch "$temp_dir/ready"
if [[ ${EDGEAI_SCAN360:-false} == true ]]; then
    event '执行启动360度观察任务；目标检测继续运行，扫描结束后开始探索'
    python3 -u "$PROJECT_ROOT/src/edgeai_perception/scripts/scan360.py" \
        --ros-args -p use_sim_time:=true -p "output_file:=$temp_dir/scan360-result.json" \
        >"$temp_dir/scan360.log" 2>&1 9>&- &
    scan_pid=$!
    wait "$scan_pid"
    scan_pid=''
fi
if [[ ${EDGEAI_RECOVERY_TEST:-false} == true ]]; then
    setsid python3 -u "$PROJECT_ROOT/tests/exploration/navigation_recovery_check.py" \
        --output "$temp_dir/recovery-result.json" --ros-args -p use_sim_time:=true >"$temp_dir/explore.log" 2>&1 9>&- &
elif [[ ${EDGEAI_TURN_TEST:-false} == true ]]; then
    setsid python3 -u "$PROJECT_ROOT/tests/exploration/navigation_turn_check.py" \
        --output "$temp_dir/turn-result.json" --ros-args -p use_sim_time:=true >"$temp_dir/explore.log" 2>&1 9>&- &
elif [[ $backend == frontier ]]; then
    event '协调策略：Nav2 连续 10 秒仿真时间未移动满 10 cm 时触发恢复：清图后立即后退，受阻则旋转；探索器保留当前任务，等待最终结果后再选目标'
    python3 -u "$PROJECT_ROOT/src/edgeai_exploration/scripts/watch_exploration_completion.py" "$temp_dir/completed" >"$temp_dir/watch.log" 2>&1 9>&- &
    watch_pid=$!
    setsid ros2 run frontier_exploration_ros2 frontier_explorer --ros-args \
        --params-file "$exploration_dir/config/frontier_exploration.yaml" \
        -r control_exploration:=/exploration/control_exploration >"$temp_dir/explore.log" 2>&1 9>&- &
else
    setsid ros2 run explore_lite explore --ros-args --params-file "$explore_config" \
        -r navigate_to_pose:=/exploration/navigate_to_pose >"$temp_dir/explore.log" 2>&1 9>&- &
fi
explore_pid=$!
if [[ ${EDGEAI_OBSERVATION_SCAN:-true} == true && ${EDGEAI_TARGET_SEARCH:-true} == true && ${EDGEAI_TURN_TEST:-false} != true && ${EDGEAI_RECOVERY_TEST:-false} != true ]]; then
    python3 - "$nav_config" "$PROJECT_ROOT/src/edgeai_perception/config/observation_scan.yaml" "$temp_dir/observation-scan-runtime.yaml" "$temp_dir" <<'SCAN_CONFIG'
import json, math, sys, yaml
from pathlib import Path
nav, template, output, directory = sys.argv[1:]
c = yaml.safe_load(Path(nav).read_text())
radii = []
for name in ('local_costmap', 'global_costmap'):
    geometry = c[name][name]['ros__parameters']
    pad = geometry.get('footprint_padding', 0.0)
    if 'footprint' in geometry:
        radii.append(max(math.hypot(x+math.copysign(pad,x), y+math.copysign(pad,y))
                         for x,y in json.loads(geometry['footprint'])))
    else:
        radii.append(geometry['robot_radius']+pad)
p = yaml.safe_load(Path(template).read_text())
p['observation_scan']['ros__parameters'].update(rotation_radius_m=max(radii), output_directory=directory)
Path(output).write_text(yaml.safe_dump(p, sort_keys=False))
SCAN_CONFIG
    python3 -u "$PROJECT_ROOT/src/edgeai_perception/scripts/observation_scan.py" --ros-args \
        --params-file "$temp_dir/observation-scan-runtime.yaml" >"$temp_dir/observation.log" 2>&1 9>&- &
    observation_pid=$!
fi
start=$SECONDS
reason=time_limit
while (( SECONDS-start < duration )); do
    if [[ ${EDGEAI_RECOVERY_TEST:-false} == true && -f "$temp_dir/recovery-result.json" ]]; then
        reason=native_recovery_check_finished
        break
    fi
    if [[ ${EDGEAI_TURN_TEST:-false} == true && -f "$temp_dir/turn-result.json" ]]; then
        reason=isolated_turn_check_finished
        break
    fi
    if ! kill -0 "$sim_pid" 2>/dev/null || ! kill -0 "$nav_pid" 2>/dev/null || ! kill -0 "$explore_pid" 2>/dev/null || ! kill -0 "$authority_pid" 2>/dev/null; then
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
timeout 5s ros2 service call /navigation_authority/stop std_srvs/srv/Trigger '{}' \
    >"$temp_dir/authority-stop.log" 2>&1 || true
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
if [[ -n $map_basename ]]; then
    mkdir -p "$(dirname "$map_basename")"
    timeout 25s ros2 run nav2_map_server map_saver_cli -f "$map_basename" --ros-args -p use_sim_time:=true >"$temp_dir/map-save.log" 2>&1
    if [[ $arena == competition ]]; then
        python3 "$PROJECT_ROOT/tests/exploration/evaluate_arena_map.py" "$map_basename.yaml" "$sim_dir/worlds/competition_rooms.sdf" | tee "$temp_dir/evaluation.log"
    else
        python3 "$PROJECT_ROOT/tests/exploration/evaluate_exploration_map.py" "$map_basename.yaml" | tee "$temp_dir/evaluation.log"
    fi
fi
# Goal changes and failures have already been reported by the event stream.
if [[ ${EDGEAI_TURN_TEST:-false} == true || ${EDGEAI_RECOVERY_TEST:-false} == true ]]; then
    test_result="$temp_dir/turn-result.json"
    [[ ${EDGEAI_RECOVERY_TEST:-false} != true ]] || test_result="$temp_dir/recovery-result.json"
    python3 - "$test_result" <<'PY'
import json, pathlib, sys
p = pathlib.Path(sys.argv[1])
sys.exit(0 if p.exists() and json.loads(p.read_text()).get('success') else 1)
PY
fi

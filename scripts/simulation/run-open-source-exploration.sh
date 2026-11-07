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
source "$PROJECT_ROOT/scripts/display/configure-desktop.sh"
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
export EDGEAI_CONTROLLER=$controller
arena=${4:-midterm}
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
navigation_test=${EDGEAI_NAVIGATION_TEST:-false}
cooperative_default=false
if [[ $arena == midterm && $navigation_test != true && ${EDGEAI_TURN_TEST:-false} != true && ${EDGEAI_SCAN360:-false} != true && ${EDGEAI_RECOVERY_TEST:-false} != true ]]; then
    cooperative_default=true
fi
cooperative=${EDGEAI_COOPERATIVE_RESCUE:-$cooperative_default}
export EDGEAI_COOPERATIVE_RESCUE=$cooperative
if [[ $cooperative == true && -z ${CYCLONEDDS_URI:-} ]]; then
    export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
fi
if [[ $cooperative == true && ( $arena != midterm || $navigation_test == true || ${EDGEAI_TURN_TEST:-false} == true ) ]]; then
    echo 'Cooperative rescue requires the normal midterm exploration mission.' >&2; exit 2
fi
check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config_dir="$PROJECT_ROOT/src/edgeai_navigation/config"
sim_dir="$PROJECT_ROOT/src/edgeai_simulation"
exploration_dir="$PROJECT_ROOT/src/edgeai_exploration"
arena_args=("targets_file:=$sim_dir/config/targets.yaml"
            "robots_file:=$sim_dir/config/robots.yaml" 'use_robot_pose:=false')
camera_pose='3.8 -4.35 6 0 0.80 2.288'
perception_args=()
if [[ $arena == competition ]]; then
    arena_args+=("targets_file:=$sim_dir/config/targets_competition.yaml")
elif [[ $arena == midterm ]]; then
    arena_args=("targets_file:=$sim_dir/config/targets_midterm.yaml"
                "robots_file:=$sim_dir/config/robots_midterm.yaml" 'use_robot_pose:=true')
    camera_pose='2.5 -3.8 7 0 0.85 1.57079632679'
    perception_args=(-p marker_size_m:=0.16 -p 'target_types:=1:normal,2:cooperative')
    export EDGEAI_RESCUE_CONFIG=${EDGEAI_RESCUE_CONFIG:-$config_dir/midterm_single_rescue.yaml}
elif [[ $arena != simple ]]; then
    echo "Unknown arena: $arena" >&2; exit 2
fi
case "$controller" in
    dwb|mppi|rpp) nav_config="$config_dir/nav2_exploration_rectangle.yaml" ;;
    *) echo "Unknown controller: $controller" >&2; exit 2 ;;
esac
explore_config="$config_dir/explore_lite.yaml"
if [[ $profile == safe ]]; then
    explore_config="$config_dir/explore_lite_safe.yaml"
elif [[ $profile != baseline && $profile != rectangle && $profile != conservative ]]; then
    echo "Unknown profile: $profile" >&2; exit 2
fi
export EDGEAI_NAV_PROFILE=$profile
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
sys.path.insert(0,str(source.parent.parent/'scripts'))
from navigation_config import load_navigation
profile=os.environ.get('EDGEAI_NAV_PROFILE','rectangle')
depth_switch=os.environ.get('EDGEAI_DEPTH_OBSTACLES','config').lower()
if depth_switch not in ('config','true','false'):
    raise ValueError('EDGEAI_DEPTH_OBSTACLES must be config, true or false')
config=load_navigation(source.parent,os.environ.get('EDGEAI_CONTROLLER','dwb'),profile,
                       None if depth_switch=='config' else depth_switch=='true')
planner_profile=os.environ.get('EDGEAI_GLOBAL_PLANNER','lattice' if profile=='rectangle' else 'navfn')
if planner_profile=='lattice':
    from ament_index_python.packages import get_package_share_directory
    from planner_profile import configure_lattice
    configure_lattice(config,source.parent,get_package_share_directory('nav2_smac_planner'))
elif planner_profile!='navfn':
    raise ValueError('EDGEAI_GLOBAL_PLANNER must be lattice or navfn')
for section in ('local_costmap', 'global_costmap'):
    costmap = config[section][section]['ros__parameters']
    for dimension in ('width', 'height'):
        value = costmap.get(dimension)
        if value is not None and (type(value) is not int or value <= 0):
            raise ValueError(f'{section}.{dimension} must be a positive integer in Nav2 Jazzy; '
                             'set fractional_window.width_m/height_m for fractional local dimensions')
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
# A selectable stopped checker tightens rescue acceptance without changing exploration.
rescue_config = yaml.safe_load((Path(os.environ.get('EDGEAI_RESCUE_CONFIG', str(source.parents[0]/'normal_rescue.yaml')))).read_text())
control = config['controller_server']['ros__parameters']
control['general_goal_checker']['plugin'] = 'nav2_controller::PositionGoalChecker'
control['general_goal_checker'].pop('yaw_goal_tolerance', None)
if os.environ.get('EDGEAI_POINT_TEST_PARAMS'):
    assert os.environ.get('EDGEAI_TURN_TEST')=='true' and os.environ.get('EDGEAI_MOTION_TEST')=='points'
    overrides=yaml.safe_load(Path(os.environ['EDGEAI_POINT_TEST_PARAMS']).read_text()) or {}
    control['progress_checker'].update(overrides.pop('progress_checker',{}))
    control['FollowPath'].update(overrides)
is_navfn=planner['plugin']=='nav2_navfn_planner::NavfnPlanner'
if is_navfn:planner['use_final_approach_orientation'] = True
# Per-action policies use fixed plugin IDs; never mutate a live planner parameter.
from copy import deepcopy
planners = config['planner_server']['ros__parameters']
if is_navfn:
    planners['GridBasedHeading'] = deepcopy(planner)
    planners['GridBasedHeading']['use_final_approach_orientation'] = False
planners['planner_plugins'] = list(dict.fromkeys(planners['planner_plugins']+['GridBasedHeading']))
control['goal_checker_plugins'] = list(dict.fromkeys(control['goal_checker_plugins']+['heading_goal_checker','rescue_goal_checker']))
control['heading_goal_checker'] = dict(plugin='nav2_controller::SimpleGoalChecker', stateful=True,
    xy_goal_tolerance=control['general_goal_checker']['xy_goal_tolerance'],
    yaw_goal_tolerance=float(rescue_config['yaw_tolerance_rad']))
control['rescue_goal_checker'] = dict(plugin='nav2_controller::StoppedGoalChecker', stateful=True,
    xy_goal_tolerance=float(rescue_config['position_tolerance_m']),
    yaw_goal_tolerance=float(rescue_config['yaw_tolerance_rad']),
    trans_stopped_velocity=float(rescue_config['stopped_linear_mps']),
    rot_stopped_velocity=float(rescue_config['stopped_angular_rps']))
sys.path.insert(0,str(source.parent.parent/'scripts'))
from goal_policy import configure_controllers
configure_controllers(config)
from motion_limits import configure_motion_limits
import json
motion_report=configure_motion_limits(config)
output.with_name('motion-limits.json').write_text(json.dumps(motion_report,indent=2))
if motion_report['controllers']:
    print('MOTION_LIMITS: RPP预测前速度已联动公共限幅；详见 motion-limits.json',flush=True)
for warning in motion_report['warnings']:print('MOTION_LIMITS_WARNING: '+warning,flush=True)
bt = config['bt_navigator']['ros__parameters']
local_geometry=config['local_costmap']['local_costmap']['ros__parameters']
escape=config['behavior_server']['ros__parameters']['escape_occupied']
escape['actual_footprint']=local_geometry.get('footprint','[]')
escape['footprint_padding']=float(local_geometry.get('footprint_padding',0.))
bt['plugin_lib_names']=list(dict.fromkeys(bt.get('plugin_lib_names',[])+['edgeai_navigation_bt_nodes']))
bt['navigators'] = ['navigate_to_pose']
bt['default_nav_to_pose_bt_xml'] = str((source.parent/'behavior_trees/explore_recovery.xml').resolve())
output.write_text(yaml.safe_dump(config, sort_keys=False))
PY
nav_config="$temp_dir/nav2-runtime.yaml"
for name in sim nav explore rviz rosout watch alignment authority perception observation suspects; do touch "$temp_dir/$name.log"; done
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
if os.environ.get('EDGEAI_NAVIGATION_TEST') == 'true':
    record['mode'] = 'truth_navigation_test'
for path in (pathlib.Path(registration), pathlib.Path(directory)/'session.json'):
    path.write_text(json.dumps(record, indent=2))
PY
printf '%s\n' "$temp_dir" > "$PROJECT_ROOT/runtime/latest-session.txt"
[[ $backend != frontier ]] || touch "$temp_dir/frontier-backend"
python3 -u "$PROJECT_ROOT/scripts/ros/stream_session_logs.py" "$temp_dir" 2>"$temp_dir/stream-errors.log" 9>&- &
tail_pid=$!
sim_pid='' nav_pid='' explore_pid='' watch_pid='' rviz_pid='' monitor_pid='' diagnostic_pid='' authority_pid='' command_probe_pid='' rescue_pid='' perception_pid='' scan_pid='' observation_pid='' suspects_pid=''
cooperative_pid=''
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
    [[ -z $cooperative_pid ]] || kill -TERM "$cooperative_pid" 2>/dev/null || true
    [[ -z $cooperative_pid ]] || wait "$cooperative_pid" 2>/dev/null || true
    [[ -z $monitor_pid ]] || kill -INT "$monitor_pid" 2>/dev/null || true
    [[ -z $diagnostic_pid ]] || kill -TERM "$diagnostic_pid" 2>/dev/null || true
    [[ -z $command_probe_pid ]] || kill -TERM "$command_probe_pid" 2>/dev/null || true
    [[ -z $rescue_pid ]] || kill -TERM "$rescue_pid" 2>/dev/null || true
    [[ -z $rescue_pid ]] || wait "$rescue_pid" 2>/dev/null || true
    [[ -z $observation_pid ]] || kill -TERM "$observation_pid" 2>/dev/null || true
    [[ -z $observation_pid ]] || wait "$observation_pid" 2>/dev/null || true
    [[ -z $scan_pid ]] || kill -TERM "$scan_pid" 2>/dev/null || true
    [[ -z $scan_pid ]] || wait "$scan_pid" 2>/dev/null || true
    [[ -z $perception_pid ]] || kill -TERM "$perception_pid" 2>/dev/null || true
    [[ -z $perception_pid ]] || wait "$perception_pid" 2>/dev/null || true
    [[ -z $suspects_pid ]] || kill -TERM "$suspects_pid" 2>/dev/null || true
    [[ -z $suspects_pid ]] || wait "$suspects_pid" 2>/dev/null || true
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
[[ $arena != midterm ]] || world_source="$sim_dir/worlds/midterm.sdf"
python3 "$PROJECT_ROOT/src/edgeai_simulation/tools/prepare_observation_world.py" "$world_source" "$temp_dir/observation.sdf" \
    "$EDGEAI_GZ_GUI_CONFIG" "$camera_pose"
arena_args+=("world_file:=$temp_dir/observation.sdf")
simulation_slam=true
if [[ $navigation_test == true ]]; then
    simulation_slam=false
    targets_config="$sim_dir/config/targets.yaml"
    [[ $arena != midterm ]] || targets_config="$sim_dir/config/targets_midterm.yaml"
    [[ $arena != competition ]] || targets_config="$sim_dir/config/targets_competition.yaml"
    python3 - "$sim_dir" "$targets_config" "$temp_dir" <<'TRUTH_MAP'
import importlib.util,sys
from pathlib import Path
share,targets,directory=map(Path,sys.argv[1:])
spec=importlib.util.spec_from_file_location('support',share/'launch/simulation_support.py')
support=importlib.util.module_from_spec(spec);spec.loader.exec_module(support)
support.make_world(share,targets,directory/'truth-world.sdf',directory/'observation.sdf')
TRUTH_MAP
    python3 "$sim_dir/tools/generate_navigation_truth_map.py" "$temp_dir/truth-world.sdf" "$temp_dir/map"
    arena_args+=('navigation_test:=true' "map_file:=$temp_dir/map.yaml")
    event '真值导航测试：加载真实碰撞地图和仿真定位，禁用SLAM；map坐标与仿真world一致'
fi
python3 -u "$PROJECT_ROOT/scripts/ros/monitor_ros.py" >"$temp_dir/rosout.log" 2>&1 9>&- &
monitor_pid=$!
python3 -u "$PROJECT_ROOT/scripts/ros/navigation_diagnostics.py" --ros-args \
    -p use_sim_time:=true -p "static_map:=$navigation_test" -p "log_directory:=$temp_dir" >"$temp_dir/alignment.log" 2>&1 9>&- &
diagnostic_pid=$!

export EDGEAI_TRUTH_DIRECTORY="$temp_dir"
fleet_args=('multi_robot:=false' 'robot_count:=1')
if [[ $cooperative == true ]]; then
    fleet_args=('multi_robot:=true' 'robot_count:=2' 'primary_unscoped:=true')
    python3 -u "$PROJECT_ROOT/src/edgeai_navigation/scripts/cooperative_rescue.py" \
        --directory "$temp_dir" --config "$config_dir/cooperative_rescue.yaml" \
        --ros-args -p use_sim_time:=true >"$temp_dir/cooperative.log" 2>&1 9>&- &
    cooperative_pid=$!
    event '双机任务：robot1探索/S1，robot2等待；S1成功后共享地图激光定位并协作救援C1'
fi
setsid ros2 launch edgeai_simulation simulation_real.launch.py \
    "${fleet_args[@]}" gui:="$gui" rviz:=false slam:="$simulation_slam" "${arena_args[@]}" >"$temp_dir/sim.log" 2>&1 9>&- &
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
[[ $navigation_test != true ]] || authority_mode=MANUAL
[[ ${EDGEAI_SCAN360:-false} != true ]] || authority_mode=STOPPED
[[ ${EDGEAI_TURN_TEST:-false} != true ]] || authority_mode=MANUAL
[[ ${EDGEAI_RECOVERY_TEST:-false} != true ]] || authority_mode=MANUAL
allow_auto=true
[[ $navigation_test != true ]] || allow_auto=false
python3 -u "$PROJECT_ROOT/src/edgeai_navigation/scripts/navigation_authority.py" --directory "$temp_dir" \
    --mode "$authority_mode" --tree "$config_dir/behavior_trees/explore_recovery.xml" \
    --ros-args -p use_sim_time:=true -p "allow_auto:=$allow_auto" >"$temp_dir/authority.log" 2>&1 9>&- &
authority_pid=$!
if [[ ${EDGEAI_TARGET_SEARCH:-true} == true ]]; then
    if [[ ${EDGEAI_DISABLE_RGBD_CAMERA:-false} == true ]]; then
        echo 'Target search requires RGB-D camera; unset EDGEAI_DISABLE_RGBD_CAMERA or disable EDGEAI_TARGET_SEARCH.' >&2
        exit 2
    fi
    python3 -u "$PROJECT_ROOT/src/edgeai_perception/scripts/aruco_targets.py" --ros-args \
        --params-file "$PROJECT_ROOT/src/edgeai_perception/config/aruco_targets.yaml" \
        "${perception_args[@]}" \
        -p "output_directory:=$temp_dir" >"$temp_dir/perception.log" 2>&1 9>&- &
    perception_pid=$!
    # Low-rate mapping/search detection with session-scoped multi-frame memory.
    suspect_panel_size=0.24
    [[ $arena != midterm ]] || suspect_panel_size=0.20
    python3 -u "$PROJECT_ROOT/src/edgeai_perception/scripts/suspect_targets.py" --ros-args \
        --params-file "$PROJECT_ROOT/src/edgeai_perception/config/suspect_targets.yaml" \
        -p "panel_size_m:=$suspect_panel_size" -p "output_directory:=$temp_dir" \
        >"$temp_dir/suspects.log" 2>&1 9>&- &
    suspects_pid=$!
    event '目标寻找已启动：ArUco图像检测、多帧确认和地图定位；详见 perception.log、targets.json'
fi
if [[ $rviz == true ]]; then
    rviz_config="$exploration_dir/rviz/exploration.rviz"
    # Generate all modes so the depth display follows the actual runtime switch.
    if [[ -f $nav_config ]]; then
        python3 - "$rviz_config" "$temp_dir/navigation-test.rviz" "$navigation_test" "$controller" "$nav_config" <<'TEST_RVIZ'
from pathlib import Path
import sys,yaml,os
from copy import deepcopy
source,output=map(Path,sys.argv[1:3]);config=yaml.safe_load(source.read_text())
manager=config['Visualization Manager']
nav=yaml.safe_load(Path(sys.argv[5]).read_text())
depth_enabled='depth_layer' in nav['local_costmap']['local_costmap']['ros__parameters']['plugins']
if sys.argv[3]=='true':manager['Views']['Current']['Focal Point'].update(X=2.5,Y=2.5)
for display in manager['Displays']:
    if display.get('Name')=='Depth costmap input (cyan)':
        display['Enabled']=depth_enabled;display['Value']=depth_enabled
    if sys.argv[3]=='true' and display.get('Name')=='SLAM map (occupancy)':
        display['Name']='True map (world coordinates)'
    if sys.argv[4]=='rpp' and display.get('Name')=='Local trajectory (pink)':
        display['Name']='RPP collision arc (pink)';display['Topic']['Value']='/lookahead_arc'
if sys.argv[4]=='rpp':
    manager['Displays'].append({'Class':'rviz_default_plugins/PointStamped',
        'Name':'RPP lookahead point','Enabled':True,'Value':True,
        'Color':'255; 190; 0','Radius':0.05,'History Length':1,
        'Topic':{'Value':'/lookahead_point','Depth':1,'Reliability Policy':'Reliable','Durability Policy':'Volatile'}})
if os.environ.get('EDGEAI_COOPERATIVE_RESCUE')=='true':
    for display in list(manager['Displays']):
        if display.get('Class')=='rviz_default_plugins/RobotModel':
            robot=deepcopy(display);robot['Name']='Robot2';robot['Description Topic']['Value']='/robot2/robot_description'
            manager['Displays'].append(robot)
        elif display.get('Name')=='Global path (green)':
            path=deepcopy(display);path['Name']='Robot2 global path';path['Topic']['Value']='/robot2/plan'
            path['Color']='255; 170; 0';manager['Displays'].append(path)
        elif display.get('Name')=='Depth costmap input (cyan)':
            cloud=deepcopy(display);cloud['Name']='Robot2 depth costmap input (blue)'
            cloud['Topic']['Value']='/robot2/camera/obstacle_points';cloud['Color']='80; 140; 255'
            manager['Displays'].append(cloud)
output.write_text(yaml.safe_dump(config,sort_keys=False))
TEST_RVIZ
        rviz_config="$temp_dir/navigation-test.rviz"
    fi
    setsid ros2 run rviz2 rviz2 -d "$rviz_config" \
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
if [[ -n $suspects_pid ]] && ! kill -0 "$suspects_pid" 2>/dev/null; then
    event '疑似目标检测节点退出，搜寻将回退到视锥/空间候选；请检查 suspects.log'
fi
if ! $ready; then
    echo 'Nav2 did not become active. Recent simulator and navigation output:' >&2
    event '启动失败：Nav2 未进入 active 状态，请查看 sim.log 和 nav.log'
    tail -n 35 "$temp_dir/sim.log" >&2
    tail -n 80 "$temp_dir/nav.log" >&2
    exit 1
fi
if [[ $navigation_test == true && ${EDGEAI_TURN_TEST:-false} != true ]]; then
    event '真值地图已就绪：使用RViz的2D Goal Pose手动选点；默认只要求位置，可勾选终点朝向'
elif [[ ${EDGEAI_RECOVERY_TEST:-false} == true ]]; then
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
if [[ $navigation_test == true && ${EDGEAI_TURN_TEST:-false} != true ]]; then
    setsid sleep "$duration" &
elif [[ ${EDGEAI_RECOVERY_TEST:-false} == true ]]; then
    setsid python3 -u "$PROJECT_ROOT/tests/exploration/navigation_recovery_check.py" \
        --output "$temp_dir/recovery-result.json" --ros-args -p use_sim_time:=true >"$temp_dir/explore.log" 2>&1 9>&- &
elif [[ ${EDGEAI_TURN_TEST:-false} == true ]]; then
    motion_check="$PROJECT_ROOT/tests/exploration/navigation_turn_check.py"
    [[ ${EDGEAI_MOTION_TEST:-turn} != points ]] || motion_check="$PROJECT_ROOT/tests/exploration/navigation_point_check.py"
    setsid python3 -u "$motion_check" \
        --output "$temp_dir/turn-result.json" --ros-args -p use_sim_time:=true >"$temp_dir/explore.log" 2>&1 9>&- &
elif [[ $backend == frontier ]]; then
    event '协调策略：按 nav2-runtime.yaml 的进展距离和等待时间判断停滞；清图后后退、受阻则旋转，探索器等待最终结果后再选目标'
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
if [[ $navigation_test != true && ${EDGEAI_OBSERVATION_SCAN:-true} == true && ${EDGEAI_TARGET_SEARCH:-true} == true && ${EDGEAI_TURN_TEST:-false} != true && ${EDGEAI_RECOVERY_TEST:-false} != true ]]; then
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
event "探索阶段结束：$reason，用时 $((SECONDS-start)) 秒"
rescue_config=${EDGEAI_RESCUE_CONFIG:-$config_dir/normal_rescue.yaml}
rescue_status=0
if [[ ${EDGEAI_RESCUE:-true} == true && ( $reason == upstream_completion_event || $reason == no_frontiers ) ]] &&
   python3 -c 'import sys,yaml; sys.exit(0 if yaml.safe_load(open(sys.argv[1]))["enabled"] else 1)' "$rescue_config"; then
    # Stop observation scheduling before taking MANUAL ownership; keep Nav2 and SLAM alive.
    [[ -z $observation_pid ]] || kill -TERM "$observation_pid" 2>/dev/null || true
    [[ -z $observation_pid ]] || wait "$observation_pid" 2>/dev/null || true
    observation_pid=''
    handover_map=${map_basename:-$temp_dir/map}
    mkdir -p "$(dirname "$handover_map")"
    timeout 25s ros2 run nav2_map_server map_saver_cli -f "$handover_map" --ros-args -p use_sim_time:=true >"$temp_dir/map-handover.log" 2>&1
    python3 "$PROJECT_ROOT/src/edgeai_navigation/scripts/export_handover.py" "$temp_dir" "$handover_map.yaml"
    cp "$rescue_config" "$temp_dir/rescue-config.yaml"
    python3 - "$nav_config" "$temp_dir/rescue-config.yaml" <<'SEARCH_CONFIG'
import json, math, sys, yaml
from pathlib import Path
nav = yaml.safe_load(Path(sys.argv[1]).read_text())
path = Path(sys.argv[2])
config = yaml.safe_load(path.read_text())
radii = []
for name in ('local_costmap', 'global_costmap'):
    geometry = nav[name][name]['ros__parameters']
    pad = geometry.get('footprint_padding', 0.0)
    if 'footprint' in geometry:
        radii.append(max(math.hypot(x+math.copysign(pad,x), y+math.copysign(pad,y))
                         for x,y in json.loads(geometry['footprint'])))
    else:
        radii.append(geometry['robot_radius']+pad)
config['search_clearance_m'] = max(config.get('search_clearance_m', .40), max(radii)+.05)
path.write_text(yaml.safe_dump(config, sort_keys=False))
SEARCH_CONFIG
    event '建图自然完成，开始一次普通救援；详见 rescue.log 和 rescue-result.json'
    python3 -u "$PROJECT_ROOT/src/edgeai_navigation/scripts/normal_rescue.py" \
        --config "$temp_dir/rescue-config.yaml" --directory "$temp_dir" \
        --ros-args -p use_sim_time:=true >"$temp_dir/rescue.log" 2>&1 9>&- &
    rescue_pid=$!
    wait "$rescue_pid" || rescue_status=$?
    rescue_pid=''
    if [[ $cooperative == true && $rescue_status -eq 0 ]]; then
        event 'S1完成，等待双机定位与C1协作救援；详见 cooperative.log'
        wait "$cooperative_pid" || rescue_status=$?
        cooperative_pid=''
    fi
    event "救援阶段结束，退出码 $rescue_status；开始截图与地图评估"
fi
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
if [[ -n $map_basename && $navigation_test != true ]]; then
    mkdir -p "$(dirname "$map_basename")"
    if [[ ! -f "$temp_dir/task1-output.json" ]]; then
        timeout 25s ros2 run nav2_map_server map_saver_cli -f "$map_basename" --ros-args -p use_sim_time:=true >"$temp_dir/map-save.log" 2>&1
        python3 "$PROJECT_ROOT/src/edgeai_navigation/scripts/export_handover.py" "$temp_dir" "$map_basename.yaml"
    fi
    if [[ $arena == competition ]]; then
        python3 "$PROJECT_ROOT/tests/exploration/evaluate_arena_map.py" "$map_basename.yaml" "$sim_dir/worlds/competition_rooms.sdf" | tee "$temp_dir/evaluation.log"
    elif [[ $arena == simple ]]; then
        python3 "$PROJECT_ROOT/tests/exploration/evaluate_exploration_map.py" "$map_basename.yaml" | tee "$temp_dir/evaluation.log"
    else
        echo 'Midterm map saved; legacy fixed-start development coverage evaluator is not applicable.' | tee "$temp_dir/evaluation.log"
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

[[ $rescue_status -eq 0 ]] || exit "$rescue_status"

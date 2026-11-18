#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
usage() {
    cat <<'HELP'
Usage: bash scripts/simulation/start-autonomous-exploration.sh [options]
  --duration SECONDS       Wall-clock test budget, 30..3600 (default: 3600)
  --arena simple|competition|midterm         (default: midterm)
  --controller dwb|mppi|rpp                  (default: rpp)
  --planner lattice|navfn                     (default: lattice)
  --backend native|pixi                      (default: native)
  --domain ID                                (default: 86)
  --targets ID:type,...    Override arena target list (normal or cooperative)
  --target-count N         Verify the number of required targets (0 = infer)
  --map PATH               Output map basename; normally generated per run
  --scan360                Scan one full turn before exploration
  --single-robot          Disable cooperative handover
  --navigation-test       Static truth map and manual RViz goals; single robot
  --depth-obstacles config|true|false       Local depth layer (default: config)
  --check                  Check runtime/packages without starting/stopping tests
  --language zh|en         Operator display/log language (default: zh)
  --runtime-profile debug|performance       Diagnostics/display preset (default: debug)
  --joint-mapping          Integrated two-robot joint SLAM and concurrent exploration
  --help                   Show this help
Legacy positional form remains: [duration] [map basename] [arena] [domain]
HELP
}
duration=3600
map_basename=''
arena=midterm
domain=86
check_only=false
if [[ $# -gt 0 && $1 != --* ]]; then
    [[ $# -le 4 ]] || { usage >&2; exit 2; }
    duration=${1:-3600}
    map_basename=${2:-}
    arena=${3:-midterm}
    domain=${4:-86}
else
    while (( $# )); do
        case "$1" in
            --help) usage; exit 0 ;;
            --scan360) export EDGEAI_SCAN360=true; shift ;;
            --single-robot) export EDGEAI_COOPERATIVE_RESCUE=false; shift ;;
            --joint-mapping) export EDGEAI_JOINT_MAPPING=true; shift ;;
            --navigation-test) export EDGEAI_NAVIGATION_TEST=true EDGEAI_COOPERATIVE_RESCUE=false; shift ;;
            --check) check_only=true; shift ;;
            --duration|--targets|--target-count|--map|--arena|--domain|--controller|--backend|--planner|--depth-obstacles|--language|--runtime-profile)
                [[ $# -ge 2 && $2 != --* ]] || { echo "Missing value: $1" >&2; exit 2; }
                case "$1" in
                    --duration) duration=$2 ;;
                    --targets) export EDGEAI_TARGETS=$2 ;;
                    --target-count) export EDGEAI_TARGET_COUNT=$2 ;;
                    --map) map_basename=$2 ;;
                    --arena) arena=$2 ;;
                    --domain) domain=$2 ;;
                    --controller) export EDGEAI_CONTROLLER=$2 ;;
                    --planner) export EDGEAI_GLOBAL_PLANNER=$2 ;;
                    --backend) export EDGEAI_ROS_BACKEND=$2 ;;
                    --depth-obstacles) export EDGEAI_DEPTH_OBSTACLES=$2 ;;
                    --language) export EDGEAI_LANGUAGE=$2 ;;
                    --runtime-profile) export EDGEAI_RUNTIME_PROFILE=$2 ;;
                esac
                shift 2 ;;
            *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
        esac
    done
fi
export EDGEAI_LANGUAGE=${EDGEAI_LANGUAGE:-zh}
[[ $EDGEAI_LANGUAGE =~ ^(zh|en)$ ]] || { echo 'Language must be zh or en.' >&2; exit 2; }
if [[ ${EDGEAI_JOINT_MAPPING:-false} == true && ( ( $arena == midterm && ${EDGEAI_COOPERATIVE_RESCUE:-true} == false ) || ${EDGEAI_MIDTERM_PART:-integrated} != integrated || ${EDGEAI_NAVIGATION_TEST:-false} == true || ${EDGEAI_SCAN360:-false} == true || ${EDGEAI_TURN_TEST:-false} == true || ${EDGEAI_RECOVERY_TEST:-false} == true ) ]]; then
    echo 'Joint mapping requires an integrated two-robot mission.' >&2; exit 2
fi
[[ ${EDGEAI_DEPTH_OBSTACLES:-config} =~ ^(config|true|false)$ ]] || { echo 'Depth obstacles must be config, true or false.' >&2; exit 2; }
if [[ ${EDGEAI_SCAN360:-false} == true && ( ${EDGEAI_TURN_TEST:-false} == true || ${EDGEAI_RECOVERY_TEST:-false} == true ) ]]; then
    echo 'Scan360 cannot be combined with motion/recovery tests.' >&2; exit 2
fi
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
cache=$EDGEAI_RUNTIME_CACHE
if [[ ! -f "$cache/frontier-install/local_setup.bash" ]]; then
    echo 'Nav2/frontier explorer overlay is missing. Run scripts/setup/prepare-open-source.sh first.' >&2
    exit 1
fi
if [[ ${EDGEAI_JOINT_MAPPING:-false} == true ]] && ! grep -qx 'joint-monitor-v2-candidates' "$cache/frontier-install/edgeai-recovery-owner"; then
    echo 'Rebuild the joint exploration monitor: bash scripts/setup/build-frontier-runtime.sh' >&2; exit 1
fi
if ! grep -qx 'frontier-endpoint-clearance-v1' "$cache/frontier-install/edgeai-recovery-owner"; then
    echo 'Rebuild the frontier endpoint check: bash scripts/setup/build-frontier-runtime.sh' >&2; exit 1
fi
export EDGEAI_RUNTIME_PROFILE=${EDGEAI_RUNTIME_PROFILE:-debug}
[[ $EDGEAI_RUNTIME_PROFILE =~ ^(debug|performance)$ ]] || { echo 'Runtime profile must be debug or performance.' >&2; exit 2; }
if [[ $EDGEAI_RUNTIME_PROFILE == performance ]]; then
    export EDGEAI_GUI=${EDGEAI_GUI:-false} EDGEAI_RVIZ_FPS=${EDGEAI_RVIZ_FPS:-15}
else
    export EDGEAI_GUI=${EDGEAI_GUI:-true} EDGEAI_RVIZ_FPS=${EDGEAI_RVIZ_FPS:-30}
fi
export EDGEAI_RVIZ=${EDGEAI_RVIZ:-true}
export EDGEAI_CAMERA_RATE=${EDGEAI_CAMERA_RATE:-15}
export EDGEAI_RVIZ_FPS=${EDGEAI_RVIZ_FPS:-30}
export EDGEAI_ROS_DOMAIN=$domain
controller=${EDGEAI_CONTROLLER:-rpp}
export EDGEAI_GLOBAL_PLANNER=${EDGEAI_GLOBAL_PLANNER:-lattice}
[[ $EDGEAI_GLOBAL_PLANNER == lattice || $EDGEAI_GLOBAL_PLANNER == navfn ]] || { echo 'Planner must be lattice or navfn.' >&2; exit 2; }
[[ $controller == mppi || $controller == dwb || $controller == rpp ]] || { echo 'Controller must be mppi, dwb or rpp.' >&2; exit 2; }
[[ $duration =~ ^[0-9]+$ && $duration -ge 30 && $duration -le 3600 ]] || {
    echo 'Duration must be 30-3600 wall-clock seconds.' >&2; exit 2;
}
[[ $domain =~ ^[0-9]+$ && $domain -le 232 ]] || { echo 'ROS domain must be 0-232.' >&2; exit 2; }
[[ $arena == competition || $arena == simple || $arena == midterm ]] || { echo 'Arena must be competition, simple or midterm.' >&2; exit 2; }
if [[ -n ${EDGEAI_TARGETS:-} || ${EDGEAI_TARGET_COUNT:-0} != 0 ]]; then
    arena_suffix=''
    [[ $arena == simple ]] || arena_suffix="_$arena"
    cooperative_default=false
    [[ $arena == simple ]] || cooperative_default=true
    python3 - "$PROJECT_ROOT" "$arena_suffix" "${EDGEAI_COOPERATIVE_RESCUE:-$cooperative_default}" <<'TARGET_CHECK'
import os
from pathlib import Path
import sys

root = Path(sys.argv[1])
sys.path.insert(0, str(root/'src/edgeai_mission/scripts'))
from prepare_mission import mission_targets

try:
    targets = mission_targets(root/f'src/edgeai_simulation/config/targets{sys.argv[2]}.yaml',
                              sys.argv[3] == 'true', os.environ.get('EDGEAI_TARGETS', ''),
                              int(os.environ.get('EDGEAI_TARGET_COUNT', '0')))
except ValueError as error:
    raise SystemExit(str(error))
print('Required targets: ' + ','.join(f"{t['id']}:{t['type']}" for t in targets))
TARGET_CHECK
fi
echo "ROS backend=$EDGEAI_ROS_BACKEND controller=$controller arena=$arena domain=$domain duration=${duration}s runtime=$EDGEAI_RUNTIME_PROFILE gui=$EDGEAI_GUI rviz=$EDGEAI_RVIZ fps=$EDGEAI_RVIZ_FPS"
if $check_only; then
    for package in edgeai_simulation edgeai_mapping edgeai_navigation edgeai_exploration edgeai_perception edgeai_mission; do
        bash "$PROJECT_ROOT/scripts/ros/ros-command.sh" ros2 pkg prefix "$package"
    done
    if [[ $controller == rpp ]]; then
        bash "$PROJECT_ROOT/scripts/ros/ros-command.sh" ros2 pkg prefix nav2_regulated_pure_pursuit_controller
    fi
    if [[ $EDGEAI_GLOBAL_PLANNER == lattice ]]; then
        bash "$PROJECT_ROOT/scripts/ros/ros-command.sh" ros2 pkg prefix nav2_smac_planner
    fi
    echo 'Runtime check passed; no test was started or stopped.'
    exit 0
fi
# A Windows launcher supplies its own session and observer. Direct WSL callers
# get the same runner, plus scoped process cleanup, map saving and retention.
if [[ -z ${EDGEAI_SESSION_DIR:-} || ${EDGEAI_MANAGED_SESSION:-false} == true ]]; then
    echo 'Stopping registered previous project sessions...'
    python3 "$check_dir/stop-test-sessions.py"
    bash "$check_dir/stop-simulation.sh"
    export EDGEAI_SESSION_DIR="${EDGEAI_SESSION_DIR:-$PROJECT_ROOT/tests/results/runs/$(date +%Y%m%d-%H%M%S)-domain$EDGEAI_ROS_DOMAIN-$$}"
    mkdir -p "$EDGEAI_SESSION_DIR"
    echo "Session logs and map: $EDGEAI_SESSION_DIR"
    echo "Display settings: Gazebo GUI=$EDGEAI_GUI RViz=$EDGEAI_RVIZ. Live events appear here; Ctrl+C stops the test."
    retain_history() {
        result=$?
        trap - EXIT
        echo 'Test stopped. Cleaning completed test history (Windows-mounted files may scan slowly)...'
        python3 "$PROJECT_ROOT/scripts/maintenance/prune_test_results.py" --apply ||
            echo 'Test-history cleanup failed; inspect the output above.' >&2
        exit "$result"
    }
    trap retain_history EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" \
        bash "$check_dir/run-open-source-exploration.sh" "$duration" \
        "${map_basename:-$EDGEAI_SESSION_DIR/map}" "$controller" "$arena" rectangle
    exit 0
fi
exec bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" \
    bash "$check_dir/run-open-source-exploration.sh" "$duration" "$map_basename" "$controller" "$arena" rectangle

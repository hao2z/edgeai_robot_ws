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
  --map PATH               Output map basename; normally generated per run
  --scan360                Scan one full turn before exploration
  --single-robot          Disable cooperative handover
  --navigation-test       Static truth map and manual RViz goals; single robot
  --depth-obstacles config|true|false       Local depth layer (default: config)
  --check                  Check runtime/packages without starting/stopping tests
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
            --navigation-test) export EDGEAI_NAVIGATION_TEST=true EDGEAI_COOPERATIVE_RESCUE=false; shift ;;
            --check) check_only=true; shift ;;
            --duration|--map|--arena|--domain|--controller|--backend|--planner|--depth-obstacles)
                [[ $# -ge 2 && $2 != --* ]] || { echo "Missing value: $1" >&2; exit 2; }
                case "$1" in
                    --duration) duration=$2 ;;
                    --map) map_basename=$2 ;;
                    --arena) arena=$2 ;;
                    --domain) domain=$2 ;;
                    --controller) export EDGEAI_CONTROLLER=$2 ;;
                    --planner) export EDGEAI_GLOBAL_PLANNER=$2 ;;
                    --backend) export EDGEAI_ROS_BACKEND=$2 ;;
                    --depth-obstacles) export EDGEAI_DEPTH_OBSTACLES=$2 ;;
                esac
                shift 2 ;;
            *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
        esac
    done
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
export EDGEAI_GUI=true EDGEAI_RVIZ=true
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
echo "ROS backend=$EDGEAI_ROS_BACKEND controller=$controller arena=$arena domain=$domain duration=${duration}s"
if $check_only; then
    for package in edgeai_simulation edgeai_navigation edgeai_exploration edgeai_perception edgeai_mission; do
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
    echo 'Gazebo and RViz will open; live events appear in this terminal. Ctrl+C stops the test.'
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

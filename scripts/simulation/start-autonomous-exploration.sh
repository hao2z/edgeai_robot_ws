#!/usr/bin/env bash
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set -euo pipefail
check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
usage() {
    cat <<'HELP'
Usage: bash scripts/simulation/start-autonomous-exploration.sh [options]
  --duration SECONDS       Wall-clock test budget, 30..3600 (default: 300)
  --arena simple|competition                 (default: competition)
  --controller dwb|mppi                      (default: dwb)
  --backend native|pixi                      (default: native)
  --domain ID                                (default: 86)
  --map PATH               Output map basename; normally generated per run
  --check                  Check runtime/packages without starting/stopping tests
  --help                   Show this help
Legacy positional form remains: [duration] [map basename] [arena] [domain]
HELP
}
duration=300
map_basename=''
arena=competition
domain=86
check_only=false
if [[ $# -gt 0 && $1 != --* ]]; then
    [[ $# -le 4 ]] || { usage >&2; exit 2; }
    duration=${1:-300}
    map_basename=${2:-}
    arena=${3:-competition}
    domain=${4:-86}
else
    while (( $# )); do
        case "$1" in
            --help) usage; exit 0 ;;
            --check) check_only=true; shift ;;
            --duration|--map|--arena|--domain|--controller|--backend)
                [[ $# -ge 2 && $2 != --* ]] || { echo "Missing value: $1" >&2; exit 2; }
                case "$1" in
                    --duration) duration=$2 ;;
                    --map) map_basename=$2 ;;
                    --arena) arena=$2 ;;
                    --domain) domain=$2 ;;
                    --controller) export EDGEAI_CONTROLLER=$2 ;;
                    --backend) export EDGEAI_ROS_BACKEND=$2 ;;
                esac
                shift 2 ;;
            *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
        esac
    done
fi
source "$PROJECT_ROOT/scripts/ros/workspace-env.sh"
cache=$EDGEAI_RUNTIME_CACHE
if [[ ! -f "$cache/frontier-install/local_setup.bash" ]]; then
    echo 'Nav2/frontier explorer overlay is missing. Run scripts/setup/prepare-open-source.sh first.' >&2
    exit 1
fi
export EDGEAI_GUI=true EDGEAI_RVIZ=true
export EDGEAI_ROS_DOMAIN=$domain
controller=${EDGEAI_CONTROLLER:-dwb}
[[ $controller == mppi || $controller == dwb ]] || { echo 'Controller must be mppi or dwb.' >&2; exit 2; }
[[ $duration =~ ^[0-9]+$ && $duration -ge 30 && $duration -le 3600 ]] || {
    echo 'Duration must be 30-3600 wall-clock seconds.' >&2; exit 2;
}
[[ $domain =~ ^[0-9]+$ && $domain -le 232 ]] || { echo 'ROS domain must be 0-232.' >&2; exit 2; }
[[ $arena == competition || $arena == simple ]] || { echo 'Arena must be competition or simple.' >&2; exit 2; }
echo "ROS backend=$EDGEAI_ROS_BACKEND controller=$controller arena=$arena domain=$domain duration=${duration}s"
if $check_only; then
    for package in edgeai_simulation edgeai_navigation edgeai_exploration; do
        bash "$PROJECT_ROOT/scripts/ros/ros-command.sh" ros2 pkg prefix "$package"
    done
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

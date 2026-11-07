#!/usr/bin/env bash
set -euo pipefail
edgeai_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$edgeai_root"
duration=600; domain=86; params=''; directory=''; extra=()
while (( $# )); do
    case "$1" in
        --help|-h)
            echo 'Usage: bash edgeai.sh cooperative [--params FILE] [--duration SECONDS] [--domain ID] [--directory DIR] [--filter-check]'
            echo 'Defaults: latest run nav2-runtime.yaml, 600 wall seconds, domain 86. Diagnostic only; skips exploration/S1.'
            exit 0 ;;
        --filter-check) extra+=(--filter-check); shift ;;
        --params|--duration|--domain|--directory)
            [[ $# -ge 2 && $2 != --* ]] || { echo "Missing value: $1" >&2; exit 2; }
            case "$1" in
                --params) params=$2 ;;
                --duration) duration=$2 ;;
                --domain) domain=$2 ;;
                --directory) directory=$2 ;;
            esac
            shift 2 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
[[ $duration =~ ^[0-9]+$ && $duration -ge 30 && $duration -le 3600 ]] || { echo 'Duration must be 30..3600 seconds.' >&2; exit 2; }
[[ $domain =~ ^[0-9]+$ && $domain -le 232 ]] || { echo 'Domain must be 0..232.' >&2; exit 2; }
if [[ -z $params ]]; then
    previous=$(cat runtime/latest-session.txt 2>/dev/null || true)
    params="$previous/nav2-runtime.yaml"
fi
[[ -f $params ]] || { echo 'Missing runtime config. Run a mission/navigation test first, or pass --params FILE.' >&2; exit 1; }
params=$(realpath "$params")
source scripts/ros/workspace-env.sh
[[ -f "$EDGEAI_BUILD_ROOT/install/local_setup.bash" && -f "$EDGEAI_RUNTIME_CACHE/frontier-install/local_setup.bash" ]] || {
    echo 'Build the workspace first: bash edgeai.sh build' >&2; exit 1;
}
directory=${directory:-"$edgeai_root/tests/results/runs/$(date +%Y%m%d-%H%M%S-%3N)-cooperative-truth-test"}
directory=$(realpath -m "$directory")
[[ ! -e $directory ]] || { echo 'Output directory already exists; choose a new --directory.' >&2; exit 1; }
python3 scripts/simulation/stop-test-sessions.py
bash scripts/simulation/stop-simulation.sh
mkdir -p "$directory" "$edgeai_root/runtime"
cp -- "$params" "$directory/params-source.yaml"
echo "Cooperative diagnostic logs: $directory"
exec bash scripts/ros/with-ros-env.sh bash scripts/simulation/run-cooperative-truth-test.sh \
    --directory "$directory" --params "$directory/params-source.yaml" \
    --duration "$duration" --domain "$domain" "${extra[@]}"

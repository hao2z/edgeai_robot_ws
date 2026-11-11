#!/usr/bin/env bash
# Linux/WSL entry point. Run from any working directory with bash edgeai.sh.
set -euo pipefail
edgeai_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$edgeai_root"
command=${1:-help}
(( $# == 0 )) || shift
case "$command" in
    help|--help|-h)
        cat <<'HELP'
Usage: bash edgeai.sh COMMAND [options]
  mission       Explore, rescue S1, then cooperative C1 (RPP, 3600s)
  midterm       Independent Part1 then Part2; shared 780s wall budget
  navigation    Static truth-map navigation; select goals manually in RViz
  cooperative   Static-map C1 diagnostic (--params FILE, or latest run)
  simulation    Simulator only (ros2 launch arguments, e.g. robot_count:=2)
  diagnose      Read-only controller probe (--duration 20 --hz 10)
  monitor       Follow latest events.log (optional session directory)
  stop          Stop registered project sessions and simulator
  shell         Interactive project ROS shell
  build         Build explorer overlay and workspace
  check         Check mission dependencies without launching/stopping tests
Mission/navigation: --duration SEC --controller rpp|dwb|mppi --planner lattice|navfn
                    --domain ID --single-robot --depth-obstacles config|true|false --language zh|en
Midterm: add --language en (English) or --language zh (Chinese, default)
See docs/linux-quickstart.md. GUI capture/window automation remains Windows-only.
HELP
        ;;
    mission|navigation|check)
        if [[ $command == navigation ]]; then export EDGEAI_NAVIGATION_TEST=true EDGEAI_COOPERATIVE_RESCUE=false; fi
        options=()
        if [[ $command == check ]]; then options+=(--check); fi
        exec bash scripts/simulation/start-autonomous-exploration.sh "${options[@]}" "$@"
        ;;
    midterm) exec bash scripts/simulation/start-midterm-check.sh "$@" ;;
    cooperative) exec bash scripts/simulation/start-cooperative-test.sh "$@" ;;
    simulation)
        source scripts/display/configure-desktop.sh
        exec bash start_sim.sh "$@"
        ;;
    diagnose) exec bash scripts/ros/with-ros-env.sh python3 scripts/ros/controller_probe.py "$@" ;;
    monitor)
        if (( $# > 1 )); then echo 'Usage: bash edgeai.sh monitor [session directory]' >&2; exit 2; fi
        session=${1:-$(cat runtime/latest-session.txt 2>/dev/null || true)}
        [[ -f "$session/events.log" ]] || { echo 'No session events.log; start a mission first or supply its directory.' >&2; exit 1; }
        exec tail -n 40 -F "$session/events.log"
        ;;
    stop)
        python3 scripts/simulation/stop-test-sessions.py
        exec bash scripts/simulation/stop-simulation.sh
        ;;
    shell) exec bash scripts/ros/open-ros-terminal.sh auto "$@" ;;
    build) exec bash scripts/setup/prepare-open-source.sh "$@" ;;
    *) echo "Unknown command: $command. Run: bash edgeai.sh --help" >&2; exit 2 ;;
esac

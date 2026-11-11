#!/usr/bin/env bash
# Run the independent midterm demonstrations; ordinary mission entry is unchanged.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../ros/workspace-env.sh"
exec bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" python3 "$PROJECT_ROOT/scripts/simulation/midterm_check.py" "$@"

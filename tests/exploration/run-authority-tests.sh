#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../scripts/ros/workspace-env.sh"
if [[ ${EDGEAI_ROS_ENV_ACTIVE:-false} != true ]]; then
    exec bash "$PROJECT_ROOT/scripts/ros/with-ros-env.sh" env EDGEAI_ROS_ENV_ACTIVE=true bash "$0" "$@"
fi
source "$EDGEAI_RUNTIME_CACHE/frontier-install/local_setup.bash"
export ROS_DOMAIN_ID=${EDGEAI_AUTHORITY_TEST_DOMAIN:-96}
python3 "$PROJECT_ROOT/tests/exploration/test_navigation_authority.py"
python3 "$PROJECT_ROOT/tests/exploration/test_authority_ros.py"

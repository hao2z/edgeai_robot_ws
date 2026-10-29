#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/workspace-env.sh"
if [[ $EDGEAI_ROS_BACKEND == native ]]; then
    [[ -z ${CONDA_PREFIX:-} && -z ${PIXI_PROJECT_ROOT:-} ]] || {
        echo 'Start native ROS from a clean WSL shell, outside Pixi/Conda.' >&2; exit 1;
    }
    set +u
    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    exec "$@"
fi
exec pixi run --manifest-path "$EDGEAI_ENV_MANIFEST" "$@"

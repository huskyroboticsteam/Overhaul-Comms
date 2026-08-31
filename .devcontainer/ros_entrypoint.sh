#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

workspace_setup="${ROS_WORKSPACE}/install/setup.bash"
if [[ -f "${workspace_setup}" ]]; then
    source "${workspace_setup}"
fi

exec "$@"

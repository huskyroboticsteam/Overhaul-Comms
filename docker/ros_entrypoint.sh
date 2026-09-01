#!/usr/bin/env bash
# shellcheck disable=SC1091
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/rover_ws/install/setup.bash

exec "$@"

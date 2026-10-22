#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

source /opt/ros/"$ROS_DISTRO"/setup.bash --
source /ros2_ws/install/setup.bash --

# CycloneDDS needs multicast on loopback when ROS_LOCALHOST_ONLY=1
# (only possible when the container has NET_ADMIN)
ip link set lo multicast on 2>/dev/null || true

exec "$@"

#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/go2w_ws/install/setup.bash

exec "$@"

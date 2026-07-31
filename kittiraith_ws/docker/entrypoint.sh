#!/usr/bin/env bash
# =============================================================================
# Container entrypoint: source ROS + workspace overlay, then run the command.
# =============================================================================
set -e

source /opt/ros/noetic/setup.bash

if [[ -f "${ROS_WS}/devel/setup.bash" ]]; then
  source "${ROS_WS}/devel/setup.bash"
fi

exec "$@"

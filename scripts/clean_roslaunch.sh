#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <package> <launchfile> [roslaunch args...]" >&2
  exit 64
fi

cleanup() {
  pkill -f gzserver >/dev/null 2>&1 || true
  pkill -f gzclient >/dev/null 2>&1 || true
  pkill -f rosmaster >/dev/null 2>&1 || true
  pkill -f roscore >/dev/null 2>&1 || true
}

trap cleanup EXIT

# Explicit clean launch for Gazebo-based runs. This is intentionally aggressive:
# it resets any stale Gazebo/ROS master processes left by prior interrupted runs.
cleanup

sleep 1

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

if [[ -f "${HOME}/catkin_ws/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/catkin_ws/devel/setup.bash"
fi

roslaunch "$@"

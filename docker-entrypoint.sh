#!/usr/bin/env bash
set -e

source "/opt/ros/noetic/setup.bash"
source "/albert_ws/devel/setup.bash"

exec "$@"

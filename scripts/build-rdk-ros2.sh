#!/usr/bin/env bash

set -eo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace_dir=$(cd -- "${script_dir}/../ros2_ws" && pwd)

source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="/usr/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"

cd "${workspace_dir}"
colcon build \
    --packages-select action_interfaces action_pkg vla_dataset \
    --symlink-install \
    "$@"

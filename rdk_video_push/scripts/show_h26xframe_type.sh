#!/usr/bin/env bash
set -euo pipefail

TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"

if [ ! -f "$TROS_SETUP" ]; then
  echo "TROS setup file not found: $TROS_SETUP" >&2
  exit 1
fi

set +u
source "$TROS_SETUP"
set -u

ros2 interface show img_msgs/msg/H26XFrame

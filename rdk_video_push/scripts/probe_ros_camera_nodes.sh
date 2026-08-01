#!/usr/bin/env bash
set -u

TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"

if [ ! -f "$TROS_SETUP" ]; then
  echo "TROS setup file not found: $TROS_SETUP" >&2
  exit 1
fi

set +u
source "$TROS_SETUP"
set -u

echo "## ROS2 camera/image/hobot packages"
ros2 pkg list | grep -Ei "usb|camera|mipi|image|hobot" | sort || true

echo
echo "## ROS2 camera/image launch files"
find /opt/tros/humble/share -iname "*launch.py" | grep -Ei "usb|camera|mipi|image" | sort || true

echo
echo "## Current ROS2 topics"
timeout 5 ros2 topic list 2>/dev/null | sort || true

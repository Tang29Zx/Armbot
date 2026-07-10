#!/usr/bin/env bash
set -u

echo "## /dev/video* and /dev/media*"
if ls /dev/video* /dev/media* >/dev/null 2>&1; then
  ls -l /dev/video* /dev/media* 2>/dev/null
else
  echo "No /dev/video* or /dev/media* nodes found."
fi

echo
echo "## v4l2-ctl"
if ! command -v v4l2-ctl >/dev/null 2>&1; then
  echo "v4l2-ctl is not installed."
  echo "Install it with: sudo apt update && sudo apt install -y v4l-utils"
  exit 0
fi

echo "v4l2-ctl: $(command -v v4l2-ctl)"

echo
echo "## V4L2 devices"
v4l2-ctl --list-devices || true

echo
echo "## Supported formats"
for dev in /dev/video*; do
  [ -e "$dev" ] || continue
  echo
  echo "### $dev"
  v4l2-ctl -d "$dev" --all || true
  echo
  v4l2-ctl -d "$dev" --list-formats-ext || true
done

echo
echo "## Suggested camera_device"
if [ -e /dev/video0 ]; then
  echo "/dev/video0"
else
  first_device="$(ls /dev/video* 2>/dev/null | head -n 1 || true)"
  if [ -n "$first_device" ]; then
    echo "$first_device"
  else
    echo "No camera device found."
  fi
fi

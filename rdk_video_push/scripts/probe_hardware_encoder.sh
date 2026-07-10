#!/usr/bin/env bash
set -e

echo "===== 1. Basic devices ====="
ls -l /dev/video* 2>/dev/null || true
ls -l /dev/media* 2>/dev/null || true
ls -l /dev/vpu* /dev/*codec* /dev/*encoder* 2>/dev/null || true

echo
echo "===== 2. FFmpeg encoders ====="
ffmpeg -hide_banner -encoders | grep -Ei "h264|hevc|265|v4l2|omx|hobot|mpp|rkmpp" || true

echo
echo "===== 3. V4L2 devices ====="
if command -v v4l2-ctl >/dev/null 2>&1; then
  v4l2-ctl --list-devices || true

  for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    echo
    echo "----- $dev all -----"
    v4l2-ctl -d "$dev" --all 2>/dev/null | grep -Ei "Driver|Card|Bus|Capabilities|Device Caps|M2M|Memory|Output|Capture" || true

    echo "----- $dev output formats -----"
    v4l2-ctl -d "$dev" --list-formats-out 2>/dev/null || true

    echo "----- $dev capture formats -----"
    v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | head -80 || true
  done
else
  echo "v4l2-ctl not found. Try: sudo apt install v4l-utils"
fi

echo
echo "===== 4. GStreamer plugins ====="
if command -v gst-inspect-1.0 >/dev/null 2>&1; then
  gst-inspect-1.0 | grep -Ei "h264.*enc|h265.*enc|v4l2.*enc|omx|hobot|codec|mpp" || true
else
  echo "gst-inspect-1.0 not found."
fi

echo
echo "===== 5. TROS / hobot_codec ====="
ls /opt/tros 2>/dev/null || true
find /opt/tros -iname "*hobot_codec*" 2>/dev/null | head -50 || true

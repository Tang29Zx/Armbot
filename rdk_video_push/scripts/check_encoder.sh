#!/usr/bin/env bash
set -u

echo "## ffmpeg"
if command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg: $(command -v ffmpeg)"
  ffmpeg -version | sed -n '1,6p'
  echo
  echo "## ffmpeg h264/h265 encoders"
  ffmpeg -hide_banner -encoders 2>/dev/null | grep -Ei '(^ .*(h264|hevc|h265))' || true
  echo
  echo "## ffmpeg protocol support"
  ffmpeg -hide_banner -protocols 2>/dev/null | grep -Ei '(^  srt$|^  rtsp$|^  rtmp$)' || true
  echo
  echo "## ffmpeg encoder smoke tests"
  for enc in h264_v4l2m2m hevc_v4l2m2m h264_omx libx264 libx265; do
    if ffmpeg -hide_banner -encoders 2>/dev/null | grep -q "[[:space:]]$enc[[:space:]]"; then
      printf "%s: " "$enc"
      if timeout 8 ffmpeg -hide_banner -loglevel error \
        -f lavfi -i testsrc=size=320x240:rate=5 \
        -frames:v 5 -an -c:v "$enc" -f null - >/tmp/rdk_video_push_encoder_test.log 2>&1; then
        echo "OK"
      else
        echo "FAILED"
        sed 's/^/  /' /tmp/rdk_video_push_encoder_test.log | tail -n 8
      fi
    fi
  done
else
  echo "ffmpeg: MISSING"
  echo "Install it with: sudo apt update && sudo apt install -y ffmpeg"
fi

echo
echo "## gstreamer"
if command -v gst-launch-1.0 >/dev/null 2>&1; then
  echo "gst-launch-1.0: $(command -v gst-launch-1.0)"
  gst-launch-1.0 --version || true

  echo
  echo "## possible hardware/video encoder plugins"
  if command -v gst-inspect-1.0 >/dev/null 2>&1; then
    gst-inspect-1.0 2>/dev/null | grep -Ei 'h264|h265|hevc|venc|mpp|hobot|x5|vpu|omx|vaapi|v4l2.*enc' || true
  else
    echo "gst-inspect-1.0 is not installed."
  fi
else
  echo "gst-launch-1.0: MISSING"
  echo "Install GStreamer only if you plan to use a GStreamer pipeline."
fi

echo
echo "## note"
echo "This script reports detected encoders only. It does not assume an RDK X5 hardware encoder name."

#!/usr/bin/env bash
set -u

TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"
CODEC_PKG="${CODEC_PKG:-hobot_codec}"
CODEC_LAUNCH="${CODEC_LAUNCH:-hobot_codec_encode.launch.py}"
INPUT_TOPIC="${INPUT_TOPIC:-/hbmem_img}"
OUTPUT_TOPIC="${OUTPUT_TOPIC:-/image_h264}"
OUTPUT_FRAMERATE="${OUTPUT_FRAMERATE:-15}"
LOG_DIR="${LOG_DIR:-/tmp/rdk_video_push_hobot}"
LAUNCH_PID=""
PROCESS_PATTERNS=(
  "/opt/tros/humble/lib/hobot_codec/[h]obot_codec_republish"
  "/opt/tros/humble/lib/hobot_usb_cam/[h]obot_usb_cam"
)

if [ ! -f "$TROS_SETUP" ]; then
  echo "TROS setup file not found: $TROS_SETUP" >&2
  exit 1
fi

mkdir -p "$LOG_DIR"
set +u
source "$TROS_SETUP"
set -u
BASELINE_PIDS="$(
  for pattern in "${PROCESS_PATTERNS[@]}"; do
    pgrep -f "$pattern" || true
  done | sort -u
)"

cleanup_new_hobot_processes() {
  local current_pids pid new_pids
  current_pids="$(
    for pattern in "${PROCESS_PATTERNS[@]}"; do
      pgrep -f "$pattern" || true
    done | sort -u
  )"
  new_pids=""
  for pid in $current_pids; do
    if ! printf '%s\n' "$BASELINE_PIDS" | grep -Fx "$pid" >/dev/null 2>&1; then
      new_pids="$new_pids $pid"
    fi
  done
  if [ -n "$new_pids" ]; then
    echo "Stopping new hobot processes:$new_pids"
    kill $new_pids >/dev/null 2>&1 || true
    sleep 1
    kill -KILL $new_pids >/dev/null 2>&1 || true
  fi
}

cleanup() {
  if [ -n "$LAUNCH_PID" ]; then
    echo
    echo "Stopping hobot codec launch process group pid=$LAUNCH_PID"
    kill -- "-$LAUNCH_PID" >/dev/null 2>&1 || kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
  cleanup_new_hobot_processes
}
trap cleanup EXIT INT TERM

topic_exists() {
  local topic="$1"
  timeout 5 ros2 topic list 2>/dev/null | grep -Fx "$topic" >/dev/null 2>&1
}

wait_topic() {
  local topic="$1"
  local seconds="${2:-10}"
  local i
  for i in $(seq 1 "$seconds"); do
    if topic_exists "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

echo "## Checking input topic"
if topic_exists "$INPUT_TOPIC"; then
  echo "Found $INPUT_TOPIC"
else
  echo "Missing $INPUT_TOPIC; not starting hobot H.264 encode test."
  echo
  echo "Current topics:"
  timeout 5 ros2 topic list 2>/dev/null | sort || true
  exit 2
fi

echo
echo "## Starting hobot_codec H.264 encoder"
echo "ros2 launch $CODEC_PKG $CODEC_LAUNCH codec_in_mode:=shared_mem codec_in_format:=nv12 codec_sub_topic:=$INPUT_TOPIC codec_out_mode:=ros codec_out_format:=h264 codec_pub_topic:=$OUTPUT_TOPIC codec_output_framerate:=$OUTPUT_FRAMERATE"
setsid ros2 launch "$CODEC_PKG" "$CODEC_LAUNCH" \
  codec_in_mode:=shared_mem \
  codec_in_format:=nv12 \
  codec_sub_topic:="$INPUT_TOPIC" \
  codec_out_mode:=ros \
  codec_out_format:=h264 \
  codec_pub_topic:="$OUTPUT_TOPIC" \
  codec_output_framerate:="$OUTPUT_FRAMERATE" \
  >"$LOG_DIR/hobot_encode_h264.log" 2>&1 &
LAUNCH_PID="$!"

if wait_topic "$OUTPUT_TOPIC" 12; then
  echo "Found $OUTPUT_TOPIC"
else
  echo "Did not find $OUTPUT_TOPIC within timeout."
  echo
  echo "Launch log tail:"
  tail -n 80 "$LOG_DIR/hobot_encode_h264.log" || true
  exit 3
fi

echo
echo "## $OUTPUT_TOPIC type"
timeout 5 ros2 topic type "$OUTPUT_TOPIC" || true

echo
echo "## $OUTPUT_TOPIC info"
timeout 5 ros2 topic info -v "$OUTPUT_TOPIC" || true

echo
echo "## $OUTPUT_TOPIC hz"
timeout 12 ros2 topic hz "$OUTPUT_TOPIC" || true

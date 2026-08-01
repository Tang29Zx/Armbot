#!/usr/bin/env bash
set -uo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 'srt://YOUR_VPS_IP:8890?streamid=publish:robot001&latency=200000' [PIPELINE_FPS]" >&2
  exit 1
fi

SRT_URL="$1"
PIPELINE_FPS="${2:-25}"
TROS_SETUP="${TROS_SETUP:-/opt/tros/humble/setup.bash}"
CAMERA_PKG="${CAMERA_PKG:-hobot_usb_cam}"
CAMERA_LAUNCH="${CAMERA_LAUNCH:-hobot_usb_cam.launch.py}"
CODEC_PKG="${CODEC_PKG:-hobot_codec}"
DECODE_LAUNCH="${DECODE_LAUNCH:-hobot_codec.launch.py}"
ENCODE_LAUNCH="${ENCODE_LAUNCH:-hobot_codec_encode.launch.py}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
CAMERA_FPS="${CAMERA_FPS:-30}"
DECODE_INPUT_FPS="${DECODE_INPUT_FPS:-$PIPELINE_FPS}"
DECODE_OUTPUT_FPS="${DECODE_OUTPUT_FPS:-$PIPELINE_FPS}"
ENCODE_INPUT_FPS="${ENCODE_INPUT_FPS:-$PIPELINE_FPS}"
ENCODE_OUTPUT_FPS="${ENCODE_OUTPUT_FPS:-$PIPELINE_FPS}"
BRIDGE_INPUT_FPS="${BRIDGE_INPUT_FPS:-$PIPELINE_FPS}"
BRIDGE_QUEUE_DEPTH="${BRIDGE_QUEUE_DEPTH:-1}"
JPEG_TOPIC="${JPEG_TOPIC:-/image}"
NV12_TOPIC="${NV12_TOPIC:-/image_nv12}"
H264_TOPIC="${H264_TOPIC:-/image_h264}"
LOG_DIR="${LOG_DIR:-logs}"
SUPERVISOR_MAX_RESTARTS="${SUPERVISOR_MAX_RESTARTS:-5}"
SUPERVISOR_RESTART_DELAY="${SUPERVISOR_RESTART_DELAY:-3}"
HEALTH_INTERVAL_SEC="${HEALTH_INTERVAL_SEC:-5}"
IMAGE_STALL_TIMEOUT_SEC="${IMAGE_STALL_TIMEOUT_SEC:-8}"
H264_STALL_TIMEOUT_SEC="${H264_STALL_TIMEOUT_SEC:-8}"
IMAGE_HEALTH_MODE="${IMAGE_HEALTH_MODE:-publisher}"
H264_HEALTH_MODE="${H264_HEALTH_MODE:-publisher}"
PIDS=()
CLEANED_UP=0
STOP_REQUESTED=0
PROCESS_PATTERNS=(
  "/opt/tros/humble/lib/hobot_codec/[h]obot_codec_republish"
  "/opt/tros/humble/lib/hobot_usb_cam/[h]obot_usb_cam"
  "[h]obot_h264_to_srt.py"
  "[f]fmpeg .*pipe:0.*mpegts"
)

if [ ! -f "$TROS_SETUP" ]; then
  echo "TROS setup file not found: $TROS_SETUP" >&2
  exit 1
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
mkdir -p "$LOG_DIR"
: >"$LOG_DIR/hobot_h264_to_srt.log"

if [[ "$SRT_URL" != *"latency="* ]]; then
  echo "SRT URL has no latency parameter. Recommended low-latency start: latency=200000" | tee -a "$LOG_DIR/hobot_h264_to_srt.log"
fi

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
  if [ "$CLEANED_UP" -eq 1 ]; then
    return
  fi
  CLEANED_UP=1
  STOP_REQUESTED=1

  stop_pipeline
}

stop_pipeline() {
  local pid
  for pid in "${PIDS[@]}"; do
    echo "Stopping launch process group pid=$pid"
    kill -- "-$pid" >/dev/null 2>&1 || kill "$pid" >/dev/null 2>&1 || true
    wait "$pid" >/dev/null 2>&1 || true
  done
  PIDS=()
  cleanup_new_hobot_processes
}
trap cleanup EXIT INT TERM HUP

log() {
  echo "$*" | tee -a "$LOG_DIR/hobot_h264_to_srt.log"
}

topic_exists() {
  local topic="$1"
  timeout 5 ros2 topic list 2>/dev/null | grep -Fx "$topic" >/dev/null 2>&1
}

topic_has_publisher() {
  local topic="$1"
  local count
  count="$(timeout 5 ros2 topic info "$topic" 2>/dev/null | awk '/Publisher count:/ {print $3; exit}')"
  [ "${count:-0}" -gt 0 ] 2>/dev/null
}

topic_has_fresh_message() {
  local topic="$1"
  local seconds="${2:-8}"
  timeout "$seconds" ros2 topic hz --window 3 "$topic" 2>/dev/null | grep -m 1 -q "average rate"
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

wait_topic_message() {
  local topic="$1"
  local seconds="${2:-15}"
  topic_has_fresh_message "$topic" "$seconds"
}

start_pipeline() {
  PIDS=()

  log "## Starting USB camera"
  setsid ros2 launch "$CAMERA_PKG" "$CAMERA_LAUNCH" \
    usb_video_device:="$CAMERA_DEVICE" \
    usb_image_width:="$WIDTH" \
    usb_image_height:="$HEIGHT" \
    usb_framerate:="$CAMERA_FPS" \
    usb_pixel_format:=mjpeg \
    usb_zero_copy:=False \
    >>"$LOG_DIR/hobot_h264_to_srt.log" 2>&1 &
  PIDS+=("$!")

  if wait_topic "$JPEG_TOPIC" 15; then
    log "Found $JPEG_TOPIC"
  elif [ "$JPEG_TOPIC" = "/image_jpeg" ] && topic_exists "/image"; then
    image_type="$(timeout 5 ros2 topic type /image 2>/dev/null || true)"
    if [ "$image_type" = "sensor_msgs/msg/CompressedImage" ]; then
      log "Did not find /image_jpeg, using confirmed /image sensor_msgs/msg/CompressedImage"
      JPEG_TOPIC="/image"
    else
      log "Found /image, but type is not sensor_msgs/msg/CompressedImage: $image_type"
      return 3
    fi
  else
    log "Did not find JPEG input topic."
    return 3
  fi

  if ! topic_has_publisher "$JPEG_TOPIC"; then
    log "JPEG input topic exists but has no publisher: $JPEG_TOPIC"
    return 3
  fi

  log "## Starting JPEG -> NV12 decode"
  setsid ros2 launch "$CODEC_PKG" "$DECODE_LAUNCH" \
    codec_in_mode:=ros \
    codec_in_format:=jpeg \
    codec_sub_topic:="$JPEG_TOPIC" \
    codec_out_mode:=ros \
    codec_out_format:=nv12 \
    codec_pub_topic:="$NV12_TOPIC" \
    codec_input_framerate:="$DECODE_INPUT_FPS" \
    codec_output_framerate:="$DECODE_OUTPUT_FPS" \
    >>"$LOG_DIR/hobot_h264_to_srt.log" 2>&1 &
  PIDS+=("$!")

  wait_topic "$NV12_TOPIC" 15 || { log "Missing $NV12_TOPIC"; return 4; }

  log "## Starting NV12 -> H.264 encode"
  setsid ros2 launch "$CODEC_PKG" "$ENCODE_LAUNCH" \
    codec_in_mode:=ros \
    codec_in_format:=nv12 \
    codec_sub_topic:="$NV12_TOPIC" \
    codec_out_mode:=ros \
    codec_out_format:=h264 \
    codec_pub_topic:="$H264_TOPIC" \
    codec_input_framerate:="$ENCODE_INPUT_FPS" \
    codec_output_framerate:="$ENCODE_OUTPUT_FPS" \
    >>"$LOG_DIR/hobot_h264_to_srt.log" 2>&1 &
  PIDS+=("$!")

  wait_topic "$H264_TOPIC" 30 || { log "Missing $H264_TOPIC"; return 5; }
  if ! topic_has_publisher "$H264_TOPIC"; then
    log "H.264 topic exists but has no publisher: $H264_TOPIC"
    return 5
  fi
  if [ "$H264_HEALTH_MODE" = "hz" ] && ! wait_topic_message "$H264_TOPIC" 15; then
    log "No fresh message from $H264_TOPIC during startup"
    return 6
  fi

  log "## Starting H.264 -> SRT bridge"
  setsid python3 rdk_video_push/hobot_h264_to_srt.py \
    --topic "$H264_TOPIC" \
    --srt-url "$SRT_URL" \
    --input-fps "$BRIDGE_INPUT_FPS" \
    --queue-depth "$BRIDGE_QUEUE_DEPTH" \
    >>"$LOG_DIR/hobot_h264_to_srt.log" 2>&1 &
  PIDS+=("$!")
}

monitor_pipeline() {
  local bridge_pid="$1"
  while [ "$STOP_REQUESTED" -eq 0 ]; do
    sleep "$HEALTH_INTERVAL_SEC"

    if ! kill -0 "$bridge_pid" >/dev/null 2>&1; then
      log "Watchdog: bridge process exited; restarting pipeline."
      return 1
    fi

    if ! topic_has_publisher "$JPEG_TOPIC"; then
      log "Watchdog: $JPEG_TOPIC has no publisher; restarting full pipeline."
      return 1
    fi

    if [ "$IMAGE_HEALTH_MODE" = "hz" ] && ! topic_has_fresh_message "$JPEG_TOPIC" "$IMAGE_STALL_TIMEOUT_SEC"; then
      log "Watchdog: no fresh message from $JPEG_TOPIC within ${IMAGE_STALL_TIMEOUT_SEC}s; restarting full pipeline."
      return 1
    fi

    if ! topic_has_publisher "$H264_TOPIC"; then
      log "Watchdog: $H264_TOPIC has no publisher; restarting full pipeline."
      return 1
    fi

    if [ "$H264_HEALTH_MODE" = "hz" ] && ! topic_has_fresh_message "$H264_TOPIC" "$H264_STALL_TIMEOUT_SEC"; then
      log "Watchdog: no fresh message from $H264_TOPIC within ${H264_STALL_TIMEOUT_SEC}s; restarting full pipeline."
      return 1
    fi
  done
  return 0
}

echo "Logging to $LOG_DIR/hobot_h264_to_srt.log"
log "SRT URL: $SRT_URL"
log "Pipeline FPS target: $PIPELINE_FPS"
log "Decode FPS: input=$DECODE_INPUT_FPS output=$DECODE_OUTPUT_FPS"
log "Encode FPS: input=$ENCODE_INPUT_FPS output=$ENCODE_OUTPUT_FPS"
log "Input FPS for ffmpeg timestamps: $BRIDGE_INPUT_FPS"
log "Bridge ROS2 queue depth: $BRIDGE_QUEUE_DEPTH"
log "Watchdog: max_restarts=$SUPERVISOR_MAX_RESTARTS restart_delay=${SUPERVISOR_RESTART_DELAY}s health_interval=${HEALTH_INTERVAL_SEC}s image_mode=${IMAGE_HEALTH_MODE} image_timeout=${IMAGE_STALL_TIMEOUT_SEC}s h264_mode=${H264_HEALTH_MODE} h264_timeout=${H264_STALL_TIMEOUT_SEC}s"

restart_count=0
while [ "$STOP_REQUESTED" -eq 0 ]; do
  CLEANED_UP=0
  log "## Pipeline start attempt $((restart_count + 1))"

  if start_pipeline; then
    bridge_pid="${PIDS[-1]}"
    monitor_pipeline "$bridge_pid"
  else
    status="$?"
    log "Pipeline startup failed with code=$status"
  fi

  if [ "$STOP_REQUESTED" -eq 1 ]; then
    break
  fi

  stop_pipeline
  restart_count=$((restart_count + 1))
  if [ "$restart_count" -gt "$SUPERVISOR_MAX_RESTARTS" ]; then
    log "Watchdog: restart limit exceeded (${SUPERVISOR_MAX_RESTARTS}); exiting."
    exit 10
  fi
  log "Watchdog: restarting in ${SUPERVISOR_RESTART_DELAY}s (restart_count=$restart_count/${SUPERVISOR_MAX_RESTARTS})"
  sleep "$SUPERVISOR_RESTART_DELAY"
done

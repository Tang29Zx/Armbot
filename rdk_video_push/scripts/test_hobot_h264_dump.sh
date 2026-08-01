#!/usr/bin/env bash
set -uo pipefail

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
OUTPUT_FPS="${OUTPUT_FPS:-15}"
JPEG_TOPIC="${JPEG_TOPIC:-/image}"
NV12_TOPIC="${NV12_TOPIC:-/image_nv12}"
H264_TOPIC="${H264_TOPIC:-/image_h264}"
DUMP_SECONDS="${DUMP_SECONDS:-10}"
OUTPUT_FILE="${OUTPUT_FILE:-test_hobot.h264}"
LOG_DIR="${LOG_DIR:-logs}"
PIDS=()
PROCESS_PATTERNS=(
  "/opt/tros/humble/lib/hobot_codec/[h]obot_codec_republish"
  "/opt/tros/humble/lib/hobot_usb_cam/[h]obot_usb_cam"
)

if [ ! -f "$TROS_SETUP" ]; then
  echo "TROS setup file not found: $TROS_SETUP" >&2
  exit 1
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
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
  local pid
  for pid in "${PIDS[@]}"; do
    echo "Stopping launch process group pid=$pid"
    kill -- "-$pid" >/dev/null 2>&1 || kill "$pid" >/dev/null 2>&1 || true
    wait "$pid" >/dev/null 2>&1 || true
  done
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

echo "## Starting USB camera"
setsid ros2 launch "$CAMERA_PKG" "$CAMERA_LAUNCH" \
  usb_video_device:="$CAMERA_DEVICE" \
  usb_image_width:="$WIDTH" \
  usb_image_height:="$HEIGHT" \
  usb_framerate:="$CAMERA_FPS" \
  usb_pixel_format:=mjpeg \
  usb_zero_copy:=False \
  >"$LOG_DIR/hobot_dump_usb_cam.log" 2>&1 &
PIDS+=("$!")

if wait_topic "$JPEG_TOPIC" 15; then
  echo "Found $JPEG_TOPIC"
elif [ "$JPEG_TOPIC" = "/image_jpeg" ] && topic_exists "/image"; then
  image_type="$(timeout 5 ros2 topic type /image 2>/dev/null || true)"
  if [ "$image_type" = "sensor_msgs/msg/CompressedImage" ]; then
    echo "Did not find /image_jpeg, but confirmed /image is sensor_msgs/msg/CompressedImage."
    echo "Using /image as JPEG input topic."
    JPEG_TOPIC="/image"
  else
    echo "Found /image, but type is not sensor_msgs/msg/CompressedImage: $image_type"
    exit 3
  fi
else
  echo "Did not find JPEG input topic."
  timeout 5 ros2 topic list 2>/dev/null | sort || true
  tail -n 120 "$LOG_DIR/hobot_dump_usb_cam.log" || true
  exit 3
fi

echo
echo "## Starting JPEG -> NV12 decode"
setsid ros2 launch "$CODEC_PKG" "$DECODE_LAUNCH" \
  codec_in_mode:=ros \
  codec_in_format:=jpeg \
  codec_sub_topic:="$JPEG_TOPIC" \
  codec_out_mode:=ros \
  codec_out_format:=nv12 \
  codec_pub_topic:="$NV12_TOPIC" \
  codec_output_framerate:="$OUTPUT_FPS" \
  >"$LOG_DIR/hobot_dump_decode.log" 2>&1 &
PIDS+=("$!")

wait_topic "$NV12_TOPIC" 15 || { echo "Missing $NV12_TOPIC"; tail -n 120 "$LOG_DIR/hobot_dump_decode.log" || true; exit 4; }

echo
echo "## Starting NV12 -> H.264 encode"
setsid ros2 launch "$CODEC_PKG" "$ENCODE_LAUNCH" \
  codec_in_mode:=ros \
  codec_in_format:=nv12 \
  codec_sub_topic:="$NV12_TOPIC" \
  codec_out_mode:=ros \
  codec_out_format:=h264 \
  codec_pub_topic:="$H264_TOPIC" \
  codec_output_framerate:="$OUTPUT_FPS" \
  >"$LOG_DIR/hobot_dump_encode.log" 2>&1 &
PIDS+=("$!")

wait_topic "$H264_TOPIC" 15 || { echo "Missing $H264_TOPIC"; tail -n 120 "$LOG_DIR/hobot_dump_encode.log" || true; exit 5; }

echo
echo "## H.264 topic type and hz"
timeout 5 ros2 topic type "$H264_TOPIC" || true
timeout 12 ros2 topic hz "$H264_TOPIC" || true

echo
echo "## Dumping $H264_TOPIC to $OUTPUT_FILE"
rm -f "$OUTPUT_FILE"
if ! python3 rdk_video_push/hobot_h264_dump.py --topic "$H264_TOPIC" --output "$OUTPUT_FILE" --seconds "$DUMP_SECONDS" \
  2>&1 | tee "$LOG_DIR/hobot_h264_dump.log"; then
  echo "hobot_h264_dump.py failed"
  exit 6
fi

echo
echo "## Output file"
ls -lh "$OUTPUT_FILE"

echo
echo "## ffprobe $OUTPUT_FILE"
if ffprobe "$OUTPUT_FILE" 2>&1 | tee "$LOG_DIR/hobot_h264_ffprobe.log"; then
  echo
  echo "ffprobe recognized $OUTPUT_FILE"
  echo "You can try: ffplay $OUTPUT_FILE"
else
  echo
  echo "ffprobe did not recognize $OUTPUT_FILE"
  exit 7
fi

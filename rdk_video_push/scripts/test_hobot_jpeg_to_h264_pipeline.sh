#!/usr/bin/env bash
set -u

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
LOG_DIR="${LOG_DIR:-/tmp/rdk_video_push_hobot}"
PIDS=()
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

require_pkg_and_launch() {
  local pkg="$1"
  local launch="$2"
  if ! ros2 pkg list | grep -Fx "$pkg" >/dev/null 2>&1; then
    echo "Missing ROS2 package: $pkg" >&2
    return 1
  fi
  if ! find "/opt/tros/humble/share/$pkg/launch" -maxdepth 1 -name "$launch" -type f >/dev/null 2>&1; then
    echo "Missing launch file: /opt/tros/humble/share/$pkg/launch/$launch" >&2
    return 1
  fi
}

echo "## Confirming package and launch files"
require_pkg_and_launch "$CAMERA_PKG" "$CAMERA_LAUNCH" || exit 2
require_pkg_and_launch "$CODEC_PKG" "$DECODE_LAUNCH" || exit 2
require_pkg_and_launch "$CODEC_PKG" "$ENCODE_LAUNCH" || exit 2
echo "Camera launch: $CAMERA_PKG/$CAMERA_LAUNCH"
echo "Codec decode launch: $CODEC_PKG/$DECODE_LAUNCH"
echo "Codec encode launch: $CODEC_PKG/$ENCODE_LAUNCH"

echo
echo "## Starting USB camera candidate"
echo "ros2 launch $CAMERA_PKG $CAMERA_LAUNCH usb_video_device:=$CAMERA_DEVICE usb_image_width:=$WIDTH usb_image_height:=$HEIGHT usb_framerate:=$CAMERA_FPS usb_pixel_format:=mjpeg usb_zero_copy:=False"
setsid ros2 launch "$CAMERA_PKG" "$CAMERA_LAUNCH" \
  usb_video_device:="$CAMERA_DEVICE" \
  usb_image_width:="$WIDTH" \
  usb_image_height:="$HEIGHT" \
  usb_framerate:="$CAMERA_FPS" \
  usb_pixel_format:=mjpeg \
  usb_zero_copy:=False \
  >"$LOG_DIR/hobot_usb_cam.log" 2>&1 &
PIDS+=("$!")

if wait_topic "$JPEG_TOPIC" 15; then
  echo "Found $JPEG_TOPIC from USB camera path"
else
  if [ "$JPEG_TOPIC" = "/image_jpeg" ] && topic_exists "/image"; then
    image_type="$(timeout 5 ros2 topic type /image 2>/dev/null || true)"
    if [ "$image_type" = "sensor_msgs/msg/CompressedImage" ]; then
      echo "Did not find /image_jpeg, but confirmed /image is sensor_msgs/msg/CompressedImage."
      echo "Using /image as the JPEG input topic for this host."
      JPEG_TOPIC="/image"
    else
      echo "Found /image, but its type is not sensor_msgs/msg/CompressedImage: $image_type"
      exit 3
    fi
  else
    echo "Did not find $JPEG_TOPIC. This script will not guess another camera topic."
    echo
    echo "Current topics:"
    timeout 5 ros2 topic list 2>/dev/null | sort || true
    echo
    echo "USB camera launch log tail:"
    tail -n 120 "$LOG_DIR/hobot_usb_cam.log" || true
    exit 3
  fi
fi

echo
echo "## Starting JPEG -> NV12 decode"
echo "ros2 launch $CODEC_PKG $DECODE_LAUNCH codec_in_mode:=ros codec_in_format:=jpeg codec_sub_topic:=$JPEG_TOPIC codec_out_mode:=ros codec_out_format:=nv12 codec_pub_topic:=$NV12_TOPIC codec_output_framerate:=$OUTPUT_FPS"
setsid ros2 launch "$CODEC_PKG" "$DECODE_LAUNCH" \
  codec_in_mode:=ros \
  codec_in_format:=jpeg \
  codec_sub_topic:="$JPEG_TOPIC" \
  codec_out_mode:=ros \
  codec_out_format:=nv12 \
  codec_pub_topic:="$NV12_TOPIC" \
  codec_output_framerate:="$OUTPUT_FPS" \
  >"$LOG_DIR/hobot_decode_jpeg_to_nv12.log" 2>&1 &
PIDS+=("$!")

if wait_topic "$NV12_TOPIC" 15; then
  echo "Found $NV12_TOPIC"
else
  echo "Did not find $NV12_TOPIC."
  echo
  echo "Decode launch log tail:"
  tail -n 120 "$LOG_DIR/hobot_decode_jpeg_to_nv12.log" || true
  exit 4
fi

echo
echo "## Starting NV12 -> H.264 encode"
echo "ros2 launch $CODEC_PKG $ENCODE_LAUNCH codec_in_mode:=ros codec_in_format:=nv12 codec_sub_topic:=$NV12_TOPIC codec_out_mode:=ros codec_out_format:=h264 codec_pub_topic:=$H264_TOPIC codec_output_framerate:=$OUTPUT_FPS"
setsid ros2 launch "$CODEC_PKG" "$ENCODE_LAUNCH" \
  codec_in_mode:=ros \
  codec_in_format:=nv12 \
  codec_sub_topic:="$NV12_TOPIC" \
  codec_out_mode:=ros \
  codec_out_format:=h264 \
  codec_pub_topic:="$H264_TOPIC" \
  codec_output_framerate:="$OUTPUT_FPS" \
  >"$LOG_DIR/hobot_encode_nv12_to_h264.log" 2>&1 &
PIDS+=("$!")

if wait_topic "$H264_TOPIC" 15; then
  echo "Found $H264_TOPIC"
else
  echo "Did not find $H264_TOPIC."
  echo
  echo "Encode launch log tail:"
  tail -n 120 "$LOG_DIR/hobot_encode_nv12_to_h264.log" || true
  exit 5
fi

echo
echo "## $H264_TOPIC type"
timeout 5 ros2 topic type "$H264_TOPIC" || true

echo
echo "## $H264_TOPIC info"
timeout 5 ros2 topic info -v "$H264_TOPIC" || true

echo
echo "## $H264_TOPIC hz"
timeout 12 ros2 topic hz "$H264_TOPIC" || true

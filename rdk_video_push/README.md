# RDK X5 Camera Video Push

This module captures video from an RDK X5 camera device, encodes it with FFmpeg, and pushes it to a remote MediaMTX server over SRT by default.

The first version defaults to software H.264 (`libx264`) so the SRT path can be verified. Hardware encoding is exposed through `config.yaml` via `video.encoder_mode` and `video.encoder_name`, but the module does not pretend to know the RDK X5 hardware encoder name until the detection script confirms it.

## Directory

```text
rdk_video_push/
  config.yaml
  README.md
  requirements.txt
  main.py
  rdk_video_push/
    __init__.py
    config.py
    logger.py
    video_push.py
  scripts/
    check_camera.sh
    check_encoder.sh
    run_video_push.sh
```

## Install Dependencies

```bash
cd /home/sunrise/Armbot/rdk_video_push
sudo apt update
sudo apt install -y python3 python3-pip ffmpeg v4l-utils
python3 -m pip install -r requirements.txt
```

GStreamer is optional for this version. Install it only if you plan to switch to a GStreamer pipeline later.

## Check Camera

```bash
cd /home/sunrise/Armbot/rdk_video_push
bash scripts/check_camera.sh
```

Expected useful output includes `/dev/video0`, supported formats, resolutions, and frame rates. For the currently detected USB camera, `/dev/video0` is the image stream and `/dev/video1` is UVC metadata.

If the camera driver reports only 30fps for a format, FFmpeg may capture at 30fps. The module still applies `video.fps` on the output side with `-r`, so the default stream is encoded at 15fps.

## Check Encoders

```bash
cd /home/sunrise/Armbot/rdk_video_push
bash scripts/check_encoder.sh
```

Look for FFmpeg encoders such as `libx264`, `libx265`, or a real hardware encoder name. If a hardware encoder is confirmed, set it in `config.yaml`:

```yaml
video:
  encoder_mode: "hardware"
  encoder_name: "CONFIRMED_ENCODER_NAME"
```

Do not set a guessed hardware encoder name.

## Configure Stream

Edit `config.yaml`:

```yaml
video:
  camera_device: "/dev/video0"
  width: 1280
  height: 720
  fps: 15
  codec: "h264"
  bitrate: "2500k"
  srt_url: "srt://YOUR_VPS_IP:8890?streamid=publish:robot001"
  transport: "srt"
```

Replace `YOUR_VPS_IP` with the VPS public IP or domain. Keep audio disabled for this module.

To preview the generated FFmpeg command:

```bash
python3 main.py --print-command
```

## Start Stream

```bash
cd /home/sunrise/Armbot/rdk_video_push
bash scripts/run_video_push.sh
```

Use a custom config path if needed:

```bash
bash scripts/run_video_push.sh ./config.yaml
```

## Stop Stream

Press `Ctrl+C` in the terminal running the streamer. The Python process catches the signal and terminates FFmpeg cleanly.

You can also stop it from another shell:

```bash
pkill -f 'rdk_video_push/main.py'
```

## Confirm MediaMTX Receives the Stream

On the VPS, check the MediaMTX logs. You should see an SRT publisher connected for stream `robot001`.

Typical checks:

```bash
sudo journalctl -u mediamtx -f
```

or, if MediaMTX is running manually:

```bash
tail -f mediamtx.log
```

The configured SRT URL uses:

```text
srt://YOUR_VPS_IP:8890?streamid=publish:robot001
```

Make sure the VPS firewall allows UDP port `8890`.

## Pull Stream Test

From a high-compute machine with FFmpeg:

```bash
ffplay "srt://YOUR_VPS_IP:8890?streamid=read:robot001"
```

If MediaMTX exposes RTSP for the same path:

```bash
ffplay rtsp://YOUR_VPS_IP:8554/robot001
```

For browser playback, use a MediaMTX browser-compatible output such as WebRTC or HLS if configured on the VPS:

```text
http://YOUR_VPS_IP:8888/robot001/
```

The exact browser URL depends on the MediaMTX configuration.

## ROS2 Hobot Low-Latency SRT Experiment

This experimental path keeps the default `libx264` main path unchanged and bridges the already hardware-encoded ROS2 topic:

```text
/image_h264 -> hobot_h264_to_srt.py -> ffmpeg -c:v copy -> SRT
```

Recommended RDK push command:

```bash
cd /home/sunrise/Armbot/rdk_video_push

bash scripts/test_hobot_h264_to_srt.sh \
  "srt://150.158.51.242:8890?streamid=publish:robot001&latency=200000" \
  25
```

Do not use `latency=2000000` for low-latency testing; it is about 2 seconds. Start with `latency=200000`, try `latency=120000` for lower delay, or `latency=500000` for more stability.

The bridge uses FFmpeg timestamp generation and low muxing delay while preserving stream copy:

```bash
ffmpeg -hide_banner -loglevel info \
  -fflags +genpts \
  -use_wallclock_as_timestamps 1 \
  -r 25 \
  -f h264 \
  -i pipe:0 \
  -c:v copy \
  -muxdelay 0 \
  -muxpreload 0 \
  -flush_packets 1 \
  -f mpegts \
  "$SRT_URL"
```

On the VPS:

```bash
sudo docker logs -f mediamtx
```

Low-latency RTSP pull over TCP:

```bash
ffplay \
  -fflags nobuffer \
  -flags low_delay \
  -framedrop \
  -rtsp_transport tcp \
  -probesize 32 \
  -analyzeduration 0 \
  rtsp://150.158.51.242:8554/robot001
```

If UDP ports are open:

```bash
ffplay \
  -fflags nobuffer \
  -flags low_delay \
  -framedrop \
  -rtsp_transport udp \
  -probesize 32 \
  -analyzeduration 0 \
  rtsp://150.158.51.242:8554/robot001
```

TCP is stable but may accumulate delay during jitter. UDP may be lower-latency but can drop packets. HLS is not recommended for this low-latency test.

Simple end-to-end latency test: open a stopwatch on a phone, put it in front of the RDK camera, and compare the real stopwatch with the `ffplay` image on the high-compute machine.

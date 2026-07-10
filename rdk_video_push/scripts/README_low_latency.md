# ROS2 Hobot H.264 Low-Latency SRT Test

This experimental path uses:

```text
hobot_usb_cam -> /image -> hobot_codec decode -> /image_nv12
  -> hobot_codec encode h264 -> /image_h264
  -> hobot_h264_to_srt.py -> ffmpeg -c:v copy -> SRT
```

It does not modify the default `libx264` main path.

## RDK Push

Recommended first test:

```bash
cd /home/sunrise/Armbot/rdk_video_push

bash scripts/test_hobot_h264_to_srt.sh \
  "srt://150.158.51.242:8890?streamid=publish:robot001&latency=200000" \
  25
```

Do not use `latency=2000000` for low-latency tests. It is about 2 seconds and will visibly increase delay.

Suggested SRT latency values:

```text
latency=200000: about 0.2 seconds, try first
latency=120000: lower latency, may stutter on weaker networks
latency=500000: more stable, higher delay
```

The bridge starts FFmpeg with generated timestamps and low muxing delay:

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

`-r 25` is an input frame rate and must appear before `-i pipe:0`. The bridge keeps `-c:v copy`, so FFmpeg does not re-encode.

## VPS Check

```bash
sudo docker logs -f mediamtx
```

Look for `robot001`, `H264`, and publishing/online messages.

## Low-Latency Pull

RTSP over TCP is usually more stable:

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

If UDP ports are open, test UDP too:

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

TCP is stable, but network jitter can accumulate delay. UDP may be lower-latency but can drop packets. HLS is not for this low-latency test.

## Simple End-to-End Latency Test

Open a stopwatch on a phone, put it in front of the RDK camera, and watch the `ffplay` output on the high-compute machine. The difference between the real phone stopwatch and the displayed stopwatch is the approximate end-to-end delay.

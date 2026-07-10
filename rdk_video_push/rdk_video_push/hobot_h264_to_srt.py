#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shlex
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from typing import Any

import rclpy
from img_msgs.msg import H26XFrame
from rclpy.node import Node


class H264ToSrtNode(Node):
    def __init__(
        self,
        topic: str,
        srt_url: str,
        ffmpeg_path: str = "ffmpeg",
        input_fps: float = 7.5,
        ffmpeg_loglevel: str = "info",
        low_latency: bool = True,
        queue_depth: int = 1,
    ) -> None:
        super().__init__("hobot_h264_to_srt")
        self.topic = topic
        self.srt_url = srt_url
        self.input_fps = input_fps
        self.frame_count = 0
        self.total_bytes = 0
        self.last_report_bytes = 0
        self.write_count_since_report = 0
        self.write_time_sum_since_report = 0.0
        self.write_time_max_since_report = 0.0
        self.skipped_frames = 0
        self.started_streaming = False
        self.start_time = time.monotonic()
        self.first_h264_frame_time: float | None = None
        self.sps_found_time: float | None = None
        self.first_write_to_ffmpeg_time: float | None = None
        self.last_report_time = self.start_time
        self.last_report_frames = 0
        self.done = False
        self.invalid_structure_reported = False
        self.stderr_lines: deque[str] = deque(maxlen=200)

        self.command = [
            ffmpeg_path,
            "-hide_banner",
            "-loglevel",
            ffmpeg_loglevel,
            "-fflags",
            "+genpts",
            "-use_wallclock_as_timestamps",
            "1",
            "-r",
            format_fps(input_fps),
            "-f",
            "h264",
            "-i",
            "pipe:0",
            "-c:v",
            "copy",
        ]
        if low_latency:
            self.command.extend([
                "-muxdelay",
                "0",
                "-muxpreload",
                "0",
                "-flush_packets",
                "1",
            ])
        self.command.extend([
            "-f",
            "mpegts",
            srt_url,
        ])
        print("ffmpeg command:", shlex.join(self.command), flush=True)

        self.process = subprocess.Popen(
            self.command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=False,
            bufsize=0,
        )
        threading.Thread(target=self._read_stderr, daemon=True).start()

        self.get_logger().info(f"Subscribing topic: {topic}, queue_depth={queue_depth}")
        self.subscription = self.create_subscription(H26XFrame, topic, self.on_frame, queue_depth)

    def close(self) -> None:
        self.done = True
        process = self.process
        if process.stdin and not process.stdin.closed:
            try:
                process.stdin.close()
            except BrokenPipeError:
                pass

        try:
            exit_code = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                exit_code = process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                exit_code = process.wait()

        print(f"ffmpeg exited with code={exit_code}", flush=True)
        print(
            f"summary: frames={self.frame_count} skipped_before_sps={self.skipped_frames} "
            f"bytes={self.total_bytes}",
            flush=True,
        )
        if exit_code != 0 or self.stderr_lines:
            print("ffmpeg stderr tail:", flush=True)
            for line in list(self.stderr_lines)[-80:]:
                print(line, file=sys.stderr, flush=True)

    def on_frame(self, msg: H26XFrame) -> None:
        now = time.monotonic()
        if self.first_h264_frame_time is None:
            self.first_h264_frame_time = now
            print(f"first H264 frame received at: {now:.6f}", flush=True)

        if self.process.poll() is not None:
            print(f"ffmpeg exited early with code={self.process.returncode}", flush=True)
            self.done = True
            return

        payload = self.extract_payload(msg)
        if payload is None:
            self.done = True
            return

        nal_types = h264_nal_types(payload)
        if not self.started_streaming:
            if 7 not in nal_types:
                self.skipped_frames += 1
                print(
                    f"skipping frame before SPS: skipped={self.skipped_frames} "
                    f"len={len(payload)} nal_types={nal_types}",
                    flush=True,
                )
                return
            self.started_streaming = True
            self.sps_found_time = now
            assert self.first_h264_frame_time is not None
            time_to_sps_sec = self.sps_found_time - self.first_h264_frame_time
            print(f"SPS found after: {time_to_sps_sec:.3f} sec", flush=True)
            print(f"sps_found_time: {self.sps_found_time:.6f}", flush=True)
            print(f"skipped frames before SPS: {self.skipped_frames}", flush=True)
            print(f"found SPS nal_types={nal_types}", flush=True)
            print("start writing to ffmpeg", flush=True)

        try:
            assert self.process.stdin is not None
            write_start = time.monotonic()
            self.process.stdin.write(payload)
            write_elapsed = time.monotonic() - write_start
            self.write_count_since_report += 1
            self.write_time_sum_since_report += write_elapsed
            self.write_time_max_since_report = max(self.write_time_max_since_report, write_elapsed)
            if self.first_write_to_ffmpeg_time is None:
                self.first_write_to_ffmpeg_time = time.monotonic()
                print(f"first_write_to_ffmpeg_time: {self.first_write_to_ffmpeg_time:.6f}", flush=True)
        except (BrokenPipeError, OSError) as exc:
            print(f"failed to write to ffmpeg stdin: {exc}", file=sys.stderr, flush=True)
            self.done = True
            return

        self.frame_count += 1
        self.total_bytes += len(payload)
        self.report_frame(len(payload), nal_types)

    def extract_payload(self, msg: H26XFrame) -> bytes | None:
        if not hasattr(msg, "data"):
            self.report_invalid_structure(msg)
            return None
        try:
            return bytes(getattr(msg, "data"))
        except TypeError:
            self.report_invalid_structure(msg)
            return None

    def report_invalid_structure(self, msg: Any) -> None:
        if self.invalid_structure_reported:
            return
        self.invalid_structure_reported = True
        attrs = [name for name in dir(msg) if not name.startswith("_")]
        print("H26XFrame does not expose a byte-like 'data' field at runtime.", file=sys.stderr, flush=True)
        print(f"Message class: {type(msg)}", file=sys.stderr, flush=True)
        print(f"Available public attributes: {attrs}", file=sys.stderr, flush=True)
        print(f"Message repr: {msg!r}", file=sys.stderr, flush=True)

    def report_frame(self, frame_len: int, nal_types: list[int]) -> None:
        now = time.monotonic()
        if self.frame_count == 1 or now - self.last_report_time >= 1.0:
            elapsed = max(now - self.start_time, 1e-6)
            window_elapsed = max(now - self.last_report_time, 1e-6)
            current_fps = (self.frame_count - self.last_report_frames) / window_elapsed
            avg_fps = self.frame_count / elapsed
            window_bytes = self.total_bytes - self.last_report_bytes
            window_mbps = (window_bytes * 8.0) / window_elapsed / 1_000_000.0
            avg_mbps = (self.total_bytes * 8.0) / elapsed / 1_000_000.0
            avg_write_ms = 0.0
            max_write_ms = 0.0
            if self.write_count_since_report:
                avg_write_ms = (self.write_time_sum_since_report / self.write_count_since_report) * 1000.0
                max_write_ms = self.write_time_max_since_report * 1000.0
            print(
                f"frames={self.frame_count} len={frame_len} bytes={self.total_bytes} "
                f"fps={current_fps:.2f} avg_fps={avg_fps:.2f} "
                f"mbps={window_mbps:.2f} avg_mbps={avg_mbps:.2f} "
                f"write_ms_avg={avg_write_ms:.2f} write_ms_max={max_write_ms:.2f} "
                f"nal_types={nal_types}",
                flush=True,
            )
            self.last_report_time = now
            self.last_report_frames = self.frame_count
            self.last_report_bytes = self.total_bytes
            self.write_count_since_report = 0
            self.write_time_sum_since_report = 0.0
            self.write_time_max_since_report = 0.0

    def _read_stderr(self) -> None:
        assert self.process.stderr is not None
        for raw_line in self.process.stderr:
            line = raw_line.decode("utf-8", errors="replace").rstrip()
            self.stderr_lines.append(line)
            print(f"[ffmpeg] {line}", file=sys.stderr, flush=True)


def h264_nal_types(payload: bytes) -> list[int]:
    types: list[int] = []
    i = 0
    size = len(payload)
    while i + 4 <= size:
        start_len = 0
        if payload[i:i + 4] == b"\x00\x00\x00\x01":
            start_len = 4
        elif payload[i:i + 3] == b"\x00\x00\x01":
            start_len = 3

        if start_len:
            nal_index = i + start_len
            if nal_index < size:
                types.append(payload[nal_index] & 0x1F)
            i = nal_index + 1
        else:
            i += 1
    return types


def format_fps(value: float) -> str:
    return f"{value:g}"


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "true", "yes", "y", "on"):
        return True
    if normalized in ("0", "false", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge img_msgs/msg/H26XFrame H.264 payload to SRT via ffmpeg copy")
    parser.add_argument("--topic", default="/image_h264")
    parser.add_argument("--srt-url", required=True)
    parser.add_argument("--ffmpeg-path", default="ffmpeg")
    parser.add_argument("--input-fps", type=float, default=7.5)
    parser.add_argument("--ffmpeg-loglevel", default="info")
    parser.add_argument("--low-latency", type=parse_bool, default=True)
    parser.add_argument("--queue-depth", type=int, default=1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = H264ToSrtNode(
        args.topic,
        args.srt_url,
        args.ffmpeg_path,
        args.input_fps,
        args.ffmpeg_loglevel,
        args.low_latency,
        args.queue_depth,
    )

    def handle_signal(_signum: int, _frame: object) -> None:
        node.done = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if node.frame_count > 0 and not node.invalid_structure_reported else 2


if __name__ == "__main__":
    raise SystemExit(main())

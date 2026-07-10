#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path
from typing import Any

import rclpy
from img_msgs.msg import H26XFrame
from rclpy.node import Node


class H264DumpNode(Node):
    def __init__(self, topic: str, output: Path, seconds: float) -> None:
        super().__init__("hobot_h264_dump")
        self.topic = topic
        self.output = output
        self.seconds = seconds
        self.start_time = time.monotonic()
        self.frame_count = 0
        self.done = False
        self.last_report_time = self.start_time
        self.last_report_frames = 0
        self.total_bytes = 0
        self.skipped_frames = 0
        self.started_writing = False
        self.keyframe_fields = [name for name in dir(H26XFrame()) if "key" in name.lower() or "idr" in name.lower()]
        self.invalid_structure_reported = False
        self.file = output.open("wb")

        self.get_logger().info(f"Subscribing topic: {topic}")
        self.get_logger().info(f"Writing raw H.264 to: {output}")
        if self.keyframe_fields:
            self.get_logger().info(f"Keyframe-like fields found: {self.keyframe_fields}")
        else:
            self.get_logger().info("Keyframe-like fields found: none")

        self.subscription = self.create_subscription(H26XFrame, topic, self.on_frame, 10)

    def close(self) -> None:
        if not self.file.closed:
            self.file.flush()
            self.file.close()

    def on_frame(self, msg: H26XFrame) -> None:
        if not hasattr(msg, "data"):
            self.report_invalid_structure(msg)
            return

        data = getattr(msg, "data")
        try:
            payload = bytes(data)
        except TypeError:
            self.report_invalid_structure(msg)
            return

        nal_types = h264_nal_types(payload)
        if not self.started_writing:
            if 7 not in nal_types:
                self.skipped_frames += 1
                elapsed = time.monotonic() - self.start_time
                self.get_logger().info(
                    f"skipping frame before SPS: skipped={self.skipped_frames} len={len(payload)} "
                    f"nal_types={nal_types}"
                )
                if elapsed >= self.seconds:
                    self.get_logger().error(f"No SPS found within {self.seconds:.1f}s; output is not usable")
                    self.done = True
                return
            self.started_writing = True
            self.get_logger().info(f"found SPS; start writing from this frame nal_types={nal_types}")

        now = time.monotonic()
        self.file.write(payload)
        self.frame_count += 1
        self.total_bytes += len(payload)

        elapsed = max(now - self.start_time, 1e-6)
        window_elapsed = max(now - self.last_report_time, 1e-6)
        current_fps = (self.frame_count - self.last_report_frames) / window_elapsed
        avg_fps = self.frame_count / elapsed

        encoding = _decode_fixed_uint8_string(getattr(msg, "encoding", []))
        keyframe_state = "no keyframe field"
        if self.keyframe_fields:
            keyframe_state = ", ".join(f"{name}={getattr(msg, name)}" for name in self.keyframe_fields)

        file_size = self.output.stat().st_size if self.output.exists() else self.total_bytes
        self.get_logger().info(
            f"frame={self.frame_count} len={len(payload)} encoding={encoding or 'unknown'} nal_types={nal_types} "
            f"fps={current_fps:.2f} avg_fps={avg_fps:.2f} keyframe={keyframe_state} "
            f"file_size={file_size}"
        )

        self.last_report_time = now
        self.last_report_frames = self.frame_count

        if elapsed >= self.seconds:
            self.get_logger().info(f"Capture duration reached {self.seconds:.1f}s")
            self.done = True

    def report_invalid_structure(self, msg: Any) -> None:
        if self.invalid_structure_reported:
            return
        self.invalid_structure_reported = True
        attrs = [name for name in dir(msg) if not name.startswith("_")]
        self.get_logger().error("H26XFrame does not expose a byte-like 'data' field at runtime.")
        self.get_logger().error(f"Message class: {type(msg)}")
        self.get_logger().error(f"Available public attributes: {attrs}")
        self.get_logger().error(f"Message repr: {msg!r}")
        self.done = True


def _decode_fixed_uint8_string(value: Any) -> str:
    try:
        raw = bytes(value)
    except TypeError:
        return ""
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="ignore")


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dump img_msgs/msg/H26XFrame H.264 payload to a raw .h264 file")
    parser.add_argument("--topic", default="/image_h264")
    parser.add_argument("--output", default="test_hobot.h264")
    parser.add_argument("--seconds", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        output.unlink()

    rclpy.init()
    node = H264DumpNode(args.topic, output, args.seconds)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    size = output.stat().st_size if output.exists() else 0
    print(
        f"summary: frames={node.frame_count} skipped_before_sps={node.skipped_frames} "
        f"bytes={node.total_bytes} output={output} file_size={size}"
    )
    return 0 if node.frame_count > 0 and size > 0 and not node.invalid_structure_reported else 2


if __name__ == "__main__":
    raise SystemExit(main())

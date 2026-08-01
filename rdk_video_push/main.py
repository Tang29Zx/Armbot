#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from rdk_video_push.config import load_config
from rdk_video_push.logger import setup_logger
from rdk_video_push.video_push import VideoPusher


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDK X5 camera SRT/RTSP video pusher")
    parser.add_argument(
        "-c",
        "--config",
        default=str(Path(__file__).resolve().parent / "config.yaml"),
        help="Path to config.yaml",
    )
    parser.add_argument(
        "--print-command",
        action="store_true",
        help="Print the generated ffmpeg command and exit",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    logger = setup_logger(config["log"].get("level", "INFO"))
    pusher = VideoPusher(config, logger)

    if args.print_command:
        import shlex

        print(shlex.join(pusher.build_command()))
        return 0

    pusher.start_stream()
    return pusher.wait()


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any


DEFAULT_CONFIG: dict[str, Any] = {
    "video": {
        "camera_device": "/dev/video0",
        "width": 1280,
        "height": 720,
        "fps": 15,
        "codec": "h264",
        "bitrate": "2500k",
        "srt_url": "srt://YOUR_VPS_IP:8890?streamid=publish:robot001",
        "transport": "srt",
        "encoder_mode": "software",
        "encoder_name": "",
        "ffmpeg_path": "ffmpeg",
        "input_format": "mjpeg",
        "output_format": "mpegts",
        "extra_input_args": [],
        "extra_encoder_args": [],
        "extra_output_args": [],
    },
    "runtime": {
        "auto_restart": True,
        "max_restart_count": 5,
        "restart_interval_sec": 3,
        "stop_timeout_sec": 5,
    },
    "log": {
        "level": "INFO",
    },
}


def load_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    data = _read_yaml(config_path)
    config = deepcopy(DEFAULT_CONFIG)
    _deep_update(config, data)
    _validate_config(config)
    return config


def _read_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError:
        return _read_simple_yaml(path)

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError("config.yaml must contain a YAML mapping")
    return data


def _read_simple_yaml(path: Path) -> dict[str, Any]:
    """Small fallback parser for this project's simple config.yaml shape."""
    result: dict[str, Any] = {}
    current_section: str | None = None

    for line_no, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.split("#", 1)[0].rstrip()
        if not line:
            continue

        if not line.startswith(" "):
            if not line.endswith(":"):
                raise ValueError(f"Unsupported YAML line {line_no}: {raw_line}")
            current_section = line[:-1].strip()
            result[current_section] = {}
            continue

        if current_section is None or ":" not in line:
            raise ValueError(f"Unsupported YAML line {line_no}: {raw_line}")

        key, value = line.strip().split(":", 1)
        result[current_section][key.strip()] = _parse_scalar(value.strip())

    return result


def _parse_scalar(value: str) -> Any:
    if value == "":
        return ""
    if value in ("[]",):
        return []
    if (value.startswith('"') and value.endswith('"')) or (value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    lower = value.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    try:
        return int(value)
    except ValueError:
        return value


def _deep_update(target: dict[str, Any], updates: dict[str, Any]) -> None:
    for key, value in updates.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = value


def _validate_config(config: dict[str, Any]) -> None:
    video = config["video"]
    runtime = config["runtime"]

    if video["transport"] not in ("srt", "rtsp"):
        raise ValueError("video.transport must be 'srt' or 'rtsp'")
    if video["codec"] not in ("h264", "h265", "hevc"):
        raise ValueError("video.codec must be 'h264', 'h265', or 'hevc'")
    if int(video["width"]) <= 0 or int(video["height"]) <= 0 or int(video["fps"]) <= 0:
        raise ValueError("video.width, video.height, and video.fps must be positive")
    if int(runtime["max_restart_count"]) < 0:
        raise ValueError("runtime.max_restart_count cannot be negative")

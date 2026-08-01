from pathlib import Path

import pytest

from rdk_video_push.config import DEFAULT_CONFIG, _read_simple_yaml, load_config


def test_load_config_merges_values_with_defaults(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        """
video:
  width: 640
  transport: "srt"
runtime:
  auto_restart: false
""".strip(),
        encoding="utf-8",
    )

    config = load_config(config_path)

    assert config["video"]["width"] == 640
    assert config["video"]["height"] == DEFAULT_CONFIG["video"]["height"]
    assert config["runtime"]["auto_restart"] is False


def test_load_config_rejects_invalid_transport(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("video:\n  transport: udp\n", encoding="utf-8")

    with pytest.raises(ValueError, match="video.transport"):
        load_config(config_path)


def test_load_config_rejects_non_positive_dimensions(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text("video:\n  width: 0\n", encoding="utf-8")

    with pytest.raises(ValueError, match="must be positive"):
        load_config(config_path)


def test_simple_yaml_fallback_parses_supported_scalars(tmp_path: Path) -> None:
    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        """
video:
  width: 1920
  encoder_name: "h264_v4l2m2m"
  extra_input_args: []
runtime:
  auto_restart: true
""".strip(),
        encoding="utf-8",
    )

    config = _read_simple_yaml(config_path)

    assert config["video"]["width"] == 1920
    assert config["video"]["encoder_name"] == "h264_v4l2m2m"
    assert config["video"]["extra_input_args"] == []
    assert config["runtime"]["auto_restart"] is True


def test_load_config_requires_existing_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_config(tmp_path / "missing.yaml")

from copy import deepcopy

import pytest

from rdk_video_push.config import DEFAULT_CONFIG
from rdk_video_push.video_push import VideoPusher, _as_list


def test_build_command_uses_default_software_encoder() -> None:
    config = deepcopy(DEFAULT_CONFIG)

    command = VideoPusher(config).build_command()

    assert command[0] == "ffmpeg"
    assert command[command.index("-c:v") + 1] == "libx264"
    assert command[command.index("-video_size") + 1] == "1280x720"
    assert command[command.index("-framerate") + 1] == "15"
    assert command[-1] == config["video"]["srt_url"]


def test_build_command_supports_explicit_encoder_and_extra_args() -> None:
    config = deepcopy(DEFAULT_CONFIG)
    config["video"]["encoder_name"] = "h264_v4l2m2m"
    config["video"]["extra_input_args"] = "-thread_queue_size 4"
    config["video"]["extra_output_args"] = ["-flush_packets", "1"]

    command = VideoPusher(config).build_command()

    assert command[command.index("-c:v") + 1] == "h264_v4l2m2m"
    assert command[command.index("-thread_queue_size") + 1] == "4"
    assert command[command.index("-flush_packets") + 1] == "1"


def test_build_command_requires_rtsp_url() -> None:
    config = deepcopy(DEFAULT_CONFIG)
    config["video"]["transport"] = "rtsp"

    with pytest.raises(ValueError, match="video.rtsp_url"):
        VideoPusher(config).build_command()


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (None, []),
        (["-an", "-sn"], ["-an", "-sn"]),
        ("-preset veryfast", ["-preset", "veryfast"]),
        (3, ["3"]),
    ],
)
def test_as_list_normalizes_supported_values(value: object, expected: list[str]) -> None:
    assert _as_list(value) == expected

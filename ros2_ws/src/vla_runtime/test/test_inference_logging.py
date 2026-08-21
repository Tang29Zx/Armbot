import json

import pytest

from vla_runtime.inference_logging import InferenceJsonlLogger


def test_inference_logger_writes_jsonl_with_session_metadata(tmp_path):
    path = tmp_path / "inference.jsonl"
    logger = InferenceJsonlLogger(path, max_bytes=4096, backup_count=2)

    logger.write({"event": "inference_result", "prompt": "抓取药盒"})
    session_id = logger.session_id
    logger.close()

    record = json.loads(path.read_text(encoding="utf-8"))
    assert record["schema_version"] == 1
    assert record["session_id"] == session_id
    assert record["time_unix_ns"] > 0
    assert record["event"] == "inference_result"
    assert record["prompt"] == "抓取药盒"


def test_inference_logger_rotates_bounded_files(tmp_path):
    path = tmp_path / "inference.jsonl"
    logger = InferenceJsonlLogger(path, max_bytes=180, backup_count=2)

    for index in range(12):
        logger.write({"event": "test", "index": index, "payload": "x" * 80})
    logger.close()

    assert path.is_file()
    assert (tmp_path / "inference.jsonl.1").is_file()
    assert len(list(tmp_path.glob("inference.jsonl*"))) <= 3


@pytest.mark.parametrize(
    "max_bytes,backup_count",
    [(0, 1), (100, 0)],
)
def test_inference_logger_rejects_invalid_rotation(tmp_path, max_bytes, backup_count):
    with pytest.raises(ValueError):
        InferenceJsonlLogger(
            tmp_path / "inference.jsonl",
            max_bytes=max_bytes,
            backup_count=backup_count,
        )

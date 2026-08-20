"""Rotating structured logs for online policy inference."""

from __future__ import annotations

import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import time
import uuid


class InferenceJsonlLogger:
    """Write one machine-readable JSON object per line with bounded storage."""

    def __init__(self, path, max_bytes, backup_count):
        if not str(path).strip():
            raise ValueError("inference log path must not be empty")
        if int(max_bytes) <= 0:
            raise ValueError("inference log max_bytes must be positive")
        if int(backup_count) < 1:
            raise ValueError("inference log backup_count must be at least one")

        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.session_id = uuid.uuid4().hex
        self._logger = logging.Logger(
            "vla-inference-%s" % self.session_id, level=logging.INFO
        )
        self._logger.propagate = False
        self._handler = RotatingFileHandler(
            self.path,
            maxBytes=int(max_bytes),
            backupCount=int(backup_count),
            encoding="utf-8",
        )
        self._handler.setFormatter(logging.Formatter("%(message)s"))
        self._logger.addHandler(self._handler)

    def write(self, record):
        payload = {
            "schema_version": 1,
            "session_id": self.session_id,
            "time_unix_ns": time.time_ns(),
        }
        payload.update(record)
        line = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            separators=(",", ":"),
        )
        self._logger.info(line)

    def close(self):
        self._logger.removeHandler(self._handler)
        self._handler.close()

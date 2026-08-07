"""Small, dependency-light client for OpenPI's websocket policy server."""

from __future__ import annotations

import msgpack
import numpy as np


def _pack_array(value):
    if isinstance(value, (np.ndarray, np.generic)):
        if value.dtype.kind in ("V", "O", "c"):
            raise ValueError("unsupported NumPy dtype: %s" % value.dtype)
    if isinstance(value, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": value.tobytes(),
            b"dtype": value.dtype.str,
            b"shape": value.shape,
        }
    if isinstance(value, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": value.item(),
            b"dtype": value.dtype.str,
        }
    raise TypeError("cannot msgpack value of type %s" % type(value).__name__)


def _unpack_array(value):
    if b"__ndarray__" in value:
        return np.ndarray(
            buffer=value[b"data"],
            dtype=np.dtype(value[b"dtype"]),
            shape=value[b"shape"],
        ).copy()
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


def pack_observation(observation):
    return msgpack.packb(observation, default=_pack_array)


def unpack_response(payload):
    return msgpack.unpackb(payload, object_hook=_unpack_array)


class PolicyClient:
    """Synchronous OpenPI websocket client used only from the worker thread."""

    def __init__(self, host, port, timeout_sec=2.0):
        self._uri = "ws://%s:%d" % (host, int(port))
        self._timeout_sec = float(timeout_sec)
        self._connection = None

    @property
    def uri(self):
        return self._uri

    def close(self):
        connection, self._connection = self._connection, None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def _connect(self):
        from websockets.sync.client import connect

        self._connection = connect(
            self._uri,
            compression=None,
            max_size=None,
            open_timeout=self._timeout_sec,
        )
        # The first server message contains metadata. Reading it also proves
        # that the checkpoint has finished loading before control is enabled.
        metadata = self._connection.recv(timeout=self._timeout_sec)
        if isinstance(metadata, str):
            raise RuntimeError("policy server returned text metadata: %s" % metadata)
        unpack_response(metadata)

    def infer(self, observation):
        try:
            if self._connection is None:
                self._connect()
            self._connection.send(pack_observation(observation))
            payload = self._connection.recv(timeout=self._timeout_sec)
            if isinstance(payload, str):
                raise RuntimeError("policy server error: %s" % payload)
            return unpack_response(payload)
        except Exception:
            self.close()
            raise

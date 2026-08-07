import numpy as np
import pytest

from vla_runtime.policy_client import pack_observation, unpack_response


def test_msgpack_round_trip_preserves_numpy_arrays():
    value = {
        "image": np.arange(18, dtype=np.uint8).reshape(2, 3, 3),
        "state": np.arange(6, dtype=np.float32),
    }
    result = unpack_response(pack_observation(value))

    np.testing.assert_array_equal(result["image"], value["image"])
    np.testing.assert_array_equal(result["state"], value["state"])


def test_msgpack_rejects_object_arrays():
    with pytest.raises(ValueError, match="unsupported NumPy dtype"):
        pack_observation({"bad": np.asarray([object()], dtype=object)})

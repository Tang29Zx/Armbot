from io import BytesIO

import numpy as np
from PIL import Image

from vla_runtime.image_tools import decode_resize_with_pad


def test_decode_resize_with_pad_matches_training_shape():
    source = Image.new("RGB", (1280, 720), (10, 20, 30))
    payload = BytesIO()
    source.save(payload, format="JPEG")

    result = decode_resize_with_pad(payload.getvalue())

    assert result.shape == (224, 224, 3)
    assert result.dtype == np.uint8
    assert np.all(result[:40] == 0)
    assert np.all(result[80:140, 20:200] > 0)

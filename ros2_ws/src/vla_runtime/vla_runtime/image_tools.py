"""Image decoding that matches the offline LeRobot exporter."""

from io import BytesIO

import numpy as np
from PIL import Image


# Pillow added the Resampling enum in 9.1. Ubuntu 22.04 ships 9.0, where
# the interpolation constants still live directly on PIL.Image.
_RESAMPLING = getattr(Image, "Resampling", Image)


def decode_resize_with_pad(jpeg_bytes, size=224):
    if not jpeg_bytes:
        raise ValueError("compressed image is empty")
    with Image.open(BytesIO(jpeg_bytes)) as source:
        source.load()
        image = source.convert("RGB")
        ratio = max(image.width / size, image.height / size)
        width = max(1, int(image.width / ratio))
        height = max(1, int(image.height / ratio))
        image = image.resize((width, height), resample=_RESAMPLING.BILINEAR)
        result = Image.new("RGB", (size, size), 0)
        result.paste(image, ((size - width) // 2, (size - height) // 2))
        return np.asarray(result, dtype=np.uint8)

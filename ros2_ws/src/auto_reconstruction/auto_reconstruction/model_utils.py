"""
YOLO26 model utilities: loading, warm-up, and TensorRT export.

Used by both PC (training / export) and RDK X5 (inference).
"""

import time
from pathlib import Path

import numpy as np


def load_yolo_model(model_path: str, device: str = "cpu"):
    """Load a YOLO model from .pt or .engine file.

    Args:
        model_path: Path to weights file.
        device: 'cpu', 'cuda', or 'cuda:0'.

    Returns:
        ultralytics.YOLO model instance.
    """
    from ultralytics import YOLO

    model = YOLO(model_path)
    if device != "cpu":
        model.to(device)
    return model


def warmup_model(model, num_runs: int = 3) -> float:
    """Run dummy inferences to warm up GPU / JIT.

    Args:
        model: ultralytics.YOLO instance.
        num_runs: Number of warm-up forward passes.

    Returns:
        Average inference time in milliseconds.
    """
    dummy = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
    total_ms = 0.0
    for i in range(num_runs):
        t0 = time.perf_counter()
        _ = model.predict(dummy, imgsz=640, verbose=False)
        total_ms += (time.perf_counter() - t0) * 1000.0
    avg_ms = total_ms / num_runs
    return avg_ms


def export_to_tensorrt(
    model_path: str,
    output_path: str,
    imgsz: int = 640,
    workspace_gb: int = 4,
) -> str:
    """Export a YOLO .pt model to TensorRT .engine format.

    TensorRT engines are required for RDK X5 BPU-accelerated inference.

    Args:
        model_path: Path to source .pt weights.
        output_path: Destination path for .engine file.
        imgsz: Input image size.
        workspace_gb: GPU memory workspace for TensorRT (GB).

    Returns:
        Path to the exported .engine file.
    """
    from ultralytics import YOLO

    model = YOLO(model_path)
    exported = model.export(
        format="engine",
        imgsz=imgsz,
        workspace=workspace_gb,
        half=True,  # FP16 for speed
        device="cuda",
    )
    # Move to output path if different
    src = Path(exported) if isinstance(exported, str) else Path(str(exported))
    dst = Path(output_path)
    if src != dst:
        import shutil
        shutil.copy2(src, dst)
    return str(dst)


def load_calibrated_bbox(config_path: str) -> list:
    """Load calibrated bbox from a YAML config file.

    Expected format:
        calibrated_bbox:
          x1: 100
          y1: 200
          x2: 300
          y2: 400

    Returns:
        List [x1, y1, x2, y2] or None if not found.
    """
    import yaml

    try:
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
        bbox = cfg.get('calibrated_bbox', {})
        return [bbox['x1'], bbox['y1'], bbox['x2'], bbox['y2']]
    except (FileNotFoundError, KeyError, TypeError):
        return None

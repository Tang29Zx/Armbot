"""
YOLO26 medicine-box detector core.

Handles model loading, inference, and post-processing.
Works on both PC (PyTorch) and RDK X5 (TensorRT .engine).
"""

import time
from typing import Optional, Tuple

import numpy as np

try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False


class MedicineBoxDetector:
    """YOLO26-based detector for cuboid medicine boxes.

    Detects a single class ("medicine_box") in RGB images and compares
    detection position against a pre-calibrated reference bbox to compute
    pixel-space offsets for the VLA grasping pipeline.
    """

    #: Calibrated bounding box [x1, y1, x2, y2] in pixels.
    #: Set during setup phase by placing the medicine box at the fixed
    #: target position and running the calibration script.
    CALIBRATED_BBOX = None

    #: Maximum allowed center offset (pixels) before issuing a warning.
    OFFSET_WARN_THRESHOLD_PX = 50

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.5,
        target_class: str = "medicine_box",
        device: str = "cpu",
        calibrated_bbox: Optional[list] = None,
    ):
        """
        Args:
            model_path: Path to .pt (PyTorch) or .engine (TensorRT) weights.
            confidence_threshold: Minimum confidence for a valid detection.
            target_class: Class name to filter for.
            device: 'cpu', 'cuda', or 'cuda:0'.
            calibrated_bbox: Pre-calibrated reference bbox [x1,y1,x2,y2].
        """
        if not HAS_ULTRALYTICS:
            raise ImportError(
                "ultralytics is required. Install with: pip install ultralytics"
            )

        self.model_path = model_path
        self.conf_thresh = confidence_threshold
        self.target_class = target_class
        self.device = device
        self.CALIBRATED_BBOX = calibrated_bbox

        self._model: Optional[YOLO] = None

    @property
    def model(self):
        """Lazy-load the YOLO model on first access."""
        if self._model is None:
            self._model = YOLO(self.model_path)
            if self.device != "cpu":
                self._model.to(self.device)
        return self._model

    def detect(self, image: np.ndarray) -> dict:
        """Run detection on a single BGR/ RGB image.

        Args:
            image: numpy array of shape (H, W, 3), BGR or RGB.

        Returns:
            dict with keys:
                confirmed (bool)
                class_name (str)
                x1, y1, x2, y2 (float) — bbox in pixels
                offset_x, offset_y (float) — bbox-center offset from image center
                confidence (float)
                image_width, image_height (int)
        """
        h, w = image.shape[:2]

        results = self.model.predict(
            image, imgsz=640, conf=self.conf_thresh, verbose=False
        )

        # Default result: no detection
        output = {
            "confirmed": False,
            "class_name": "",
            "x1": 0.0, "y1": 0.0, "x2": 0.0, "y2": 0.0,
            "offset_x": 0.0, "offset_y": 0.0,
            "confidence": 0.0,
            "image_width": w, "image_height": h,
        }

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return output

        # Filter for target class and pick highest-confidence detection
        best_conf = -1.0
        best_box = None
        for box in boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names.get(cls_id, "")
            if cls_name == self.target_class and float(box.conf[0]) > best_conf:
                best_conf = float(box.conf[0])
                best_box = box

        if best_box is None or best_conf < self.conf_thresh:
            return output

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        # Pixel offset from image center
        offset_x = cx - w / 2.0
        offset_y = cy - h / 2.0

        output["confirmed"] = True
        output["class_name"] = self.target_class
        output["x1"], output["y1"] = float(x1), float(y1)
        output["x2"], output["y2"] = float(x2), float(y2)
        output["offset_x"] = float(offset_x)
        output["offset_y"] = float(offset_y)
        output["confidence"] = best_conf

        return output

    def verify_fixed_position(self, detection: dict) -> dict:
        """Compare detection against calibrated reference bbox.

        Adds a 'warning' field if the medicine box has shifted significantly
        from its calibrated position.

        Args:
            detection: Output dict from detect().

        Returns:
            Same dict with optional 'warning' key.
        """
        detection["warning"] = ""

        if self.CALIBRATED_BBOX is None:
            return detection

        if not detection["confirmed"]:
            detection["warning"] = "no_detection"
            return detection

        cal_cx = (self.CALIBRATED_BBOX[0] + self.CALIBRATED_BBOX[2]) / 2.0
        cal_cy = (self.CALIBRATED_BBOX[1] + self.CALIBRATED_BBOX[3]) / 2.0
        det_cx = (detection["x1"] + detection["x2"]) / 2.0
        det_cy = (detection["y1"] + detection["y2"]) / 2.0

        shift_x = det_cx - cal_cx
        shift_y = det_cy - cal_cy

        if abs(shift_x) > self.OFFSET_WARN_THRESHOLD_PX or \
           abs(shift_y) > self.OFFSET_WARN_THRESHOLD_PX:
            detection["warning"] = "large_shift"
            detection["offset_x"] = float(shift_x)
            detection["offset_y"] = float(shift_y)

        return detection

    @staticmethod
    def annotate_image(image: np.ndarray, detection: dict) -> np.ndarray:
        """Draw detection bbox and offsets on image for debugging.

        Args:
            image: BGR image (numpy array).
            detection: Output dict from detect().

        Returns:
            Annotated BGR image (copy).
        """
        import cv2

        annotated = image.copy()
        if not detection["confirmed"]:
            return annotated

        x1, y1 = int(detection["x1"]), int(detection["y1"])
        x2, y2 = int(detection["x2"]), int(detection["y2"])
        conf = detection["confidence"]

        # Draw bbox
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw label
        label = f'{detection["class_name"]} {conf:.2f}'
        cv2.putText(
            annotated, label, (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
        )

        # Draw center cross and offset info
        h, w = image.shape[:2]
        cx_img, cy_img = w // 2, h // 2
        cx_box = int((x1 + x2) / 2)
        cy_box = int((y1 + y2) / 2)
        cv2.drawMarker(
            annotated, (cx_img, cy_img), (255, 0, 0),
            cv2.MARKER_CROSS, 20, 1
        )
        cv2.line(annotated, (cx_img, cy_img), (cx_box, cy_box), (0, 0, 255), 1)

        off_text = f'offset: ({detection["offset_x"]:.0f}, {detection["offset_y"]:.0f}) px'
        cv2.putText(
            annotated, off_text, (10, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1
        )

        if detection.get("warning"):
            cv2.putText(
                annotated, f'WARN: {detection["warning"]}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
            )

        return annotated

#!/usr/bin/env python3
"""
Offline data augmentation for medicine-box real images.

Applies random transformations to each real image while correctly
adjusting YOLO-format bounding boxes.  Generates N augmented copies
per source image.

Usage:
    python scripts/augment_dataset.py \
        --image-dir data/real_boxes/images \
        --label-dir data/real_boxes/labels \
        --output-dir data/medicine_box \
        --aug-per-image 10
"""

import argparse
import os
import random
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: opencv-python is required.  pip install opencv-python")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline augmentation for real medicine-box images"
    )
    parser.add_argument("--image-dir", required=True)
    parser.add_argument("--label-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--aug-per-image", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def read_yolo_labels(label_path: str) -> list:
    """Read YOLO-format labels.

    Returns:
        List of [class_id, cx, cy, w, h] (normalised 0..1).
    """
    boxes = []
    if not os.path.exists(label_path):
        return boxes
    with open(label_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
                boxes.append([float(x) for x in parts[:5]])
    return boxes


def write_yolo_labels(label_path: str, boxes: list):
    """Write YOLO-format labels."""
    with open(label_path, "w") as f:
        for b in boxes:
            f.write(f"{int(b[0])} {b[1]:.6f} {b[2]:.6f} {b[3]:.6f} {b[4]:.6f}\n")


def flip_horizontal(img: np.ndarray, boxes: list):
    """Horizontal flip. x_center = 1 - x_center."""
    img = cv2.flip(img, 1)
    new_boxes = []
    for b in boxes:
        cls_id, cx, cy, w, h = b
        new_boxes.append([cls_id, 1.0 - cx, cy, w, h])
    return img, new_boxes


def rotate_image(img: np.ndarray, boxes: list, angle_deg: float):
    """Rotate image and adjust bounding boxes.

    Only works well for small angles (±20°).  Returns the rotated
    image and filtered boxes (boxes that are mostly visible after rotation).
    """
    h, w = img.shape[:2]
    center = (w // 2, h // 2)
    matrix = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

    # Rotate image
    rotated = cv2.warpAffine(
        img, matrix, (w, h),
        borderMode=cv2.BORDER_REFLECT101
    )

    new_boxes = []
    for b in boxes:
        cls_id, cx, cy, bw, bh = b
        # Convert to pixel coords (4 corners)
        px_cx, px_cy = cx * w, cy * h
        px_w, px_h = bw * w, bh * h
        x1, y1 = px_cx - px_w / 2, px_cy - px_h / 2
        x2, y2 = px_cx + px_w / 2, px_cy - px_h / 2
        x3, y3 = px_cx + px_w / 2, px_cy + px_h / 2
        x4, y4 = px_cx - px_w / 2, px_cy + px_h / 2

        # Rotate each corner
        corners = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
        ones = np.ones((4, 1))
        corners_h = np.hstack([corners, ones])
        rotated_corners = matrix.dot(corners_h.T).T

        # New axis-aligned bbox
        rx_min = max(0, rotated_corners[:, 0].min())
        ry_min = max(0, rotated_corners[:, 1].min())
        rx_max = min(w, rotated_corners[:, 0].max())
        ry_max = min(h, rotated_corners[:, 1].max())

        new_w = rx_max - rx_min
        new_h = ry_max - ry_min
        if new_w < 5 or new_h < 5:
            continue  # too small, skip

        new_cx = (rx_min + rx_max) / 2.0 / w
        new_cy = (ry_min + ry_max) / 2.0 / h
        new_bw = new_w / w
        new_bh = new_h / h

        # Clamp
        new_cx = max(0.0, min(1.0, new_cx))
        new_cy = max(0.0, min(1.0, new_cy))
        new_bw = max(0.0, min(1.0, new_bw))
        new_bh = max(0.0, min(1.0, new_bh))

        new_boxes.append([cls_id, new_cx, new_cy, new_bw, new_bh])

    return rotated, new_boxes


def adjust_brightness_contrast(img: np.ndarray, alpha: float, beta: float):
    """alpha=contrast, beta=brightness. Bboxes unchanged."""
    return cv2.convertScaleAbs(img, alpha=alpha, beta=beta)


def add_gaussian_noise(img: np.ndarray, sigma: float = 5.0):
    """Add Gaussian noise. Bboxes unchanged."""
    noise = np.random.normal(0, sigma, img.shape).astype(np.int16)
    noisy = img.astype(np.int16) + noise
    return np.clip(noisy, 0, 255).astype(np.uint8)


def perspective_warp(img: np.ndarray, boxes: list, scale: float = 0.03):
    """Apply slight perspective distortion and adjust boxes.

    scale: max fractional perturbation of each corner.
    """
    h, w = img.shape[:2]
    # Source corners
    src_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    # Perturb corners
    dst_pts = src_pts.copy()
    for i in range(4):
        dst_pts[i][0] += random.uniform(-scale, scale) * w
        dst_pts[i][1] += random.uniform(-scale, scale) * h

    matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(
        img, matrix, (w, h),
        borderMode=cv2.BORDER_REFLECT101
    )

    new_boxes = []
    for b in boxes:
        cls_id, cx, cy, bw, bh = b
        px_cx, px_cy = cx * w, cy * h
        px_w, px_h = bw * w, bh * h
        x1, y1 = px_cx - px_w / 2, px_cy - px_h / 2
        x2, y2 = px_cx + px_w / 2, px_cy - px_h / 2
        x3, y3 = px_cx + px_w / 2, px_cy + px_h / 2
        x4, y4 = px_cx - px_w / 2, px_cy + px_h / 2

        corners = np.float32([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
        corners = corners.reshape(-1, 1, 2)
        warped_corners = cv2.perspectiveTransform(corners, matrix).reshape(4, 2)

        wx_min = max(0, warped_corners[:, 0].min())
        wy_min = max(0, warped_corners[:, 1].min())
        wx_max = min(w, warped_corners[:, 0].max())
        wy_max = min(h, warped_corners[:, 1].max())

        new_w = wx_max - wx_min
        new_h = wy_max - wy_min
        if new_w < 5 or new_h < 5:
            continue

        new_boxes.append([
            cls_id,
            (wx_min + wx_max) / 2.0 / w,
            (wy_min + wy_max) / 2.0 / h,
            new_w / w,
            new_h / h,
        ])

    return warped, new_boxes


def augment_one(
    img: np.ndarray,
    boxes: list,
    aug_index: int,
) -> (np.ndarray, list):
    """Apply a random set of augmentations to one image.

    Returns (augmented_image, new_boxes).
    """
    h, w = img.shape[:2]

    # 1. Horizontal flip (50%)
    if random.random() < 0.5:
        img, boxes = flip_horizontal(img, boxes)

    # 2. Rotation (±15°)
    if random.random() < 0.6:
        angle = random.uniform(-15, 15)
        img, boxes = rotate_image(img, boxes, angle)

    # 3. Brightness (probability 0.7)
    if random.random() < 0.7:
        alpha = random.uniform(0.7, 1.3)  # contrast
        beta = random.randint(-30, 30)     # brightness shift
        img = adjust_brightness_contrast(img, alpha, beta)

    # 4. Gaussian noise (probability 0.4)
    if random.random() < 0.4:
        img = add_gaussian_noise(img, sigma=random.uniform(2, 8))

    # 5. Perspective warp (probability 0.5)
    if random.random() < 0.5:
        img, boxes = perspective_warp(img, boxes, scale=random.uniform(0.02, 0.05))

    # 6. Slight blur (probability 0.2)
    if random.random() < 0.2:
        img = cv2.GaussianBlur(img, (3, 3), 0)

    return img, boxes


def main():
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)

    image_dir = Path(args.image_dir)
    label_dir = Path(args.label_dir)
    out_img_dir = Path(args.output_dir) / "images" / "train"
    out_label_dir = Path(args.output_dir) / "labels" / "train"
    out_img_dir.mkdir(parents=True, exist_ok=True)
    out_label_dir.mkdir(parents=True, exist_ok=True)

    image_files = sorted(image_dir.glob("*.jpg"))
    print(f"Found {len(image_files)} source images")

    total = 0
    for img_path in image_files:
        label_path = label_dir / f"{img_path.stem}.txt"
        if not label_path.exists():
            print(f"  SKIP {img_path.name}: no label")
            continue

        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  SKIP {img_path.name}: cannot read")
            continue

        boxes = read_yolo_labels(str(label_path))
        if not boxes:
            print(f"  SKIP {img_path.name}: empty labels")
            continue

        base = img_path.stem

        for i in range(args.aug_per_image):
            aug_img, aug_boxes = augment_one(img.copy(), [b[:] for b in boxes], i)

            if not aug_boxes:
                continue  # all boxes lost

            name = f"{base}_aug{i:02d}"
            cv2.imwrite(str(out_img_dir / f"{name}.jpg"), aug_img)
            write_yolo_labels(str(out_label_dir / f"{name}.txt"), aug_boxes)
            total += 1

        print(f"  {img_path.name}: {args.aug_per_image} generated")

    print(f"\nTotal augmented images: {total}")


if __name__ == "__main__":
    main()

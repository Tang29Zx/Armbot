#!/usr/bin/env python3
"""
Generate synthetic cuboid medicine-box training images.

Creates realistic-looking rectangular box images with varying:
  - Box colors (white, blue, green, red, yellow, etc.)
  - Orientations and 3D perspectives
  - Lighting conditions (bright, dim, shadow)
  - Backgrounds (gradients, textures, solid)
  - Positions within frame

Output:
  data/medicine_box/images/train/   — training images
  data/medicine_box/images/val/     — validation images
  data/medicine_box/labels/train/   — YOLO-format labels
  data/medicine_box/labels/val/     — YOLO-format labels
  data/medicine_box/data.yaml       — dataset config

Usage:
  python scripts/generate_synthetic_data.py --num-train 400 --num-val 100
"""

import argparse
import os
import random
import sys
from pathlib import Path

import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFilter, ImageEnhance
except ImportError:
    print("ERROR: Pillow is required.  pip install Pillow")
    sys.exit(1)

# Pre-defined medicine-box colors (R, G, B)
BOX_COLORS = [
    (255, 255, 255),  # white
    (240, 248, 255),  # alice blue
    (220, 230, 240),  # light blue-grey
    (200, 220, 200),  # light green
    (250, 240, 230),  # warm white
    (230, 230, 210),  # cream
    (180, 200, 220),  # blue-grey
    (210, 180, 160),  # beige
    (190, 210, 200),  # mint
    (240, 220, 200),  # light brown
    (220, 210, 180),  # tan
    (250, 235, 215),  # antique white
    (255, 250, 240),  # floral white
    (230, 240, 250),  # lavender tint
    (245, 245, 220),  # light yellow
]

BOX_TEXT_COLORS = [
    (50, 50, 50),
    (30, 70, 30),
    (70, 30, 30),
    (30, 30, 70),
    (80, 20, 20),
    (20, 20, 80),
]


def random_box_color():
    return random.choice(BOX_COLORS)


def random_background_color():
    # Slightly varied background
    base = random.randint(180, 255)
    r = base + random.randint(-20, 20)
    g = base + random.randint(-20, 20)
    b = base + random.randint(-20, 20)
    return (
        max(0, min(255, r)),
        max(0, min(255, g)),
        max(0, min(255, b)),
    )


def draw_cuboid_box(
    draw: ImageDraw.ImageDraw,
    x1, y1, x2, y2,
    color,
    border_color=None,
    perspective_offset=0,
):
    """Draw a cuboid medicine box with 3D perspective.

    The box has:
      - Front face (rectangle)
      - Top face (parallelogram, perspective)
      - Side face (parallelogram, perspective)
      - Text/label area on front face
      - Border lines

    Args:
        draw: PIL ImageDraw object.
        x1, y1, x2, y2: Bounding box in pixels.
        color: (R, G, B) base color.
        border_color: (R, G, B) border color or None for auto.
        perspective_offset: Pixels of 3D depth effect.
    """
    if border_color is None:
        border_color = tuple(max(0, c - 60) for c in color)

    w = x2 - x1
    h = y2 - y1

    if perspective_offset <= 0:
        perspective_offset = max(4, int(min(w, h) * 0.12))

    # === Front face ===
    fx1, fy1 = x1, y1 + perspective_offset
    fx2, fy2 = x2, y2 + perspective_offset

    # Fill front face
    draw.rectangle([fx1, fy1, fx2, fy2], fill=color)

    # Front face border
    draw.rectangle([fx1, fy1, fx2, fy2], outline=border_color, width=2)

    # === Top face (parallelogram) ===
    top_color = tuple(min(255, c + 40) for c in color)  # lighter top
    tx1, ty1 = x1, y1
    tx2, ty2 = x2, y1
    tx3, ty3 = fx2, fy1
    tx4, ty4 = fx1, fy1
    draw.polygon([tx1, ty1, tx2, ty2, tx3, ty3, tx4, ty4], fill=top_color)
    # Top border
    draw.line([tx1, ty1, tx2, ty2], fill=border_color, width=2)

    # === Side face (parallelogram) ===
    side_color = tuple(max(0, c - 50) for c in color)  # darker side
    sx1, sy1 = fx2, fy1
    sx2, sy2 = fx2, fy2
    sx3, sy3 = x2 + perspective_offset, y2
    sx4, sy4 = x2 + perspective_offset, y1
    draw.polygon([sx1, sy1, sx2, sy2, sx3, sy3, sx4, sy4], fill=side_color)

    # === Text label on front face ===
    text_cx = (fx1 + fx2) // 2
    text_cy = (fy1 + fy2) // 2
    text_color = random.choice(BOX_TEXT_COLORS)

    # Drug name (generic)
    drug_names = [
        "PAROL", "MAJEZIK", "ARVELES", "AFERIN",
        "ASPIRIN", "DOLOREX", "UNISOM", "PANADOL",
        "BETADINE", "VITAMIN", "IBUPHIL", "XYZALL",
        "药 品", "MEDICINE", "TABLET", "CAPSULE",
    ]
    name = random.choice(drug_names)

    # Dosage info
    dosages = [
        "500mg", "100mg", "250mg", "600mg",
        "20 tablets", "30 caps", "10 tablets",
        "50mg/5ml", "200mg", "1000mg",
    ]
    dosage = random.choice(dosages)

    # Draw text
    try:
        font_size = max(10, min(w, h) // 8)
        # Pillow default font
        draw.text(
            (text_cx - font_size * 2, text_cy - font_size),
            name,
            fill=text_color,
        )
        draw.text(
            (text_cx - font_size * 2, text_cy + font_size // 2),
            dosage,
            fill=text_color,
        )
    except Exception:
        pass  # font rendering optional

    # === Barcode stripe ===
    stripe_y = fy2 - max(6, h // 8)
    stripe_color = tuple(max(0, c - 30) for c in color)
    draw.rectangle([fx1 + 5, stripe_y, fx2 - 5, fy2 - 2], fill=stripe_color)

    # Return the actual bounding box (image-space, including 3D extents)
    return [x1, y1, x2 + perspective_offset, y2 + perspective_offset]


def generate_image(
    img_width=640,
    img_height=640,
    box_color=None,
):
    """Generate one synthetic image with a cuboid medicine box.

    Returns:
        (image: PIL.Image, bbox_yolo: [x_center, y_center, width, height])
    """
    bg_color = random_background_color()
    img = Image.new("RGB", (img_width, img_height), bg_color)

    # Add background noise/texture
    pixels = img.load()
    for _ in range(img_width * img_height // 20):
        px = random.randint(0, img_width - 1)
        py = random.randint(0, img_height - 1)
        noise = random.randint(-8, 8)
        r, g, b = pixels[px, py]
        pixels[px, py] = (
            max(0, min(255, r + noise)),
            max(0, min(255, g + noise)),
            max(0, min(255, b + noise)),
        )

    # Draw desk/table surface
    table_y = random.randint(img_height * 2 // 3, img_height - 10)
    table_color = random_background_color()
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, table_y, img_width, img_height], fill=table_color)

    # Box parameters
    if box_color is None:
        box_color = random_box_color()

    # Random box size (cuboid proportions)
    box_w = random.randint(120, 300)
    box_h = random.randint(80, 200)

    # Random position
    x1 = random.randint(20, img_width - box_w - 80)
    y1 = random.randint(20, table_y - box_h - 10)

    x2 = x1 + box_w
    y2 = y1 + box_h

    perspective = random.randint(8, max(10, box_w // 5))

    # Draw the cuboid
    actual = draw_cuboid_box(
        draw, x1, y1, x2, y2,
        color=box_color,
        perspective_offset=perspective,
    )

    ax1, ay1, ax2, ay2 = actual

    # Ensure within bounds
    ax1 = max(0, ax1)
    ay1 = max(0, ay1)
    ax2 = min(img_width, ax2)
    ay2 = min(img_height, ay2)

    # === Post-processing ===
    # Random rotation (slight)
    if random.random() < 0.3:
        angle = random.uniform(-8, 8)
        img = img.rotate(angle, expand=False, fillcolor=bg_color)

    # Brightness variation
    if random.random() < 0.5:
        factor = random.uniform(0.7, 1.3)
        img = ImageEnhance.Brightness(img).enhance(factor)

    # Slight blur (simulate motion/camera)
    if random.random() < 0.2:
        img = img.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.5, 1.5)))

    # YOLO format: class_id x_center y_center width height (normalized)
    cx = (ax1 + ax2) / 2.0 / img_width
    cy = (ay1 + ay2) / 2.0 / img_height
    bw = (ax2 - ax1) / img_width
    bh = (ay2 - ay1) / img_height

    # Clamp to [0, 1]
    cx = max(0.0, min(1.0, cx))
    cy = max(0.0, min(1.0, cy))
    bw = max(0.0, min(1.0, bw))
    bh = max(0.0, min(1.0, bh))

    return img, (0, cx, cy, bw, bh)


def main():
    parser = argparse.ArgumentParser(
        description="Generate synthetic cuboid medicine-box images"
    )
    parser.add_argument("--num-train", type=int, default=400)
    parser.add_argument("--num-val", type=int, default=100)
    parser.add_argument("--img-size", type=int, default=640)
    parser.add_argument("--out-dir", type=str, default="data/medicine_box")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    out = Path(args.out_dir)
    for sub in ["images/train", "images/val", "labels/train", "labels/val"]:
        (out / sub).mkdir(parents=True, exist_ok=True)

    print(f"Generating {args.num_train} train + {args.num_val} val images...")

    for split, count in [("train", args.num_train), ("val", args.num_val)]:
        for i in range(count):
            img, (cls_id, cx, cy, bw, bh) = generate_image(
                img_width=args.img_size,
                img_height=args.img_size,
            )

            name = f"medbox_{split}_{i:04d}"
            img_path = out / "images" / split / f"{name}.jpg"
            img.save(img_path, quality=92)

            label_path = out / "labels" / split / f"{name}.txt"
            with open(label_path, "w") as f:
                f.write(f"{cls_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n")

            if (i + 1) % 100 == 0:
                print(f"  {split}: {i+1}/{count}")

    # Write data.yaml
    yaml_content = f"""# Medicine box dataset (synthetic + real)
path: {out.resolve()}
train: images/train
val: images/val
nc: 1
names: ['medicine_box']
"""
    with open(out / "data.yaml", "w") as f:
        f.write(yaml_content)

    print(f"\nDone! Generated {count} images to {out.resolve()}")
    print(f"  data.yaml written to {out / 'data.yaml'}")


if __name__ == "__main__":
    main()

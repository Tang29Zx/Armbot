#!/usr/bin/env python3
"""
Train a YOLO model to detect cuboid medicine boxes.

Expected directory layout:
    data/medicine_box/
      images/
        train/          # training images
        val/            # validation images
      labels/
        train/          # YOLO-format labels
        val/
      data.yaml         # dataset config

Usage:
    python scripts/train_medicine_box.py \
        --data data/medicine_box/data.yaml \
        --epochs 100 \
        --export-engine

Requirements:
    pip install ultralytics onnx onnxruntime-gpu  # for TensorRT export
"""

import argparse
import sys
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description='Train YOLO model for medicine-box detection'
    )
    parser.add_argument(
        '--data', type=str, default='data/medicine_box/data.yaml',
        help='Path to dataset YAML config'
    )
    parser.add_argument(
        '--model', type=str, default='yolo11n.pt',
        help='Pretrained model to start from (yolo11n.pt / yolo11s.pt / etc.)'
    )
    parser.add_argument(
        '--epochs', type=int, default=100,
        help='Number of training epochs'
    )
    parser.add_argument(
        '--imgsz', type=int, default=640,
        help='Training image size'
    )
    parser.add_argument(
        '--batch', type=int, default=16,
        help='Batch size (adjust for GPU memory)'
    )
    parser.add_argument(
        '--device', type=str, default='cuda',
        help="Device: 'cuda' (GPU) or 'cpu'"
    )
    parser.add_argument(
        '--name', type=str, default='medicine_box_detector',
        help='Run name for logging / output directory'
    )
    parser.add_argument(
        '--export-engine', action='store_true',
        help='Export trained model to TensorRT .engine after training'
    )
    parser.add_argument(
        '--export-onnx', action='store_true',
        help='Export trained model to ONNX format'
    )
    parser.add_argument(
        '--lr', type=float, default=1e-3,
        help='Initial learning rate'
    )
    return parser.parse_args()


def main():
    args = parse_args()

    # Verify data exists
    if not Path(args.data).exists():
        print(f'ERROR: Data config not found: {args.data}')
        print('Create data/medicine_box/ with the following structure:')
        print('  data/medicine_box/')
        print('    data.yaml')
        print('    images/train/')
        print('    images/val/')
        print('    labels/train/')
        print('    labels/val/')
        sys.exit(1)

    # Import here so --help works even without ultralytics installed
    try:
        from ultralytics import YOLO
    except ImportError:
        print('ERROR: ultralytics not installed.')
        print('  pip install ultralytics')
        sys.exit(1)

    print(f'Loading pretrained model: {args.model}')
    model = YOLO(args.model)

    print(f'Starting training: {args.epochs} epochs on {args.device}')
    print(f'  Data:    {args.data}')
    print(f'  Img size: {args.imgsz}')
    print(f'  Batch:   {args.batch}')
    print(f'  LR:      {args.lr}')

    # --- Train ---
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        name=args.name,
        lr0=args.lr,
        # Data augmentation (reasonable defaults for 300-500 image dataset)
        hsv_h=0.015,    # hue
        hsv_s=0.7,      # saturation
        hsv_v=0.4,      # value (brightness)
        degrees=10.0,   # rotation
        translate=0.1,  # translation
        scale=0.5,      # scaling
        fliplr=0.5,     # horizontal flip
        # Early stopping
        patience=20,
    )

    # Print metrics
    print('\n=== Training complete ===')
    print(f"Best model saved to: {results.save_dir}")

    # --- Export ---
    best_pt = Path(results.save_dir) / 'weights' / 'best.pt'

    if args.export_onnx:
        print('\nExporting to ONNX...')
        onnx_model = YOLO(str(best_pt))
        onnx_model.export(format='onnx', imgsz=args.imgsz, opset=12)
        print('ONNX export complete.')

    if args.export_engine:
        print('\nExporting to TensorRT (.engine)...')
        try:
            engine_model = YOLO(str(best_pt))
            engine_model.export(
                format='engine',
                imgsz=args.imgsz,
                half=True,
                workspace=4,
                device='cuda',
            )
            print('TensorRT export complete.')
        except Exception as e:
            print(f'TensorRT export failed: {e}')
            print('You can export manually on the target machine with:')
            print(f'  yolo export model={best_pt} format=engine imgsz={args.imgsz}')


if __name__ == '__main__':
    main()

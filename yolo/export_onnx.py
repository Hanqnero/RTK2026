"""Export a fine-tuned YOLO .pt checkpoint to ONNX for RPi 5 deployment.

Usage:
    python export_onnx.py --weights yolo26_finetuned.pt --imgsz 320 --output model.onnx
"""

import argparse
from pathlib import Path


try:
    from ultralytics import YOLO
except ImportError as exc:
    raise SystemExit(
        "ultralytics is not installed. Install it with: pip install ultralytics"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export YOLO .pt to ONNX for ONNX Runtime deployment on RPi 5."
    )
    parser.add_argument(
        "--weights",
        default="yolo26_finetuned.pt",
        help="Path to fine-tuned .pt checkpoint.",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=320,
        help="Inference image size (320 recommended for RPi 5 real-time).",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output .onnx path. Defaults to <weights stem>.onnx in same directory.",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=12,
        help="ONNX opset version (12 is broadly compatible with onnxruntime on ARM).",
    )
    parser.add_argument(
        "--simplify",
        action="store_true",
        default=True,
        help="Run onnx-simplifier after export (reduces graph size).",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        default=False,
        help="Export FP16 weights (only useful if running on GPU; keep False for RPi 5).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    weights_path = Path(args.weights)
    if not weights_path.exists():
        raise FileNotFoundError(f"Weights not found: {weights_path}")

    output_path = Path(args.output) if args.output else weights_path.with_suffix(".onnx")

    model = YOLO(str(weights_path))
    exported = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        simplify=args.simplify,
        half=args.half,
        dynamic=False,
    )

    exported_path = Path(exported)
    if exported_path.resolve() != output_path.resolve():
        output_path.parent.mkdir(parents=True, exist_ok=True)
        exported_path.rename(output_path)

    print(f"Exported: {output_path}  (imgsz={args.imgsz}, opset={args.opset})")
    print()
    print("Deploy to RPi 5:")
    print(f"  scp {output_path} pi@<rpi-ip>:~/models/")
    print()
    print("Run sign_detector node:")
    print(
        f"  ros2 run rtk2026_peripherals sign_detector "
        f"--ros-args -p model_path:=~/models/{output_path.name} "
        f"-p imgsz:={args.imgsz}"
    )


if __name__ == "__main__":
    main()

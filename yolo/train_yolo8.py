"""Fine-tune YOLOv8n on the RTK sign-detection dataset.

Preprocessing applied before training:
  - CLAHE (contrast-limited adaptive histogram equalisation) on the L channel
    of each image to normalise lighting across RealSense indoor/outdoor frames.
  - Preprocessed copies are written to <dataset_root>/../<split>_clahe/images/
    and a patched data.yaml is used for the training run.

Usage:
    python finetune_yolo26.py --data dataset/data.yaml
    python finetune_yolo26.py --data dataset/data.yaml --skip-preprocess
"""

import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml

try:
    from ultralytics import YOLO
except ImportError as exc:
    raise SystemExit(
        "ultralytics is not installed. Install it with: pip install ultralytics"
    ) from exc

try:
    import torch
except ImportError:
    torch = None

# Augmentation overrides tuned for RTK sign detection.
# Horizontal flip is OFF: turn_left and turn_right are separate classes and
# flipping them would produce incorrect labels.
SIGN_AUGMENTATION = dict(
    fliplr=0.0,  # no horizontal flip — turn_left/turn_right are separate classes
    flipud=0.0,  # no vertical flip
    degrees=5.0,  # small rotation (signs are roughly upright)
    translate=0.1,
    scale=0.4,
    shear=2.0,
    perspective=0.0,
    mosaic=1.0,
    close_mosaic=30,  # disable mosaic in last 30 epochs for stable convergence
    mixup=0.0,
    copy_paste=0.1,  # paste sign instances onto other backgrounds — multiplies tiny dataset
    hsv_h=0.015,
    hsv_s=0.5,
    hsv_v=0.3,
)


def resolve_path(path_str: str, root: Path) -> Path:
    p = Path(path_str)
    return p if p.is_absolute() else (root / p).resolve()


def find_best_checkpoint(run_dir: Path) -> Path:
    best = run_dir / "weights" / "best.pt"
    if best.exists():
        return best
    last = run_dir / "weights" / "last.pt"
    if last.exists():
        return last
    raise FileNotFoundError(f"No best.pt or last.pt found in {run_dir / 'weights'}")


def apply_clahe(img: np.ndarray, clip_limit: float, tile_grid: int) -> np.ndarray:
    """Apply CLAHE to the L channel of a BGR image."""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile_grid, tile_grid))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def preprocess_split(
    src_images_dir: Path,
    dst_images_dir: Path,
    clip_limit: float,
    tile_grid: int,
) -> int:
    """Copy a split's images to dst, applying CLAHE. Returns number of images processed."""
    dst_images_dir.mkdir(parents=True, exist_ok=True)
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    count = 0
    for src in src_images_dir.iterdir():
        if src.suffix.lower() not in exts:
            continue
        img = cv2.imread(str(src))
        if img is None:
            continue
        processed = apply_clahe(img, clip_limit, tile_grid)
        cv2.imwrite(str(dst_images_dir / src.name), processed)
        count += 1
    return count


def _resolve_split_dir(yaml_dir: Path, rel_path: str) -> Path:
    """Resolve a split image dir from a data.yaml value.

    Roboflow exports use paths like "../train/images" even when train/ sits
    directly under the yaml's parent directory.  Strip leading ".." segments
    so the path is always resolved relative to yaml_dir.
    """
    parts = [p for p in Path(rel_path).parts if p != ".."]
    return (yaml_dir / Path(*parts)).resolve()


def build_clahe_dataset(data_yaml: Path, clip_limit: float, tile_grid: int) -> Path:
    """Build a CLAHE-preprocessed copy of the dataset, return path to new data.yaml."""
    with open(data_yaml) as f:
        cfg = yaml.safe_load(f)

    yaml_dir = data_yaml.parent
    clahe_root = yaml_dir.parent / (yaml_dir.name + "_clahe")
    clahe_root.mkdir(parents=True, exist_ok=True)

    new_cfg = {
        k: v for k, v in cfg.items() if k not in ("train", "val", "test", "path")
    }

    for split_key in ("train", "val", "test"):
        if split_key not in cfg:
            continue

        src_abs = _resolve_split_dir(yaml_dir, cfg[split_key])
        if not src_abs.exists():
            print(f"  Skipping {split_key}: directory not found ({src_abs})")
            continue

        # Mirror the same subdirectory structure under clahe_root
        try:
            rel = src_abs.relative_to(yaml_dir)
        except ValueError:
            rel = Path(src_abs.name)
        dst_abs = clahe_root / rel

        n = preprocess_split(src_abs, dst_abs, clip_limit, tile_grid)

        # Copy labels (swap "images" → "labels" in path)
        src_labels_dir = Path(str(src_abs).replace("/images", "/labels"))
        if src_labels_dir.exists():
            dst_labels_dir = Path(str(dst_abs).replace("/images", "/labels"))
            if dst_labels_dir.exists():
                shutil.rmtree(dst_labels_dir)
            shutil.copytree(src_labels_dir, dst_labels_dir)

        # Write path relative to clahe_root so Ultralytics can find it via path:
        new_cfg[split_key] = str(rel)
        print(f"  CLAHE {split_key}: {n} images → {dst_abs}")

    new_cfg["path"] = str(clahe_root)

    new_yaml = clahe_root / "data.yaml"
    with open(new_yaml, "w") as f:
        yaml.dump(new_cfg, f, default_flow_style=False, sort_keys=False)

    return new_yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fine-tune YOLOv8n on RTK sign dataset with CLAHE preprocessing."
    )
    parser.add_argument(
        "--data",
        default="dataset/data.yaml",
        help="Path to dataset YAML.",
    )
    parser.add_argument(
        "--model",
        default="yolov8n.pt",
        help="Starting weights (yolov8n.pt, yolov8s.pt, or path to custom .pt).",
    )
    parser.add_argument(
        "--epochs", type=int, default=300, help="Number of training epochs."
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Training image size.")
    parser.add_argument("--batch", type=int, default=4, help="Batch size.")
    parser.add_argument(
        "--device",
        default="auto",
        help="Device: auto, mps, cpu, 0,1,...",
    )
    parser.add_argument(
        "--project",
        default="runs/yolov8n_finetune",
        help="Training project directory.",
    )
    parser.add_argument(
        "--name",
        default="exp",
        help="Run name under project directory.",
    )
    parser.add_argument(
        "--output",
        default="yolov8n_finetuned.pt",
        help="Final exported .pt path.",
    )
    parser.add_argument(
        "--skip-preprocess",
        action="store_true",
        default=False,
        help="Skip CLAHE preprocessing and train on raw images.",
    )
    parser.add_argument(
        "--clahe-clip",
        type=float,
        default=2.0,
        help="CLAHE clip limit (higher = more contrast enhancement).",
    )
    parser.add_argument(
        "--clahe-tile",
        type=int,
        default=8,
        help="CLAHE tile grid size.",
    )
    return parser.parse_args()


def resolve_device(device_arg: str) -> str:
    if device_arg.lower() != "auto":
        return device_arg
    if (
        torch is not None
        and hasattr(torch.backends, "mps")
        and torch.backends.mps.is_available()
    ):
        return "mps"
    if torch is not None and torch.cuda.is_available():
        return "0"
    return "cpu"


def main() -> None:
    args = parse_args()
    device = resolve_device(args.device)

    script_root = Path(__file__).resolve().parent
    data_yaml = resolve_path(args.data, script_root)
    project_dir = resolve_path(args.project, script_root)
    output_path = resolve_path(args.output, script_root)

    if not data_yaml.exists():
        raise FileNotFoundError(f"Dataset YAML not found: {data_yaml}")

    if args.skip_preprocess:
        train_yaml = data_yaml
        print("Preprocessing skipped — using raw images.")
    else:
        print(f"Applying CLAHE (clip={args.clahe_clip}, tile={args.clahe_tile})...")
        train_yaml = build_clahe_dataset(data_yaml, args.clahe_clip, args.clahe_tile)
        print(f"Preprocessed dataset: {train_yaml}")

    model = YOLO(args.model)

    result = model.train(
        data=str(train_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        project=str(project_dir),
        name=args.name,
        exist_ok=True,
        # Optimiser — AdamW + low LR for fine-tuning pretrained weights
        optimizer="AdamW",
        lr0=0.01,
        lrf=0.01,
        warmup_epochs=3,
        cos_lr=True,
        # Regularisation — essential for <50 image datasets
        weight_decay=0.001,
        dropout=0.1,
        # freeze=10,  # freeze first 10 backbone layers, train neck+head only
        # Misc
        patience=100,
        cache=True,  # cache all images in RAM (fine for tiny datasets)
        save_period=50,
        **SIGN_AUGMENTATION,
    )

    run_dir = Path(result.save_dir)
    best_ckpt = find_best_checkpoint(run_dir)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_ckpt, output_path)

    print(f"\nTraining complete.")
    print(f"  Run dir:    {run_dir}")
    print(f"  Device:     {device}")
    print(f"  Checkpoint: {best_ckpt}")
    print(f"  Output:     {output_path}")
    print(f"\nNext: export to ONNX for RPi 5 deployment:")
    print(f"  python export_onnx.py --weights {output_path} --imgsz 320")


if __name__ == "__main__":
    main()

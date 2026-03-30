import argparse
import shutil
from pathlib import Path


try:
    from ultralytics import YOLO
except ImportError as exc:
    raise SystemExit(
        "ultralytics is not installed. Install it with: pip install ultralytics"
    ) from exc

try:
    import torch
except ImportError:  # pragma: no cover - ultralytics depends on torch in practice.
    torch = None


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fine-tune YOLO on the augmented dataset and export a final .pt checkpoint."
    )
    parser.add_argument(
        "--data",
        default="dataset_augmented/dataset.yaml",
        help="Path to dataset YAML.",
    )
    parser.add_argument(
        "--model",
        default="yolo26n.pt",
        help="Starting weights or model name (e.g., yolo26n.pt, yolo26s.pt, custom.pt).",
    )
    parser.add_argument(
        "--epochs", type=int, default=100, help="Number of training epochs."
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Training image size.")
    parser.add_argument("--batch", type=int, default=16, help="Batch size.")
    parser.add_argument(
        "--device",
        default="auto",
        help="Device: auto, mps, cpu, 0,1,... (auto prefers Apple Metal on macOS).",
    )
    parser.add_argument(
        "--project",
        default="runs/yolo26_finetune",
        help="Training project directory.",
    )
    parser.add_argument(
        "--name",
        default="exp",
        help="Run name under project directory.",
    )
    parser.add_argument(
        "--output",
        default="yolo26_finetuned.pt",
        help="Final exported .pt filename/path.",
    )
    return parser.parse_args()


def resolve_device(device_arg: str) -> str:
    """Resolve --device to a backend supported by the current machine."""
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

    model = YOLO(args.model)

    result = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        project=str(project_dir),
        name=args.name,
        exist_ok=True,
    )

    run_dir = Path(result.save_dir)
    best_ckpt = find_best_checkpoint(run_dir)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best_ckpt, output_path)

    print(f"Training complete. Run dir: {run_dir}")
    print(f"Training device: {device}")
    print(f"Selected checkpoint: {best_ckpt}")
    print(f"Exported checkpoint: {output_path}")


if __name__ == "__main__":
    main()

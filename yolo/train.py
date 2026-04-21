from __future__ import annotations

import argparse
import os
from pathlib import Path

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		description="Train YOLOv26 quickly with strong object-detection defaults"
	)
	parser.add_argument(
		"--data",
		type=str,
		default="dataset/data.yaml",
		help="Path to dataset YAML",
	)
	parser.add_argument(
		"--model",
		type=str,
		default="yolo26n.pt",
		help="Starting weights (use yolo26n/s/m/l/x depending on speed vs accuracy)",
	)
	parser.add_argument(
		"--imgsz",
		type=int,
		default=640,
		help="Input image size",
	)
	parser.add_argument(
		"--epochs",
		type=int,
		default=100,
		help="Training epochs",
	)
	parser.add_argument(
		"--batch",
		type=int,
		default=-1,
		help="Batch size (-1 for auto)",
	)
	parser.add_argument(
		"--device",
		type=str,
		default="0",
		help="Device (e.g. 0, 0,1 or cpu)",
	)
	parser.add_argument(
		"--workers",
		type=int,
		default=0 if os.name == "nt" else 8,
		help="Dataloader workers",
	)
	parser.add_argument(
		"--cache",
		type=str,
		default="disk" if os.name == "nt" else "ram",
		help="Dataset cache mode: ram, disk, or False",
	)
	parser.add_argument(
		"--project",
		type=str,
		default="runs/yolo26",
		help="Project directory for run outputs",
	)
	parser.add_argument(
		"--name",
		type=str,
		default="fast_best",
		help="Run name",
	)
	parser.add_argument(
		"--seed",
		type=int,
		default=42,
		help="Random seed",
	)
	parser.add_argument(
		"--resume",
		action="store_true",
		help="Resume previous interrupted run",
	)
	parser.add_argument(
		"--post-val",
		action="store_true",
		help="Run an additional standalone validation pass after training",
	)
	return parser.parse_args()


def main() -> None:
	args = parse_args()
	script_dir = Path(__file__).resolve().parent
	data_yaml = (script_dir / args.data).resolve()

	if not data_yaml.exists():
		raise FileNotFoundError(f"Dataset YAML not found: {data_yaml}")

	model = YOLO(args.model)

	model.train(
		data=str(data_yaml),
		epochs=args.epochs,
		imgsz=args.imgsz,
		batch=args.batch,
		device=args.device,
		workers=args.workers,
		project=args.project,
		name=args.name,
		seed=args.seed,
		pretrained=True,
		deterministic=True,
		cache=args.cache,
		optimizer="AdamW",
		lr0=0.003,
		lrf=0.01,
		momentum=0.937,
		weight_decay=0.0005,
		warmup_epochs=3.0,
		warmup_momentum=0.8,
		warmup_bias_lr=0.1,
		cos_lr=True,
		hsv_h=0.015,
		hsv_s=0.6,
		hsv_v=0.4,
		degrees=10.0,
		translate=0.1,
		scale=0.4,
		shear=2.0,
		perspective=0.0005,
		flipud=0.0,
		fliplr=0.5,
		mosaic=0.6,
		mixup=0.05,
		copy_paste=0.1,
		close_mosaic=10,
		label_smoothing=0.0,
		box=7.5,
		cls=0.4,
		dfl=1.5,
		nbs=64,
		amp=True,
		val=True,
		plots=True,
		save=True,
		save_period=10,
		patience=40,
		resume=args.resume,
	)

	if args.post_val:
		model.val(data=str(data_yaml), imgsz=args.imgsz, device=args.device)
	model.export(format="onnx", imgsz=args.imgsz, dynamic=True, simplify=True)


if __name__ == "__main__":
	main()

import os
import random
from pathlib import Path

import cv2
import numpy as np

# ======== CONFIG ========
DATASET_PATH = "./dataset"
OUTPUT_PATH = "dataset_augmented"

CLASSES = [
    "bus_stop",
    "move_forward",
    "no_turn_left",
    "no_turn_right",
    "obstacle",
    "parking_spot",
    "turn_left",
    "turn_right",
]

IMG_SIZE = 640
SYNTH_PER_IMAGE = 120
TRAIN_SPLIT = 0.8

MULTI_OBJECT = True
MAX_OBJECTS = 3

# ========================

IMAGE_EXTS = ("*.jpg", "*.jpeg", "*.png", "*.bmp")


def resolve_path(path_str):
    p = Path(path_str)
    if p.is_absolute():
        return p
    # Resolve relative to this script so execution cwd does not matter.
    return (Path(__file__).resolve().parent / p).resolve()


def collect_samples(dataset_root):
    """Collect (image_path, label_path) pairs from common YOLO directory layouts."""
    pairs = []

    # Flat layout: dataset/images + dataset/labels
    flat_img = dataset_root / "images"
    flat_lbl = dataset_root / "labels"
    if flat_img.exists() and flat_lbl.exists():
        for pattern in IMAGE_EXTS:
            for img_path in flat_img.glob(pattern):
                pairs.append((img_path, flat_lbl / (img_path.stem + ".txt")))

    # Split layout: dataset/train|val|test/images + .../labels
    for split in ("train", "val", "test"):
        img_dir = dataset_root / split / "images"
        lbl_dir = dataset_root / split / "labels"
        if not (img_dir.exists() and lbl_dir.exists()):
            continue
        for pattern in IMAGE_EXTS:
            for img_path in img_dir.glob(pattern):
                pairs.append((img_path, lbl_dir / (img_path.stem + ".txt")))

    return pairs


def load_labels(path, w, h):
    boxes = []
    if not path.exists():
        return boxes

    with open(path) as f:
        for line in f:
            cls, x, y, bw, bh = map(float, line.split())
            x1 = int((x - bw / 2) * w)
            y1 = int((y - bh / 2) * h)
            x2 = int((x + bw / 2) * w)
            y2 = int((y + bh / 2) * h)
            boxes.append((int(cls), x1, y1, x2, y2))
    return boxes


def crop_objects(img, boxes):
    out = []
    for cls, x1, y1, x2, y2 in boxes:
        crop = img[y1:y2, x1:x2]
        if crop.size > 0:
            out.append((cls, crop))
    return out


def augment(obj):
    scale = random.uniform(0.4, 1.4)
    obj = cv2.resize(obj, None, fx=scale, fy=scale)

    if random.random() > 0.5:
        obj = cv2.flip(obj, 1)

    alpha = random.uniform(0.7, 1.3)
    beta = random.randint(-25, 25)
    obj = cv2.convertScaleAbs(obj, alpha=alpha, beta=beta)

    if random.random() > 0.7:
        k = random.choice([3, 5])
        obj = cv2.GaussianBlur(obj, (k, k), 0)

    return obj


def paste(bg, obj):
    H, W = bg.shape[:2]
    h, w = obj.shape[:2]

    if h >= H or w >= W:
        return None

    x = random.randint(0, W - w)
    y = random.randint(0, H - h)

    # soft blending
    alpha = np.ones((h, w, 1), dtype=float) * random.uniform(0.7, 1.0)
    roi = bg[y : y + h, x : x + w]

    blended = (roi * (1 - alpha) + obj * alpha).astype(np.uint8)
    bg[y : y + h, x : x + w] = blended

    return x, y, w, h


def save_label(path, boxes, W, H):
    with open(path, "w") as f:
        for cls, x, y, w, h in boxes:
            cx = (x + w / 2) / W
            cy = (y + h / 2) / H
            nw = w / W
            nh = h / H
            f.write(f"{cls} {cx} {cy} {nw} {nh}\n")


def main():
    dataset_root = resolve_path(DATASET_PATH)
    output_root = resolve_path(OUTPUT_PATH)

    samples = collect_samples(dataset_root)
    if not samples:
        raise FileNotFoundError(
            f"No images found under '{dataset_root}'. Expected either "
            "dataset/images or dataset/<split>/images structure."
        )

    train_img = output_root / "images/train"
    val_img = output_root / "images/val"
    train_lbl = output_root / "labels/train"
    val_lbl = output_root / "labels/val"

    for p in [train_img, val_img, train_lbl, val_lbl]:
        p.mkdir(parents=True, exist_ok=True)

    random.shuffle(samples)

    split_idx = int(len(samples) * TRAIN_SPLIT)
    train_set = samples[:split_idx]
    val_set = samples[split_idx:]

    def process(sample_pairs, split):
        for img_path, label_path in sample_pairs:
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            h, w = img.shape[:2]

            boxes = load_labels(label_path, w, h)
            crops = crop_objects(img, boxes)

            if not crops:
                continue

            for i in range(SYNTH_PER_IMAGE):
                bg = np.full(
                    (IMG_SIZE, IMG_SIZE, 3), random.randint(0, 255), dtype=np.uint8
                )

                new_boxes = []
                n = random.randint(1, MAX_OBJECTS) if MULTI_OBJECT else 1

                for _ in range(n):
                    cls, obj = random.choice(crops)
                    obj = augment(obj)

                    result = paste(bg, obj)
                    if result:
                        x, y, ow, oh = result
                        new_boxes.append((cls, x, y, ow, oh))

                if not new_boxes:
                    continue

                name = f"{img_path.stem}_{i}.jpg"

                if split == "train":
                    cv2.imwrite(str(train_img / name), bg)
                    save_label(
                        train_lbl / name.replace(".jpg", ".txt"),
                        new_boxes,
                        IMG_SIZE,
                        IMG_SIZE,
                    )
                else:
                    cv2.imwrite(str(val_img / name), bg)
                    save_label(
                        val_lbl / name.replace(".jpg", ".txt"),
                        new_boxes,
                        IMG_SIZE,
                        IMG_SIZE,
                    )

    process(train_set, "train")
    process(val_set, "val")

    # dataset.yaml
    with open(output_root / "dataset.yaml", "w") as f:
        f.write(f"path: {output_root}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"nc: {len(CLASSES)}\n")
        f.write(f"names: {CLASSES}\n")


if __name__ == "__main__":
    main()

import random
import shutil
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import cv2
import numpy as np

# ======== CONFIG ========
DATASET_PATH = "./dataset"
OUTPUT_PATH = "dataset_augmented_street"
BACKGROUND_DIR = "./backgrounds_street"

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
NUM_BACKGROUNDS = 150
TRAIN_SPLIT = 0.8

# Free stock source (no API key required). Unsplash Source returns random images
# by query. Picsum is used as a fallback when Unsplash rate-limits.
UNSPLASH_QUERY = "street,road,city,avenue,urban,traffic"
UNSPLASH_SIZE = (1280, 960)
USE_PICSUM_FALLBACK = True
DOWNLOAD_RETRIES = 4
DOWNLOAD_TIMEOUT_SEC = 20

# ========================

IMAGE_EXTS = ("*.jpg", "*.jpeg", "*.png", "*.bmp")


def collect_images(image_dir):
    images = []
    for pattern in IMAGE_EXTS:
        images.extend(sorted(image_dir.glob(pattern)))
    return images


def resolve_path(path_str):
    p = Path(path_str)
    if p.is_absolute():
        return p
    # Resolve relative to this script so execution cwd does not matter.
    return (Path(__file__).resolve().parent / p).resolve()


def collect_samples(dataset_root):
    """Collect (image_path, label_path) pairs from common YOLO directory layouts."""
    pairs = []

    flat_img = dataset_root / "images"
    flat_lbl = dataset_root / "labels"
    if flat_img.exists() and flat_lbl.exists():
        for pattern in IMAGE_EXTS:
            for img_path in flat_img.glob(pattern):
                pairs.append((img_path, flat_lbl / (img_path.stem + ".txt")))

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


def download_street_backgrounds(background_dir, target_count):
    background_dir.mkdir(parents=True, exist_ok=True)
    existing = collect_images(background_dir)
    if len(existing) >= target_count:
        return existing[:target_count]

    width, height = UNSPLASH_SIZE
    query_enc = urllib.parse.quote_plus(UNSPLASH_QUERY)

    start_idx = len(existing)
    for idx in range(start_idx, target_count):
        unsplash_url = (
            f"https://source.unsplash.com/random/{width}x{height}"
            f"/?{query_enc}&sig={idx}"
        )
        picsum_url = f"https://picsum.photos/seed/street-{idx}/{width}/{height}"
        out_path = background_dir / f"street_bg_{idx:04d}.jpg"

        success = False
        candidate_urls = [unsplash_url]
        if USE_PICSUM_FALLBACK:
            candidate_urls.append(picsum_url)

        for url in candidate_urls:
            for attempt in range(1, DOWNLOAD_RETRIES + 1):
                try:
                    req = urllib.request.Request(
                        url, headers={"User-Agent": "Mozilla/5.0"}
                    )
                    with urllib.request.urlopen(req, timeout=DOWNLOAD_TIMEOUT_SEC) as r:
                        image_bytes = r.read()

                    out_path.write_bytes(image_bytes)
                    success = True
                    break
                except (urllib.error.URLError, TimeoutError, OSError):
                    if attempt == DOWNLOAD_RETRIES:
                        break
                    time.sleep(0.8 * attempt)

            if success:
                break

        if not success and out_path.exists():
            out_path.unlink(missing_ok=True)

    return collect_images(background_dir)[:target_count]


def fit_background_to_square(img, size):
    h, w = img.shape[:2]
    if h == 0 or w == 0:
        return None

    scale = max(size / w, size / h)
    new_w = max(size, int(w * scale))
    new_h = max(size, int(h * scale))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    x0 = 0 if new_w == size else random.randint(0, new_w - size)
    y0 = 0 if new_h == size else random.randint(0, new_h - size)
    return resized[y0 : y0 + size, x0 : x0 + size].copy()


def collect_sign_crops(samples):
    crops = []
    for img_path, label_path in samples:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        boxes = load_labels(label_path, w, h)
        crops.extend(crop_objects(img, boxes))
    return crops


def rotate_image(img, angle_deg):
    h, w = img.shape[:2]
    c_x, c_y = w // 2, h // 2
    m = cv2.getRotationMatrix2D((c_x, c_y), angle_deg, 1.0)

    cos = abs(m[0, 0])
    sin = abs(m[0, 1])
    n_w = int((h * sin) + (w * cos))
    n_h = int((h * cos) + (w * sin))

    m[0, 2] += (n_w / 2) - c_x
    m[1, 2] += (n_h / 2) - c_y
    return cv2.warpAffine(
        img,
        m,
        (n_w, n_h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101,
    )


def augment_sign(obj):
    scale = random.uniform(0.55, 1.25)
    obj = cv2.resize(obj, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    if random.random() > 0.55:
        obj = cv2.flip(obj, 1)

    obj = rotate_image(obj, random.uniform(-12.0, 12.0))

    alpha = random.uniform(0.85, 1.15)
    beta = random.randint(-15, 15)
    obj = cv2.convertScaleAbs(obj, alpha=alpha, beta=beta)

    if random.random() > 0.8:
        k = random.choice([3, 5])
        obj = cv2.GaussianBlur(obj, (k, k), 0)

    return obj


def paste(bg, obj):
    h_bg, w_bg = bg.shape[:2]
    h, w = obj.shape[:2]
    if h >= h_bg or w >= w_bg:
        return None

    x = random.randint(0, w_bg - w)
    y = random.randint(0, h_bg - h)

    alpha = np.ones((h, w, 1), dtype=float) * random.uniform(0.75, 1.0)
    roi = bg[y : y + h, x : x + w]
    blended = (roi * (1 - alpha) + obj * alpha).astype(np.uint8)
    bg[y : y + h, x : x + w] = blended

    return x, y, w, h


def save_label(path, boxes, w_img, h_img):
    with open(path, "w") as f:
        for cls, x, y, w, h in boxes:
            cx = (x + w / 2) / w_img
            cy = (y + h / 2) / h_img
            nw = w / w_img
            nh = h / h_img
            f.write(f"{cls} {cx} {cy} {nw} {nh}\n")


def verify_class_presence(label_dir):
    present = set()
    for txt in sorted(label_dir.glob("*.txt")):
        with open(txt) as f:
            for line in f:
                parts = line.strip().split()
                if parts:
                    present.add(int(float(parts[0])))
    return present


def main():
    dataset_root = resolve_path(DATASET_PATH)
    output_root = resolve_path(OUTPUT_PATH)
    background_root = resolve_path(BACKGROUND_DIR)

    samples = collect_samples(dataset_root)
    if not samples:
        raise FileNotFoundError(
            f"No images found under '{dataset_root}'. Expected either "
            "dataset/images or dataset/<split>/images structure."
        )

    sign_crops = collect_sign_crops(samples)
    if not sign_crops:
        raise RuntimeError("No sign crops found in dataset labels/images.")

    # Each sign image should be used in each split. If this fails, class IDs are missing.
    available_class_ids = sorted({cls for cls, _ in sign_crops})
    if len(available_class_ids) < len(CLASSES):
        raise RuntimeError(
            "Not all classes are present in sign crops. "
            f"Found classes: {available_class_ids}, expected 0..{len(CLASSES)-1}."
        )

    backgrounds = download_street_backgrounds(background_root, NUM_BACKGROUNDS)
    if len(backgrounds) < 2:
        raise RuntimeError("Need at least 2 background images to split into train/val.")

    random.shuffle(backgrounds)
    split_idx = max(1, min(len(backgrounds) - 1, int(len(backgrounds) * TRAIN_SPLIT)))
    train_bgs = backgrounds[:split_idx]
    val_bgs = backgrounds[split_idx:]

    # Overwrite current augmented dataset output.
    if output_root.exists():
        shutil.rmtree(output_root)

    train_img = output_root / "images/train"
    val_img = output_root / "images/val"
    train_lbl = output_root / "labels/train"
    val_lbl = output_root / "labels/val"
    for p in (train_img, val_img, train_lbl, val_lbl):
        p.mkdir(parents=True, exist_ok=True)

    def process(background_paths, out_img_dir, out_lbl_dir, split_name):
        out_index = 0
        for bg_path in background_paths:
            base_bg = cv2.imread(str(bg_path))
            if base_bg is None:
                continue

            for sign_idx, (cls, sign_crop) in enumerate(sign_crops):
                bg = fit_background_to_square(base_bg, IMG_SIZE)
                if bg is None:
                    continue

                obj = augment_sign(sign_crop)
                result = paste(bg, obj)
                if result is None:
                    continue

                x, y, ow, oh = result
                name = f"{split_name}_{bg_path.stem}_{sign_idx:03d}_{out_index:05d}.jpg"
                out_index += 1

                cv2.imwrite(str(out_img_dir / name), bg)
                save_label(
                    out_lbl_dir / name.replace(".jpg", ".txt"),
                    [(cls, x, y, ow, oh)],
                    IMG_SIZE,
                    IMG_SIZE,
                )

    process(train_bgs, train_img, train_lbl, "train")
    process(val_bgs, val_img, val_lbl, "val")

    with open(output_root / "dataset.yaml", "w") as f:
        f.write(f"path: {output_root}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"nc: {len(CLASSES)}\n")
        f.write(f"names: {CLASSES}\n")

    train_present = verify_class_presence(train_lbl)
    val_present = verify_class_presence(val_lbl)
    needed = set(range(len(CLASSES)))
    if train_present != needed or val_present != needed:
        raise RuntimeError(
            "Class presence check failed. "
            f"train_present={sorted(train_present)} val_present={sorted(val_present)}"
        )

    train_count = len(list(train_img.glob("*.jpg")))
    val_count = len(list(val_img.glob("*.jpg")))

    print(f"Sign crops collected: {len(sign_crops)}")
    print(f"Backgrounds used: {len(backgrounds)}")
    print(f"Train images: {train_count}")
    print(f"Val images: {val_count}")
    print(f"Augmented dataset written to: {output_root}")


if __name__ == "__main__":
    main()

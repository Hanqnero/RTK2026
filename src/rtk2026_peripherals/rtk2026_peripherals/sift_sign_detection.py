#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass
class ReferenceSign:
    sign_id: int
    class_name: str
    keypoints: list
    descriptors: np.ndarray
    w: int
    h: int


@dataclass
class DetectionResult:
    sign_id: int
    score: float
    polygon: np.ndarray


def create_sift() -> cv2.SIFT:
    if not hasattr(cv2, "SIFT_create"):
        raise RuntimeError(
            "SIFT is unavailable. Install opencv-contrib-python in the container."
        )
    return cv2.SIFT_create(
        nfeatures=0,
        nOctaveLayers=3,
        contrastThreshold=0.03,
        edgeThreshold=10.0,
        sigma=1.6,
    )


def preprocess_gray(img_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return clahe.apply(gray)


def read_dataset_names(data_yaml_path: Path) -> dict[int, str]:
    with data_yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    names = data.get("names", [])
    if isinstance(names, list):
        return {i: str(v) for i, v in enumerate(names)}
    if isinstance(names, dict):
        return {int(k): str(v) for k, v in names.items()}
    return {}


def yolo_to_xyxy(
    xc: float, yc: float, w: float, h: float, img_w: int, img_h: int
) -> tuple[int, int, int, int]:
    x1 = int((xc - w / 2.0) * img_w)
    y1 = int((yc - h / 2.0) * img_h)
    x2 = int((xc + w / 2.0) * img_w)
    y2 = int((yc + h / 2.0) * img_h)
    x1 = max(0, min(img_w - 1, x1))
    y1 = max(0, min(img_h - 1, y1))
    x2 = max(0, min(img_w - 1, x2))
    y2 = max(0, min(img_h - 1, y2))
    if x2 <= x1:
        x2 = min(img_w - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(img_h - 1, y1 + 1)
    return x1, y1, x2, y2


def class_name_to_sign_id(name: str) -> int:
    n = name.lower()
    if "left" in n or "лев" in n:
        return 2
    if "right" in n or "прав" in n:
        return 3
    if "inter" in n or "cross" in n or "перек" in n:
        return 1
    return 0


def build_reference_signs(
    dataset_root: Path, sift: cv2.SIFT, max_refs_per_class: int = 3
) -> list[ReferenceSign]:
    names = read_dataset_names(dataset_root / "data.yaml")
    images_dir = dataset_root / "train" / "images"
    labels_dir = dataset_root / "train" / "labels"
    if not images_dir.exists() or not labels_dir.exists():
        raise FileNotFoundError(
            f"Expected train/images and train/labels under {dataset_root}"
        )

    by_sign_id: dict[int, list[ReferenceSign]] = {}
    for label_path in sorted(labels_dir.glob("*.txt")):
        candidates = [
            images_dir / f"{label_path.stem}.jpg",
            images_dir / f"{label_path.stem}.png",
            images_dir / f"{label_path.stem}.jpeg",
        ]
        image_path = next((p for p in candidates if p.exists()), None)
        if image_path is None:
            continue
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        h, w = image.shape[:2]

        for line in label_path.read_text(encoding="utf-8").strip().splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            class_id = int(float(parts[0]))
            xc, yc, bw, bh = map(float, parts[1:5])
            x1, y1, x2, y2 = yolo_to_xyxy(xc, yc, bw, bh, w, h)
            roi = image[y1:y2, x1:x2]
            if roi.size == 0 or roi.shape[0] < 8 or roi.shape[1] < 8:
                continue
            md = min(roi.shape[:2])
            if md < 64:
                scale = 64.0 / md
                roi = cv2.resize(roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            roi_gray = preprocess_gray(roi)
            kps, des = sift.detectAndCompute(roi_gray, None)
            if des is None or len(kps) < 4:
                continue

            class_name = names.get(class_id, str(class_id))
            sign_id = class_name_to_sign_id(class_name)
            if sign_id == 0:
                continue
            ref = ReferenceSign(
                sign_id=sign_id,
                class_name=class_name,
                keypoints=kps,
                descriptors=des,
                w=roi_gray.shape[1],
                h=roi_gray.shape[0],
            )
            by_sign_id.setdefault(sign_id, []).append(ref)

    refs: list[ReferenceSign] = []
    for sign_id, items in by_sign_id.items():
        items.sort(key=lambda r: len(r.keypoints), reverse=True)
        refs.extend(items[:max_refs_per_class])
    return refs


def detect_signs(
    image_bgr: np.ndarray,
    references: list[ReferenceSign],
    ratio_thresh: float = 0.75,
    min_good: int = 6,
    min_inliers: int = 6,
    min_inlier_ratio: float = 0.3,
) -> list[DetectionResult]:
    if not references:
        return []

    sift = create_sift()
    matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)
    query_gray = preprocess_gray(image_bgr)
    q_kps, q_des = sift.detectAndCompute(query_gray, None)
    if q_des is None or len(q_kps) < 4:
        return []

    candidates: list[DetectionResult] = []
    for ref in references:
        knn = matcher.knnMatch(ref.descriptors, q_des, k=2)
        good = []
        for pair in knn:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < ratio_thresh * n.distance:
                good.append(m)
        if len(good) < min_good:
            continue

        src = np.float32([ref.keypoints[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([q_kps[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None or mask is None:
            continue
        inliers = int(mask.sum())
        if inliers < min_inliers:
            continue
        inlier_ratio = inliers / max(len(good), 1)
        if inlier_ratio < min_inlier_ratio:
            continue

        corners = np.float32(
            [[0, 0], [ref.w - 1, 0], [ref.w - 1, ref.h - 1], [0, ref.h - 1]]
        ).reshape(-1, 1, 2)
        poly = cv2.perspectiveTransform(corners, H)
        if not np.isfinite(poly).all():
            continue

        score = inliers * inlier_ratio + 5.0 * inlier_ratio
        candidates.append(DetectionResult(sign_id=ref.sign_id, score=score, polygon=poly))

    candidates.sort(key=lambda d: d.score, reverse=True)
    best_by_id: dict[int, DetectionResult] = {}
    for c in candidates:
        if c.sign_id not in best_by_id:
            best_by_id[c.sign_id] = c
    return list(best_by_id.values())


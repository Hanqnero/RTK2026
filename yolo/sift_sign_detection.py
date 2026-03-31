import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


@dataclass
class ReferenceSign:
    class_id: int
    class_name: str
    image_path: Path
    roi_gray: np.ndarray
    keypoints: list[cv2.KeyPoint]
    descriptors: np.ndarray
    width: int
    height: int


@dataclass
class DetectionResult:
    class_id: int
    class_name: str
    score: float
    good_matches: int
    inliers: int
    polygon: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Traffic sign detection using SIFT + homography with 8 references from dataset/."
    )
    parser.add_argument(
        "--dataset-root",
        type=str,
        default="dataset",
        help="Dataset root containing data.yaml and train/{images,labels}.",
    )
    parser.add_argument(
        "--input",
        type=str,
        required=True,
        help="Input image path for sign detection.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="Optional output image path with detection overlays.",
    )
    parser.add_argument(
        "--ratio-thresh",
        type=float,
        default=0.75,
        help="Lowe ratio test threshold (lower = stricter matching).",
    )
    parser.add_argument(
        "--min-good",
        type=int,
        default=6,
        help="Minimum good matches required before homography.",
    )
    parser.add_argument(
        "--min-inliers",
        type=int,
        default=6,
        help="Minimum RANSAC inliers required for positive detection.",
    )
    parser.add_argument(
        "--min-inlier-ratio",
        type=float,
        default=0.3,
        help="Minimum inlier/good-match ratio required for positive detection.",
    )
    parser.add_argument(
        "--max-refs-per-class",
        type=int,
        default=3,
        help="Maximum reference crops to keep per class (best by keypoint count).",
    )
    parser.add_argument(
        "--nms-thresh",
        type=float,
        default=0.4,
        help="IoU threshold for non-maximum suppression across detections.",
    )
    parser.add_argument(
        "--tune",
        action="store_true",
        help="Enable parameter tuning mode to find best detection parameters.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show result in an OpenCV window.",
    )
    return parser.parse_args()


def read_dataset_names(data_yaml_path: Path) -> dict[int, str]:
    with data_yaml_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    names = data.get("names", [])
    if isinstance(names, list):
        return {idx: str(name) for idx, name in enumerate(names)}
    if isinstance(names, dict):
        return {int(k): str(v) for k, v in names.items()}

    raise ValueError(f"Unexpected 'names' format in {data_yaml_path}")


def parse_label_file(label_path: Path) -> tuple[int, tuple[float, float, float, float]]:
    line = label_path.read_text(encoding="utf-8").strip().splitlines()[0]
    parts = line.split()
    if len(parts) < 5:
        raise ValueError(f"Invalid YOLO label format in {label_path}")

    class_id = int(float(parts[0]))
    xc, yc, w, h = map(float, parts[1:5])
    return class_id, (xc, yc, w, h)


def yolo_to_xyxy(
    box: tuple[float, float, float, float], image_w: int, image_h: int
) -> tuple[int, int, int, int]:
    xc, yc, w, h = box
    x1 = int((xc - w / 2.0) * image_w)
    y1 = int((yc - h / 2.0) * image_h)
    x2 = int((xc + w / 2.0) * image_w)
    y2 = int((yc + h / 2.0) * image_h)

    x1 = max(0, min(image_w - 1, x1))
    y1 = max(0, min(image_h - 1, y1))
    x2 = max(0, min(image_w - 1, x2))
    y2 = max(0, min(image_h - 1, y2))

    if x2 <= x1:
        x2 = min(image_w - 1, x1 + 1)
    if y2 <= y1:
        y2 = min(image_h - 1, y1 + 1)

    return x1, y1, x2, y2


def preprocess_gray(image_bgr: np.ndarray, enhance: bool = True) -> np.ndarray:
    """Preprocess image for SIFT detection with optional contrast enhancement.

    Based on OpenCV SIFT tutorial recommendations for better keypoint quality.
    Args:
        image_bgr: Input BGR image
        enhance: Whether to apply CLAHE contrast enhancement
    Returns:
        Preprocessed grayscale image
    """
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)

    if enhance:
        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        # Improves local contrast without over-amplifying noise
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
    else:
        gray = cv2.equalizeHist(gray)

    return gray


def build_reference_signs(
    dataset_root: Path, sift: cv2.SIFT, max_refs_per_class: int = 3
) -> list[ReferenceSign]:
    """Build reference signs by cropping each annotation's bounding box ROI.

    Cropping to the sign region isolates the sign from background clutter,
    giving SIFT keypoints that actually describe the sign's appearance.
    Multiple references per class improve recall under different lighting/angles.
    """
    data_yaml_path = dataset_root / "data.yaml"
    names = read_dataset_names(data_yaml_path)

    images_dir = dataset_root / "train" / "images"
    labels_dir = dataset_root / "train" / "labels"

    if not images_dir.exists() or not labels_dir.exists():
        raise FileNotFoundError(
            f"Expected train/images and train/labels in {dataset_root}"
        )

    # Accumulate all valid references per class, then trim to max_refs_per_class
    candidates_by_class: dict[int, list[ReferenceSign]] = {}

    for label_path in sorted(labels_dir.glob("*.txt")):
        lines = label_path.read_text(encoding="utf-8").strip().splitlines()

        img_path_jpg = images_dir / f"{label_path.stem}.jpg"
        img_path_png = images_dir / f"{label_path.stem}.png"
        img_path_jpeg = images_dir / f"{label_path.stem}.jpeg"
        for candidate in (img_path_jpg, img_path_png, img_path_jpeg):
            if candidate.exists():
                image_path = candidate
                break
        else:
            continue

        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        img_h, img_w = image.shape[:2]

        for line in lines:
            parts = line.split()
            if len(parts) < 5:
                continue

            class_id = int(float(parts[0]))
            xc, yc, w, h = map(float, parts[1:5])
            x1, y1, x2, y2 = yolo_to_xyxy((xc, yc, w, h), img_w, img_h)

            roi = image[y1:y2, x1:x2]
            if roi.size == 0 or roi.shape[0] < 8 or roi.shape[1] < 8:
                continue

            # Upscale small signs so SIFT can find enough keypoints
            min_dim = min(roi.shape[:2])
            if min_dim < 64:
                scale = 64.0 / min_dim
                roi = cv2.resize(
                    roi, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
                )

            roi_gray = preprocess_gray(roi, enhance=True)
            keypoints, descriptors = sift.detectAndCompute(roi_gray, None)

            if descriptors is None or len(keypoints) < 4:
                continue

            ref = ReferenceSign(
                class_id=class_id,
                class_name=names.get(class_id, str(class_id)),
                image_path=image_path,
                roi_gray=roi_gray,
                keypoints=keypoints,
                descriptors=descriptors,
                width=roi_gray.shape[1],
                height=roi_gray.shape[0],
            )
            candidates_by_class.setdefault(class_id, []).append(ref)

    # For each class keep the top N references by keypoint count (richer = better)
    result: list[ReferenceSign] = []
    for class_id, refs in candidates_by_class.items():
        refs.sort(key=lambda r: len(r.keypoints), reverse=True)
        result.extend(refs[:max_refs_per_class])

    built_classes = set(candidates_by_class.keys())
    all_classes = set(names.keys())
    missing = all_classes - built_classes
    if missing:
        missing_names = [names[c] for c in sorted(missing)]
        print(
            f"Warning: No usable references for {len(missing)} class(es): "
            f"{missing_names}"
        )

    return result


def is_valid_quadrilateral(polygon: np.ndarray, img_w: int, img_h: int) -> bool:
    """Reject degenerate or twisted projected quadrilaterals.

    Checks:
    - All four corners are within a reasonable margin of the image
    - Polygon is convex (consistent cross-product sign across all edges)
    - Compactness: area / perimeter^2 ratio rejects very thin slivers
    """
    pts = polygon.reshape(-1, 2).astype(np.float32)
    if len(pts) != 4:
        return False

    # Allow a modest margin outside the image frame
    margin = max(img_w, img_h) * 0.5
    if not (
        pts[:, 0].min() > -margin
        and pts[:, 0].max() < img_w + margin
        and pts[:, 1].min() > -margin
        and pts[:, 1].max() < img_h + margin
    ):
        return False

    # Convexity: all edge cross-products must share the same sign
    n = 4
    cross_signs = []
    for i in range(n):
        v1 = pts[(i + 1) % n] - pts[i]
        v2 = pts[(i + 2) % n] - pts[(i + 1) % n]
        cross = float(v1[0] * v2[1] - v1[1] * v2[0])
        if abs(cross) > 1e-6:
            cross_signs.append(1 if cross > 0 else -1)

    if len(set(cross_signs)) > 1:
        return False  # Not convex / twisted

    # Compactness rejects needle-thin slivers (e.g., collapsed homography)
    area = float(abs(cv2.contourArea(pts)))
    perimeter = float(cv2.arcLength(pts, closed=True))
    if perimeter < 1.0:
        return False
    compactness = 16.0 * area / (perimeter**2)
    if compactness < 0.04:
        return False

    return True


def polygon_iou(poly_a: np.ndarray, poly_b: np.ndarray) -> float:
    """Approximate IoU between two quadrilateral polygons via bounding box overlap."""

    def bbox(p: np.ndarray) -> tuple[float, float, float, float]:
        pts = p.reshape(-1, 2)
        return pts[:, 0].min(), pts[:, 1].min(), pts[:, 0].max(), pts[:, 1].max()

    ax1, ay1, ax2, ay2 = bbox(poly_a)
    bx1, by1, bx2, by2 = bbox(poly_b)

    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)

    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    if inter == 0.0:
        return 0.0

    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


def non_max_suppression(
    detections: list[DetectionResult], iou_thresh: float = 0.4
) -> list[DetectionResult]:
    """Suppress overlapping detections, keeping the highest-scoring one per region."""
    if not detections:
        return []

    keep: list[DetectionResult] = []
    for det in detections:  # Already sorted by score descending
        suppressed = any(
            polygon_iou(det.polygon, kept.polygon) > iou_thresh for kept in keep
        )
        if not suppressed:
            keep.append(det)
    return keep


def appearance_similarity(
    query_gray: np.ndarray,
    ref_gray: np.ndarray,
    homography: np.ndarray,
) -> float:
    """Warp the reference into the query frame and compute NCC similarity.

    This catches cases where SIFT geometry matches but the actual sign content
    (e.g. arrow direction) is different — SIFT keypoints on the circular border
    are similar for all round signs, but the interior differs.

    Returns a value in [-1, 1]; higher is more similar.
    """
    h, w = ref_gray.shape[:2]
    warped_ref = cv2.warpPerspective(
        ref_gray, homography, (query_gray.shape[1], query_gray.shape[0])
    )

    # Extract the projected region from the query
    corners = np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]]).reshape(
        -1, 1, 2
    )
    proj = cv2.perspectiveTransform(corners, homography).reshape(-1, 2)

    x1 = max(0, int(proj[:, 0].min()))
    y1 = max(0, int(proj[:, 1].min()))
    x2 = min(query_gray.shape[1], int(proj[:, 0].max()))
    y2 = min(query_gray.shape[0], int(proj[:, 1].max()))

    if x2 <= x1 or y2 <= y1:
        return 0.0

    region_query = query_gray[y1:y2, x1:x2]
    region_warped = warped_ref[y1:y2, x1:x2]

    if region_query.size < 100 or region_warped.size < 100:
        return 0.0

    # Resize to common size for comparison
    size = (64, 64)
    rq = cv2.resize(region_query, size).astype(np.float32)
    rw = cv2.resize(region_warped, size).astype(np.float32)

    # Normalised cross-correlation
    rq -= rq.mean()
    rw -= rw.mean()
    denom = np.linalg.norm(rq) * np.linalg.norm(rw)
    if denom < 1e-6:
        return 0.0
    return float(np.dot(rq.ravel(), rw.ravel()) / denom)


def create_sift_detector() -> cv2.SIFT:
    """Create SIFT detector with OpenCV paper-recommended parameters.

    Parameters from the SIFT paper (Lowe, 2004):
    - nfeatures: 0 (unlimited)
    - nOctaveLayers: 3
    - contrastThreshold: 0.03 (rejects low-contrast keypoints)
    - edgeThreshold: 10 (filters out edge responses)
    - sigma: 1.6 (initial scale)
    """
    return cv2.SIFT_create(
        nfeatures=0,
        nOctaveLayers=3,
        contrastThreshold=0.03,
        edgeThreshold=10.0,
        sigma=1.6,
    )


def create_feature_matcher(use_flann: bool = True) -> Any:
    """Create feature matcher using FLANN or BFMatcher.

    FLANN (Fast Library for Approximate Nearest Neighbors) is more efficient
    for large datasets but requires parameters tuning. BFMatcher is more reliable.

    Args:
        use_flann: Use FLANN matcher if True, else use BruteForce matcher
    Returns:
        Configured matcher object
    """
    if use_flann:
        # FLANN parameters for SIFT descriptors (128-dimensional)
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)  # Or pass empty dictionary
        return cv2.FlannBasedMatcher(index_params, search_params)
    else:
        # BruteForce matcher - more reliable for small feature sets
        return cv2.BFMatcher(cv2.NORM_L2, crossCheck=False)


def detect_signs(
    image_bgr: np.ndarray,
    references: list[ReferenceSign],
    ratio_thresh: float = 0.75,
    min_good: int = 6,
    min_inliers: int = 6,
    min_inlier_ratio: float = 0.3,
    min_ncc: float = 0.1,
    nms_thresh: float = 0.4,
    use_flann: bool = False,
) -> list[DetectionResult]:
    """Detect traffic signs using SIFT features and homography.

    Algorithm steps based on OpenCV SIFT tutorial:
    1. Scale-space extrema detection (SIFT.detectAndCompute)
    2. Keypoint localization (built into SIFT)
    3. Orientation assignment (built into SIFT)
    4. Keypoint matching (KNN+Lowe ratio test)
    5. Homography validation (RANSAC)
    6. Polygon geometry validation (convexity + compactness)
    7. Per-class deduplication + NMS

    Args:
        image_bgr: Input BGR image
        references: List of reference sign objects
        ratio_thresh: Lowe ratio threshold (lower = stricter)
        min_good: Minimum good matches for homography
        min_inliers: Minimum RANSAC inliers
        min_inlier_ratio: Minimum ratio of inliers to good matches
        nms_thresh: IoU threshold for non-maximum suppression
        use_flann: Use FLANN matcher instead of BFMatcher
    Returns:
        List of detected signs sorted by score, after NMS
    """
    sift = create_sift_detector()
    matcher = create_feature_matcher(use_flann=use_flann)

    query_gray = preprocess_gray(image_bgr, enhance=True)
    query_keypoints, query_descriptors = sift.detectAndCompute(query_gray, None)

    if query_descriptors is None or len(query_keypoints) < 4:
        return []

    img_h, img_w = image_bgr.shape[:2]
    image_area = float(img_h * img_w)

    # Collect all raw candidate detections across all references
    candidates: list[DetectionResult] = []

    for ref in references:
        knn_matches = matcher.knnMatch(ref.descriptors, query_descriptors, k=2)

        good_matches: list[cv2.DMatch] = []
        for pair in knn_matches:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < ratio_thresh * n.distance:
                good_matches.append(m)

        if len(good_matches) < min_good:
            continue

        ref_pts = np.float32(
            [ref.keypoints[m.queryIdx].pt for m in good_matches]
        ).reshape(-1, 1, 2)
        qry_pts = np.float32(
            [query_keypoints[m.trainIdx].pt for m in good_matches]
        ).reshape(-1, 1, 2)

        homography, mask = cv2.findHomography(ref_pts, qry_pts, cv2.RANSAC, 5.0)
        if homography is None or mask is None:
            continue

        inliers = int(mask.sum())
        if inliers < min_inliers:
            continue
        inlier_ratio = inliers / max(1, len(good_matches))
        if inlier_ratio < min_inlier_ratio:
            continue

        corners = np.float32(
            [
                [0, 0],
                [ref.width - 1, 0],
                [ref.width - 1, ref.height - 1],
                [0, ref.height - 1],
            ]
        ).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(corners, homography)

        if not np.isfinite(projected).all():
            continue

        # Step 6: Validate the projected quadrilateral; if degenerate (keypoints
        # clustered so homography extrapolates poorly), fall back to the bounding
        # box of the inlier query keypoints — more stable with sparse matches.
        if not is_valid_quadrilateral(projected, img_w, img_h):
            inlier_mask = mask.ravel().astype(bool)
            inlier_qry_pts = qry_pts[inlier_mask].reshape(-1, 2)
            x1_k = float(inlier_qry_pts[:, 0].min())
            y1_k = float(inlier_qry_pts[:, 1].min())
            x2_k = float(inlier_qry_pts[:, 0].max())
            y2_k = float(inlier_qry_pts[:, 1].max())
            projected = np.float32(
                [[x1_k, y1_k], [x2_k, y1_k], [x2_k, y2_k], [x1_k, y2_k]]
            ).reshape(-1, 1, 2)
            if not is_valid_quadrilateral(projected, img_w, img_h):
                continue

        polygon = projected.reshape(-1, 2)
        area = abs(cv2.contourArea(polygon.astype(np.float32)))
        if area < 200.0 or area > (0.9 * image_area):
            continue

        # Step 6b: Appearance verification — NCC between warped reference and query
        # This discriminates signs with similar geometry (e.g. circular) but different content
        ncc = appearance_similarity(query_gray, ref.roi_gray, homography)
        if ncc < min_ncc:
            continue

        # Score: geometric quality + appearance similarity
        score = inliers * inlier_ratio + 5.0 * inlier_ratio + 10.0 * max(0.0, ncc)

        candidates.append(
            DetectionResult(
                class_id=ref.class_id,
                class_name=ref.class_name,
                score=score,
                good_matches=len(good_matches),
                inliers=inliers,
                polygon=projected,
            )
        )

    if not candidates:
        return []

    candidates.sort(key=lambda d: d.score, reverse=True)

    # Step 7a: Per-class deduplication — keep only the best-scoring result per class
    seen_classes: set[int] = set()
    deduped: list[DetectionResult] = []
    for det in candidates:
        if det.class_id not in seen_classes:
            seen_classes.add(det.class_id)
            deduped.append(det)

    # Step 7b: Cross-class NMS — suppress overlapping boxes regardless of class
    return non_max_suppression(deduped, iou_thresh=nms_thresh)


def draw_detections(
    image_bgr: np.ndarray, detections: list[DetectionResult]
) -> np.ndarray:
    vis = image_bgr.copy()
    img_h, img_w = vis.shape[:2]

    # Color palette for different detections
    colors = [(0, 255, 0), (0, 165, 255), (255, 0, 0)]  # Green, Orange, Red

    for idx, det in enumerate(detections[:3], start=1):
        poly = np.int32(det.polygon)
        color = colors[idx - 1]

        # Draw polyline box
        cv2.polylines(vis, [poly], isClosed=True, color=color, thickness=3)

        # Calculate center position for text placement
        pts = poly.reshape(-1, 2)
        center_x = int(np.mean(pts[:, 0]))
        center_y = int(np.mean(pts[:, 1]))

        # Get top-left corner for reference
        top_left = pts.min(axis=0)
        top_y = int(top_left[1])

        # Text to display
        text = (
            f"{idx}) {det.class_name} | inliers={det.inliers} | good={det.good_matches}"
        )
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.65
        font_thickness = 2

        # Get text size for background
        text_size, baseline = cv2.getTextSize(text, font, font_scale, font_thickness)
        text_w, text_h = text_size

        # Position text above the box with padding
        text_x = center_x - text_w // 2
        text_y = top_y - 15

        # Clamp to image boundaries
        text_x = max(5, min(text_x, img_w - text_w - 5))
        text_y = max(text_h + 5, text_y)

        # Draw background rectangle for better text visibility
        cv2.rectangle(
            vis,
            (text_x - 5, text_y - text_h - 5),
            (text_x + text_w + 5, text_y + baseline + 5),
            color,
            -1,  # Filled rectangle
        )

        # Draw text
        cv2.putText(
            vis,
            text,
            (text_x, text_y),
            font,
            font_scale,
            (0, 0, 0),  # Black text
            font_thickness,
            cv2.LINE_AA,
        )

    return vis


def ensure_sift_available() -> None:
    if not hasattr(cv2, "SIFT_create"):
        raise RuntimeError(
            "cv2.SIFT_create is unavailable. Install OpenCV with contrib modules: "
            "pip install opencv-contrib-python"
        )


def tune_parameters(
    image_bgr: np.ndarray,
    references: list[ReferenceSign],
) -> dict[str, float]:
    """Test multiple parameter combinations and find one with best detection."""
    print("\n=== TUNING MODE ===")
    print("Testing parameter combinations to find best detection...\n")

    ratio_thresholds = [0.70, 0.75, 0.80]
    min_good_vals = [4, 8, 12]
    min_inliers_vals = [4, 8]
    min_inlier_ratios = [0.20, 0.35, 0.50]

    best_result = {
        "ratio_thresh": 0.8,
        "min_good": 4,
        "min_inliers": 4,
        "min_inlier_ratio": 0.2,
        "detections": 0,
        "total_inliers": 0,
        "score": 0,
    }

    tested = 0
    found_configs = 0

    for ratio_thresh in ratio_thresholds:
        for min_good in min_good_vals:
            for min_inliers in min_inliers_vals:
                for min_inlier_ratio in min_inlier_ratios:
                    tested += 1
                    try:
                        detections = detect_signs(
                            image_bgr,
                            references,
                            ratio_thresh,
                            min_good,
                            min_inliers,
                            min_inlier_ratio,
                        )

                        if detections:
                            found_configs += 1
                            total_inliers = sum(d.inliers for d in detections)
                            # Score: favor more detections, then more inliers
                            score = len(detections) * 100 + total_inliers

                            if score > best_result["score"]:
                                best_result["ratio_thresh"] = ratio_thresh
                                best_result["min_good"] = min_good
                                best_result["min_inliers"] = min_inliers
                                best_result["min_inlier_ratio"] = min_inlier_ratio
                                best_result["detections"] = len(detections)
                                best_result["total_inliers"] = total_inliers
                                best_result["score"] = score

                                print(
                                    f"Found: {len(detections)} detections, "
                                    f"{total_inliers} total inliers, "
                                    f"ratio_thresh={ratio_thresh}, min_good={min_good}, "
                                    f"min_inliers={min_inliers}, min_inlier_ratio={min_inlier_ratio}"
                                )
                    except Exception:
                        pass

    print(f"\nTested {tested} configurations, found {found_configs} with detections")
    print(f"\nBest parameters found:")
    print(f"  ratio_thresh: {best_result['ratio_thresh']}")
    print(f"  min_good: {best_result['min_good']}")
    print(f"  min_inliers: {best_result['min_inliers']}")
    print(f"  min_inlier_ratio: {best_result['min_inlier_ratio']}")
    print(f"  Detections: {best_result['detections']}")
    print(f"  Total inliers: {best_result['total_inliers']}\n")

    return {
        "ratio_thresh": best_result["ratio_thresh"],
        "min_good": best_result["min_good"],
        "min_inliers": best_result["min_inliers"],
        "min_inlier_ratio": best_result["min_inlier_ratio"],
    }


def main() -> None:
    args = parse_args()
    ensure_sift_available()

    dataset_root = Path(args.dataset_root).resolve()
    image_path = Path(args.input).resolve()

    if not dataset_root.exists():
        raise FileNotFoundError(f"Dataset root not found: {dataset_root}")
    if not image_path.exists():
        raise FileNotFoundError(f"Input image not found: {image_path}")

    image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError(f"Failed to read input image: {image_path}")

    sift = create_sift_detector()
    references = build_reference_signs(
        dataset_root, sift, max_refs_per_class=args.max_refs_per_class
    )

    print(f"Loaded {len(references)} reference sign crops from: {dataset_root}")
    for ref in references:
        print(
            f"  class={ref.class_id} ({ref.class_name}), "
            f"kps={len(ref.keypoints)}, file={ref.image_path.name}"
        )

    # Use tuning if requested
    if args.tune:
        params = tune_parameters(image_bgr, references)
        ratio_thresh = params["ratio_thresh"]
        min_good = params["min_good"]
        min_inliers = params["min_inliers"]
        min_inlier_ratio = params["min_inlier_ratio"]
    else:
        ratio_thresh = args.ratio_thresh
        min_good = args.min_good
        min_inliers = args.min_inliers
        min_inlier_ratio = args.min_inlier_ratio

    detections = detect_signs(
        image_bgr=image_bgr,
        references=references,
        ratio_thresh=ratio_thresh,
        min_good=min_good,
        min_inliers=min_inliers,
        min_inlier_ratio=min_inlier_ratio,
        nms_thresh=args.nms_thresh,
        min_ncc=0.1,
    )

    if not detections:
        print("No sign detected.")
        rendered = image_bgr
    else:
        print(f"Detections: {len(detections)}")
        for i, det in enumerate(detections[:5], start=1):
            print(
                f"  {i}. class={det.class_id} ({det.class_name}) "
                f"score={det.score:.3f} good={det.good_matches} inliers={det.inliers}"
            )

        rendered = draw_detections(image_bgr, detections)

    if args.output:
        output_path = Path(args.output).resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_path), rendered)
        print(f"Saved visualization: {output_path}")

    if args.show:
        cv2.imshow("SIFT Sign Detection", rendered)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

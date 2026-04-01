"""ONNX-Runtime inference backend for YOLO sign detection.

Handles:
  - Letterbox preprocessing
  - YOLOv8/YOLO11 output decoding  [1, 4+nc, num_anchors]
  - Per-class NMS
  - Optional RealSense depth gating
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


@dataclass
class YoloDetection:
    class_id: int
    class_name: str
    score: float
    x1: int
    y1: int
    x2: int
    y2: int


def load_model(model_path: str, num_threads: int = 4):
    """Load an ONNX model with ONNX Runtime.  Returns (session, input_name)."""
    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise SystemExit(
            "onnxruntime is not installed.  Install it with:\n"
            "  pip install onnxruntime"
        ) from exc

    path = Path(model_path)
    if not path.exists():
        raise FileNotFoundError(f"ONNX model not found: {path}")

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = num_threads
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

    session = ort.InferenceSession(
        str(path),
        sess_options=opts,
        providers=["CPUExecutionProvider"],
    )
    input_name = session.get_inputs()[0].name
    return session, input_name


def _letterbox(
    img: np.ndarray, new_shape: tuple[int, int]
) -> tuple[np.ndarray, float, float, float]:
    """Resize with padding to new_shape (h, w). Returns (img, ratio, pad_w, pad_h)."""
    h, w = img.shape[:2]
    ratio = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad_w = int(round(w * ratio))
    new_unpad_h = int(round(h * ratio))
    pad_w = (new_shape[1] - new_unpad_w) / 2
    pad_h = (new_shape[0] - new_unpad_h) / 2

    resized = cv2.resize(img, (new_unpad_w, new_unpad_h), interpolation=cv2.INTER_LINEAR)
    top = int(round(pad_h - 0.1))
    bottom = int(round(pad_h + 0.1))
    left = int(round(pad_w - 0.1))
    right = int(round(pad_w + 0.1))
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    return padded, ratio, pad_w, pad_h


def preprocess(frame: np.ndarray, imgsz: int) -> tuple[np.ndarray, float, float, float]:
    """BGR frame → NCHW float32 tensor + letterbox params."""
    lb, ratio, pad_w, pad_h = _letterbox(frame, (imgsz, imgsz))
    rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    tensor = rgb.transpose(2, 0, 1)[np.newaxis]  # [1, 3, H, W]
    return tensor, ratio, pad_w, pad_h


def _decode_output(
    raw: np.ndarray,
    ratio: float,
    pad_w: float,
    pad_h: float,
    conf_thresh: float,
    iou_thresh: float,
    class_names: list[str],
) -> list[YoloDetection]:
    """Decode YOLOv8/YOLO11 ONNX output [1, 4+nc, num_anchors]."""
    pred = raw[0].T  # [num_anchors, 4+nc]

    box_raw = pred[:, :4]    # cx, cy, w, h  (letterbox coords)
    cls_scores = pred[:, 4:]  # [num_anchors, nc]

    class_ids = np.argmax(cls_scores, axis=1)
    confidences = cls_scores[np.arange(len(cls_scores)), class_ids]

    mask = confidences >= conf_thresh
    if not mask.any():
        return []

    box_raw = box_raw[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]

    # cx,cy,w,h → x1,y1,x2,y2  (original image coords)
    x1 = (box_raw[:, 0] - box_raw[:, 2] / 2 - pad_w) / ratio
    y1 = (box_raw[:, 1] - box_raw[:, 3] / 2 - pad_h) / ratio
    x2 = (box_raw[:, 0] + box_raw[:, 2] / 2 - pad_w) / ratio
    y2 = (box_raw[:, 1] + box_raw[:, 3] / 2 - pad_h) / ratio

    results: list[YoloDetection] = []
    for cls in np.unique(class_ids):
        m = class_ids == cls
        cls_x1, cls_y1 = x1[m], y1[m]
        cls_x2, cls_y2 = x2[m], y2[m]
        cls_conf = confidences[m]

        # cv2.dnn.NMSBoxes expects [x, y, w, h]
        xywh = np.stack(
            [cls_x1, cls_y1, cls_x2 - cls_x1, cls_y2 - cls_y1], axis=1
        ).tolist()
        indices = cv2.dnn.NMSBoxes(xywh, cls_conf.tolist(), conf_thresh, iou_thresh)
        if len(indices) == 0:
            continue
        for i in np.array(indices).flatten():
            name = class_names[int(cls)] if int(cls) < len(class_names) else str(cls)
            results.append(
                YoloDetection(
                    class_id=int(cls),
                    class_name=name,
                    score=float(cls_conf[i]),
                    x1=int(cls_x1[i]),
                    y1=int(cls_y1[i]),
                    x2=int(cls_x2[i]),
                    y2=int(cls_y2[i]),
                )
            )

    return results


def detect(
    session,
    input_name: str,
    frame: np.ndarray,
    class_names: list[str],
    imgsz: int = 320,
    conf_thresh: float = 0.40,
    iou_thresh: float = 0.45,
    depth_frame: Optional[np.ndarray] = None,
    min_depth: float = 0.3,
    max_depth: float = 15.0,
) -> list[YoloDetection]:
    """Run inference on a BGR frame and return detections.

    Args:
        depth_frame: Optional aligned depth image (float32 metres or uint16 mm).
                     When provided, detections whose centre depth falls outside
                     [min_depth, max_depth] are discarded.
    """
    tensor, ratio, pad_w, pad_h = preprocess(frame, imgsz)
    raw = session.run(None, {input_name: tensor})

    detections = _decode_output(
        raw[0], ratio, pad_w, pad_h, conf_thresh, iou_thresh, class_names
    )

    if depth_frame is None or len(detections) == 0:
        return detections

    # Depth gating using the aligned depth image
    h_img, w_img = frame.shape[:2]
    h_d, w_d = depth_frame.shape[:2]
    scale_x = w_d / w_img
    scale_y = h_d / h_img

    # Normalise depth to metres
    if depth_frame.dtype == np.uint16:
        depth_m = depth_frame.astype(np.float32) / 1000.0
    else:
        depth_m = depth_frame.astype(np.float32)

    filtered: list[YoloDetection] = []
    for det in detections:
        cx = int(((det.x1 + det.x2) / 2) * scale_x)
        cy = int(((det.y1 + det.y2) / 2) * scale_y)
        cx = np.clip(cx, 0, w_d - 1)
        cy = np.clip(cy, 0, h_d - 1)
        dist = float(depth_m[cy, cx])
        if dist > 0 and min_depth <= dist <= max_depth:
            filtered.append(det)

    return filtered

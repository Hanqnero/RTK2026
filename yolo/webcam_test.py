"""Webcam inference GUI for RTK sign detection.

Supports both .pt (ultralytics) and .onnx (onnxruntime) models.

Usage:
    python webcam_test.py --model yolov8n_finetuned.pt
    python webcam_test.py --model yolov8n_finetuned.onnx --names dataset/data.yaml

Controls:
    Q / ESC   quit
    SPACE     pause / resume
    S         save screenshot
    +  /  =   raise confidence threshold by 0.05
    -         lower confidence threshold by 0.05
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


# ── colour palette (one per class, BGR) ────────────────────────────────────
_PALETTE = [
    (220,  20,  60), ( 30, 144, 255), ( 50, 205,  50), (255, 165,   0),
    (147, 112, 219), (  0, 206, 209), (255,  20, 147), (255, 215,   0),
]

def _colour(class_id: int) -> tuple[int, int, int]:
    return _PALETTE[class_id % len(_PALETTE)]


# ── ONNX backend (mirrors yolo_sign_detection.py, self-contained here) ─────

def _load_onnx(model_path: str, threads: int):
    try:
        import onnxruntime as ort
    except ImportError:
        raise SystemExit("onnxruntime not installed.  Run: pip install onnxruntime")
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = threads
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    sess = ort.InferenceSession(model_path, opts, providers=["CPUExecutionProvider"])
    return sess, sess.get_inputs()[0].name


def _letterbox(img, size):
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    pw = (size - nw) / 2
    ph = (size - nh) / 2
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(
        resized,
        int(round(ph - 0.1)), int(round(ph + 0.1)),
        int(round(pw - 0.1)), int(round(pw + 0.1)),
        cv2.BORDER_CONSTANT, value=(114, 114, 114),
    )
    return padded, r, pw, ph


def _infer_onnx(sess, input_name, frame, imgsz, conf_thresh, iou_thresh, names):
    lb, ratio, pw, ph = _letterbox(frame, imgsz)
    tensor = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    tensor = tensor.transpose(2, 0, 1)[np.newaxis]
    raw = sess.run(None, {input_name: tensor})[0]  # [1, 4+nc, anchors]
    pred = raw[0].T                                 # [anchors, 4+nc]

    cls_scores = pred[:, 4:]
    class_ids = np.argmax(cls_scores, axis=1)
    confidences = cls_scores[np.arange(len(cls_scores)), class_ids]
    mask = confidences >= conf_thresh
    pred, class_ids, confidences = pred[mask], class_ids[mask], confidences[mask]

    results = []
    for cls in np.unique(class_ids):
        m = class_ids == cls
        box = pred[m, :4]
        conf = confidences[m]
        x1 = (box[:, 0] - box[:, 2] / 2 - pw) / ratio
        y1 = (box[:, 1] - box[:, 3] / 2 - ph) / ratio
        x2 = (box[:, 0] + box[:, 2] / 2 - pw) / ratio
        y2 = (box[:, 1] + box[:, 3] / 2 - ph) / ratio
        xywh = np.stack([x1, y1, x2 - x1, y2 - y1], 1).tolist()
        idxs = cv2.dnn.NMSBoxes(xywh, conf.tolist(), conf_thresh, iou_thresh)
        for i in np.array(idxs).flatten():
            results.append((int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i]),
                            float(conf[i]), int(cls),
                            names[int(cls)] if int(cls) < len(names) else str(cls)))
    return results  # list of (x1,y1,x2,y2, score, class_id, name)


# ── drawing ─────────────────────────────────────────────────────────────────

def _draw(frame, detections, fps, conf_thresh, paused):
    for x1, y1, x2, y2, score, cid, name in detections:
        colour = _colour(cid)
        cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
        label = f"{name} {score:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), colour, -1)
        cv2.putText(frame, label, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    status = f"FPS: {fps:5.1f}  conf: {conf_thresh:.2f}"
    if paused:
        status += "  [PAUSED]"
    cv2.putText(frame, status, (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, "Q=quit  SPACE=pause  S=save  +/-=conf",
                (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
    return frame


# ── argument parsing ─────────────────────────────────────────────────────────

def _load_names(names_arg: str, model_path: str) -> list[str]:
    """Resolve class names from --names (yaml path or comma list) or data.yaml sibling."""
    if names_arg:
        p = Path(names_arg)
        if p.exists():
            with open(p) as f:
                cfg = yaml.safe_load(f)
            return cfg.get("names", [])
        return [n.strip() for n in names_arg.split(",")]

    # Try to find a data.yaml next to the model
    for candidate in Path(model_path).parent.glob("*.yaml"):
        with open(candidate) as f:
            cfg = yaml.safe_load(f)
        if "names" in cfg:
            return cfg["names"]

    return []


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model", required=True,
                   help=".pt or .onnx model file")
    p.add_argument("--names", default="",
                   help="data.yaml path OR comma-separated class names (ONNX only)")
    p.add_argument("--camera", type=int, default=0,
                   help="Camera index (default 0)")
    p.add_argument("--imgsz", type=int, default=320,
                   help="Inference size for ONNX models (default 320)")
    p.add_argument("--conf", type=float, default=0.40,
                   help="Confidence threshold")
    p.add_argument("--iou", type=float, default=0.45,
                   help="NMS IoU threshold (ONNX only)")
    p.add_argument("--threads", type=int, default=4,
                   help="ONNX Runtime CPU threads")
    return p.parse_args()


# ── main loop ────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    model_path = args.model
    is_onnx = model_path.endswith(".onnx")

    if is_onnx:
        names = _load_names(args.names, model_path)
        sess, input_name = _load_onnx(model_path, args.threads)
        print(f"ONNX model loaded  ({len(names)} classes)")
    else:
        try:
            from ultralytics import YOLO
        except ImportError:
            raise SystemExit("ultralytics not installed.  Run: pip install ultralytics")
        model = YOLO(model_path)
        names = list(model.names.values())
        print(f"Ultralytics model loaded  ({len(names)} classes)")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open camera {args.camera}")

    conf_thresh = args.conf
    paused = False
    fps = 0.0
    last_frame = None
    last_dets = []
    screenshot_idx = 0

    print("Window open — press Q or ESC to quit.")

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret:
                break

            t0 = time.perf_counter()
            if is_onnx:
                dets = _infer_onnx(sess, input_name, frame,
                                   args.imgsz, conf_thresh, args.iou, names)
            else:
                results = model(frame, conf=conf_thresh, verbose=False)[0]
                dets = []
                for box in results.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    cid = int(box.cls[0])
                    dets.append((x1, y1, x2, y2, float(box.conf[0]),
                                 cid, names[cid] if cid < len(names) else str(cid)))

            fps = 1.0 / max(time.perf_counter() - t0, 1e-6)
            last_frame = frame.copy()
            last_dets = dets
        else:
            frame = last_frame.copy() if last_frame is not None else np.zeros((480, 640, 3), np.uint8)

        display = _draw(frame, last_dets, fps, conf_thresh, paused)
        cv2.imshow("RTK Sign Detection", display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):   # Q or ESC
            break
        elif key == ord(" "):
            paused = not paused
        elif key in (ord("+"), ord("=")):
            conf_thresh = min(conf_thresh + 0.05, 0.95)
        elif key == ord("-"):
            conf_thresh = max(conf_thresh - 0.05, 0.05)
        elif key in (ord("s"), ord("S")):
            fname = f"screenshot_{screenshot_idx:03d}.jpg"
            cv2.imwrite(fname, display)
            print(f"Saved {fname}")
            screenshot_idx += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

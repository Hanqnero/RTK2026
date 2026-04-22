#!/usr/bin/env python3
"""Quick offline ONNX image test for RTK2026 sign detection."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort
from PIL import Image


DEFAULT_LABELS = [
    "bus_stop",
    "move_forward",
    "no_turn_left",
    "no_turn_right",
    "obstacle",
    "parking_spot",
    "turn_left",
    "turn_right",
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--image", required=True)
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--topk", type=int, default=10)
    parser.add_argument("--threshold", type=float, default=0.0)
    args = parser.parse_args()

    model_path = Path(args.model)
    image_path = Path(args.image)

    image = Image.open(image_path).convert("RGB")
    orig_w, orig_h = image.size
    resized = image.resize((args.input_size, args.input_size))
    blob = np.transpose(np.asarray(resized).astype("float32") / 255.0, (2, 0, 1))[None, ...]

    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output = np.asarray(session.run(None, {input_name: blob})[0])
    if output.ndim == 3 and output.shape[0] == 1:
        output = output[0]

    if output.ndim != 2:
        raise RuntimeError(f"Unexpected output shape: {output.shape}")
    if output.shape[1] == 6:
        rows = output
    elif output.shape[0] == 6 and output.shape[1] > 6:
        rows = output.T
    else:
        raise RuntimeError(f"Unsupported detector output format: {output.shape}")

    rows = rows[np.argsort(rows[:, 4])[::-1]]
    print(f"input={session.get_inputs()[0].shape} output={list(output.shape)}")
    for i, row in enumerate(rows[: args.topk]):
        x1, y1, x2, y2, score, cls = row[:6].tolist()
        if score < args.threshold:
            continue
        cls = int(cls)
        px1 = max(0, int(round(x1 * orig_w / args.input_size)))
        py1 = max(0, int(round(y1 * orig_h / args.input_size)))
        px2 = min(orig_w - 1, int(round(x2 * orig_w / args.input_size)))
        py2 = min(orig_h - 1, int(round(y2 * orig_h / args.input_size)))
        label = DEFAULT_LABELS[cls] if 0 <= cls < len(DEFAULT_LABELS) else str(cls)
        print(
            f"{i}: class_id={cls} label={label} score={score:.4f} "
            f"box640=({x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}) "
            f"boxOrig=({px1},{py1},{px2},{py2})"
        )


if __name__ == "__main__":
    main()

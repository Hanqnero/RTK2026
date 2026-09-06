"""Focused tests for camera-buffer handling used before inference."""

from types import SimpleNamespace

import numpy as np
import pytest

from rtk2026_cv.onnx_sign_detector_node import OnnxSignDetectorNode


def _image_message(
    data: bytes, *, width: int, height: int, encoding: str, step: int
):
    return SimpleNamespace(
        data=data,
        width=width,
        height=height,
        encoding=encoding,
        step=step,
    )


def _detector_for_postprocessing() -> OnnxSignDetectorNode:
    detector = OnnxSignDetectorNode.__new__(OnnxSignDetectorNode)
    detector._input_size = 100
    detector._conf_threshold = 0.25
    detector._nms_threshold = 0.45
    detector._class_id_to_label = {6: "turn_left"}
    detector._class_id_to_route_command = {6: "left_only"}
    detector._class_id_to_stop_action = {}
    detector._bus_class_ids = set()
    return detector


def test_rgb_image_with_row_padding_is_converted_to_bgr() -> None:
    # Two RGB pixels followed by two padding bytes.
    message = _image_message(
        bytes([255, 0, 0, 0, 255, 0, 99, 99]),
        width=2,
        height=1,
        encoding="rgb8",
        step=8,
    )

    image = OnnxSignDetectorNode._image_msg_to_bgr(message)

    assert image.tolist() == [[[0, 0, 255], [0, 255, 0]]]


@pytest.mark.parametrize("encoding", ["rgba8", "bgra8"])
def test_four_channel_camera_encodings_are_accepted(encoding: str) -> None:
    message = _image_message(
        bytes([10, 20, 30, 255]),
        width=1,
        height=1,
        encoding=encoding,
        step=4,
    )

    image = OnnxSignDetectorNode._image_msg_to_bgr(message)

    assert image.shape == (1, 1, 3)
    assert image.dtype == np.uint8


def test_unknown_camera_encoding_fails_explicitly() -> None:
    message = _image_message(
        bytes([0, 0]),
        width=1,
        height=1,
        encoding="yuyv",
        step=2,
    )

    with pytest.raises(ValueError, match="Unsupported image encoding"):
        OnnxSignDetectorNode._image_msg_to_bgr(message)


def test_end_to_end_yolo_rows_are_mapped_to_driving_command() -> None:
    detector = _detector_for_postprocessing()
    output = np.asarray([[10, 20, 40, 60, 0.9, 6]], dtype=np.float32)

    candidates = list(detector._iter_candidates(output, orig_w=200, orig_h=100))

    assert len(candidates) == 1
    assert candidates[0].class_label == "turn_left"
    assert candidates[0].command == "left_only"
    assert candidates[0].box == (20, 20, 80, 60)


def test_raw_yolo_duplicates_are_suppressed_per_class() -> None:
    detector = _detector_for_postprocessing()
    # Raw YOLO layout is [x, y, w, h, class scores] x anchors.
    output = np.zeros((12, 20), dtype=np.float32)
    output[:, 0] = [50, 50, 20, 20, 0, 0, 0, 0, 0, 0, 0.9, 0]
    output[:, 1] = [51, 51, 20, 20, 0, 0, 0, 0, 0, 0, 0.8, 0]

    candidates = list(detector._iter_candidates(output, orig_w=100, orig_h=100))

    assert len(candidates) == 1
    assert candidates[0].confidence == pytest.approx(0.9)

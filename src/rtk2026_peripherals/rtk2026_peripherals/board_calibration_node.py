#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String


class BoardCalibrationNode(Node):
    def __init__(self):
        super().__init__("board_calibration")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("overlay_topic", "/camera/calibration/chessboard_overlay")
        self.declare_parameter("status_topic", "/camera/calibration/status")
        self.declare_parameter("rms_topic", "/camera/calibration/rms_error")
        self.declare_parameter("board_cols", 7)
        self.declare_parameter("board_rows", 10)
        self.declare_parameter("square_size_m", 0.025)
        self.declare_parameter("try_pattern_variants", True)
        self.declare_parameter("min_samples", 12)
        self.declare_parameter("max_samples", 30)
        self.declare_parameter("sample_min_translation", 0.12)
        self.declare_parameter("sample_min_area_delta", 0.20)
        self.declare_parameter("sample_period_sec", 0.8)
        self.declare_parameter("solvepnp_min_samples", 4)

        self._bridge = CvBridge()
        self._camera_info: CameraInfo | None = None
        self._current_k: np.ndarray | None = None
        self._current_d: np.ndarray | None = None
        self._image_size: tuple[int, int] | None = None

        cols = int(self.get_parameter("board_cols").value)
        rows = int(self.get_parameter("board_rows").value)
        square = float(self.get_parameter("square_size_m").value)
        self._pattern_size = (cols, rows)
        self._pattern_variants = self._build_pattern_variants(
            cols,
            rows,
            bool(self.get_parameter("try_pattern_variants").value),
        )
        self._objp_cache = {
            pattern: self._make_objp(pattern[0], pattern[1], square)
            for pattern in self._pattern_variants
        }

        self._criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        self._sample_period_sec = float(self.get_parameter("sample_period_sec").value)
        self._sample_min_translation = float(
            self.get_parameter("sample_min_translation").value
        )
        self._sample_min_area_delta = float(
            self.get_parameter("sample_min_area_delta").value
        )
        self._min_samples = int(self.get_parameter("min_samples").value)
        self._max_samples = int(self.get_parameter("max_samples").value)
        self._solvepnp_min_samples = int(self.get_parameter("solvepnp_min_samples").value)

        self._objpoints: list[np.ndarray] = []
        self._imgpoints: list[np.ndarray] = []
        self._sample_features: list[tuple[float, float, float]] = []
        self._last_sample_time_sec = 0.0
        self._last_status = "Waiting for board"
        self._last_rms = float("nan")
        self._last_detection = "not_found"

        image_topic = str(self.get_parameter("image_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        overlay_topic = str(self.get_parameter("overlay_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        rms_topic = str(self.get_parameter("rms_topic").value)

        self.create_subscription(Image, image_topic, self._cb_image, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._cb_camera_info, 10)

        self._overlay_pub = self.create_publisher(Image, overlay_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self._rms_pub = self.create_publisher(Float32, rms_topic, 10)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            "board_calibration started: "
            f"image_topic={image_topic}, camera_info_topic={camera_info_topic}, "
            f"board={cols}x{rows}, square={square:.3f}m"
        )

    def _cb_camera_info(self, msg: CameraInfo):
        self._camera_info = msg
        self._current_k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._current_d = np.array(msg.d, dtype=np.float64).reshape(-1, 1)

    def _cb_image(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"imgmsg_to_cv2 failed: {exc}")
            return

        self._image_size = (frame.shape[1], frame.shape[0])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners, pattern = self._find_board(gray)

        overlay = frame.copy()
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if found and corners is not None and pattern is not None:
            corners_refined = corners
            if len(corners_refined.shape) == 2:
                corners_refined = corners_refined.reshape(-1, 1, 2)
            cv2.drawChessboardCorners(
                overlay, pattern, corners_refined, True
            )
            detection_text = self._handle_detection(corners_refined, pattern, now_sec)
            self._last_detection = detection_text
            self._draw_overlay_text(overlay, detection_text)
        else:
            pattern_text = ", ".join(f"{c}x{r}" for c, r in self._pattern_variants)
            self._last_detection = f"Board not found | tried {pattern_text}"
            self._draw_overlay_text(overlay, self._last_detection)

        overlay_msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_msg.header = msg.header
        self._overlay_pub.publish(overlay_msg)

    def _handle_detection(
        self,
        corners: np.ndarray,
        pattern: tuple[int, int],
        now_sec: float,
    ) -> str:
        centroid_x = float(np.mean(corners[:, 0, 0]))
        centroid_y = float(np.mean(corners[:, 0, 1]))
        bbox = cv2.boundingRect(corners.astype(np.float32))
        area_norm = float((bbox[2] * bbox[3]) / max(1, self._image_size[0] * self._image_size[1]))
        feature = (centroid_x, centroid_y, area_norm)

        should_store = False
        if not self._sample_features:
            should_store = True
        elif now_sec - self._last_sample_time_sec >= self._sample_period_sec:
            dist_px = max(
                math.hypot(centroid_x - px, centroid_y - py)
                for px, py, _ in self._sample_features
            )
            diag = math.hypot(self._image_size[0], self._image_size[1])
            dist_norm = dist_px / max(diag, 1.0)
            area_delta = max(abs(area_norm - pa) for _, _, pa in self._sample_features)
            should_store = (
                dist_norm >= self._sample_min_translation
                or area_delta >= self._sample_min_area_delta
            )

        if should_store and len(self._imgpoints) < self._max_samples:
            self._imgpoints.append(corners.reshape(-1, 2).astype(np.float32))
            self._objpoints.append(self._objp_cache[pattern].copy())
            self._sample_features.append(feature)
            self._last_sample_time_sec = now_sec

        self._update_calibration_metrics()

        if len(self._imgpoints) < self._min_samples:
            return (
                f"Board {pattern[0]}x{pattern[1]} | samples {len(self._imgpoints)}/{self._min_samples} "
                f"| move board across frame"
            )

        if math.isnan(self._last_rms):
            return (
                f"Board {pattern[0]}x{pattern[1]} | samples {len(self._imgpoints)} "
                "| waiting for calibration estimate"
            )

        return (
            f"Board {pattern[0]}x{pattern[1]} | samples {len(self._imgpoints)} | "
            f"RMS {self._last_rms:.3f}px"
        )

    def _update_calibration_metrics(self):
        if self._image_size is None or len(self._imgpoints) < self._solvepnp_min_samples:
            self._last_status = (
                f"{self._last_detection} | current camera_info pending validation "
                f"| samples {len(self._imgpoints)}"
            )
            self._last_rms = float("nan")
            return

        try:
            rms, k_est, d_est, _, _ = cv2.calibrateCamera(
                self._objpoints,
                self._imgpoints,
                self._image_size,
                None,
                None,
            )
        except cv2.error as exc:
            self._last_status = f"Calibration error: {exc}"
            self._last_rms = float("nan")
            return

        self._last_rms = float(rms)
        parts = [
            self._last_detection,
            f"samples={len(self._imgpoints)}",
            f"rms={self._last_rms:.3f}px",
        ]

        if self._current_k is not None:
            fx_delta = float(k_est[0, 0] - self._current_k[0, 0])
            fy_delta = float(k_est[1, 1] - self._current_k[1, 1])
            cx_delta = float(k_est[0, 2] - self._current_k[0, 2])
            cy_delta = float(k_est[1, 2] - self._current_k[1, 2])
            parts.append(
                "deltaK="
                f"[fx {fx_delta:+.1f}, fy {fy_delta:+.1f}, "
                f"cx {cx_delta:+.1f}, cy {cy_delta:+.1f}]"
            )

        if self._current_d is not None and self._current_d.size > 0:
            d_est_flat = d_est.reshape(-1)
            d_cur_flat = self._current_d.reshape(-1)
            n = min(len(d_est_flat), len(d_cur_flat), 5)
            if n > 0:
                max_d_delta = float(np.max(np.abs(d_est_flat[:n] - d_cur_flat[:n])))
                parts.append(f"max|deltaD|={max_d_delta:.5f}")

        self._last_status = " | ".join(parts)

    def _draw_overlay_text(self, image: np.ndarray, headline: str):
        lines = [
            headline,
            f"Samples: {len(self._imgpoints)}/{self._min_samples} (max {self._max_samples})",
            (
                "Current RMS: n/a"
                if math.isnan(self._last_rms)
                else f"Current RMS: {self._last_rms:.3f}px"
            ),
        ]

        y = 30
        for line in lines:
            cv2.putText(
                image,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            y += 32

    def _publish_status(self):
        status_msg = String()
        status_msg.data = self._last_status
        self._status_pub.publish(status_msg)

        rms_msg = Float32()
        rms_msg.data = float(self._last_rms) if not math.isnan(self._last_rms) else -1.0
        self._rms_pub.publish(rms_msg)

    @staticmethod
    def _make_objp(cols: int, rows: int, square_size_m: float) -> np.ndarray:
        objp = np.zeros((cols * rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        objp *= square_size_m
        return objp

    @staticmethod
    def _build_pattern_variants(
        cols: int,
        rows: int,
        enable_variants: bool,
    ) -> list[tuple[int, int]]:
        variants: list[tuple[int, int]] = [(cols, rows)]
        if not enable_variants:
            return variants

        candidates = [
            (rows, cols),
            (max(cols - 1, 2), max(rows - 1, 2)),
            (max(rows - 1, 2), max(cols - 1, 2)),
        ]
        for candidate in candidates:
            if candidate not in variants:
                variants.append(candidate)
        return variants

    def _find_board(
        self,
        gray: np.ndarray,
    ) -> tuple[bool, np.ndarray | None, tuple[int, int] | None]:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE

        for pattern in self._pattern_variants:
            corners = None
            found = False

            if hasattr(cv2, "findChessboardCornersSB"):
                found, sb_corners = cv2.findChessboardCornersSB(gray, pattern, sb_flags)
                if found and sb_corners is not None:
                    corners = sb_corners.reshape(-1, 1, 2).astype(np.float32)

            if not found:
                found, raw_corners = cv2.findChessboardCorners(gray, pattern, flags)
                if found and raw_corners is not None:
                    corners = cv2.cornerSubPix(
                        gray,
                        raw_corners,
                        (11, 11),
                        (-1, -1),
                        self._criteria,
                    )

            if found and corners is not None:
                return True, corners, pattern

        return False, None, None


def main(args=None):
    rclpy.init(args=args)
    node = BoardCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

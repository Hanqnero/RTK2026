#!/usr/bin/env python3
# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# Edge-based lane detector for same-color lane lines.
# Input:  /detect/image_input  — bird's-eye (projected) image 1000x600
# Output: /detect/lane         — Float64 center X in [0, 1000]
#         /detect/lane_state   — UInt8: 2=both, 1=left only, 3=right only, 0=none
#
# Algo: grayscale → GaussianBlur → Canny → HoughLinesP → polyfit(deg=2) per side
# Handles dashed lines via maxLineGap and polynomial extrapolation.
# Handles curves via 2nd-degree polynomial fit.
#
# Calibration mode (is_detection_calibration_mode=true):
#   /detect/image_output_sub1/compressed  — Canny edge mask (tune canny_low / canny_high)
#   /detect/image_output_sub2/compressed  — Hough lines overlay (tune hough_* params)
#   /detect/image_output/compressed       — final overlay with fitted curves + center
#
# Pi tuning:
#   ros2 param set /detect_lane canny_low 40
#   ros2 param set /detect_lane canny_high 120
#   ros2 param set /detect_lane hough_min_line_len 15
#   ros2 param set /detect_lane hough_max_line_gap 80

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, UInt8

# Output image dimensions (must match image_projection output)
IMG_W = 1000
IMG_H = 600

# Processing resolution — halved for Pi performance
PROC_W = 500
PROC_H = 300
SCALE = IMG_W / PROC_W  # 2.0

# Center X in output image space; used to split left/right lines
MIDX = PROC_W // 2

# When only one lane visible, estimated half-lane width in output pixels
HALF_LANE_PX = 280

# Moving average window for poly coefficients
MOV_AVG_LEN = 5

# Minimum Hough line points to attempt a poly fit
MIN_POINTS = 6


class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 15)
        self.declare_parameter('hough_min_line_len', 20)
        self.declare_parameter('hough_max_line_gap', 60)
        self.declare_parameter('is_detection_calibration_mode', False)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_params)

        self.cvBridge = CvBridge()
        self._counter = 0

        # Poly coefficient history: shape (N, 3)
        self._avg_left = np.empty((0, 3))
        self._avg_right = np.empty((0, 3))
        self._left_fit = None
        self._right_fit = None

        sub_topic = '/detect/image_input'
        self._sub = self.create_subscription(Image, sub_topic, self._cb, 1)

        self._pub_lane = self.create_publisher(Float64, '/detect/lane', 1)
        self._pub_state = self.create_publisher(UInt8, '/detect/lane_state', 1)
        self._pub_out = self.create_publisher(
            CompressedImage, '/detect/image_output/compressed', 1)

        # Calibration debug publishers
        self._pub_edges = self.create_publisher(
            CompressedImage, '/detect/image_output_sub1/compressed', 1)
        self._pub_hough = self.create_publisher(
            CompressedImage, '/detect/image_output_sub2/compressed', 1)

        # Legacy reliability topics (kept for compat with control_lane)
        self._pub_white_rel = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1)
        self._pub_yellow_rel = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1)

    def _load_params(self):
        self.canny_low = self.get_parameter('canny_low').value
        self.canny_high = self.get_parameter('canny_high').value
        self.hough_thr = self.get_parameter('hough_threshold').value
        self.hough_min = self.get_parameter('hough_min_line_len').value
        self.hough_gap = self.get_parameter('hough_max_line_gap').value
        self.is_calib = self.get_parameter('is_detection_calibration_mode').value

    def _on_params(self, params):
        for p in params:
            if p.name == 'canny_low':
                self.canny_low = p.value
            elif p.name == 'canny_high':
                self.canny_high = p.value
            elif p.name == 'hough_threshold':
                self.hough_thr = p.value
            elif p.name == 'hough_min_line_len':
                self.hough_min = p.value
            elif p.name == 'hough_max_line_gap':
                self.hough_gap = p.value
            elif p.name == 'is_detection_calibration_mode':
                self.is_calib = p.value
        return SetParametersResult(successful=True)

    def _cb(self, msg: Image):
        # Process every 2nd frame (~15fps on Pi at 30fps input)
        self._counter += 1
        if self._counter % 2 != 0:
            return

        cv_image = self.cvBridge.imgmsg_to_cv2(msg, 'bgr8')

        # Downscale for faster processing
        small = cv2.resize(cv_image, (PROC_W, PROC_H))

        # --- Edge detection ---
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)

        if self.is_calib:
            self._pub_edges.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(edges, 'jpg'))

        # --- Hough lines ---
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_thr,
            minLineLength=self.hough_min,
            maxLineGap=self.hough_gap,
        )

        left_pts, right_pts = self._split_lines(lines)

        if self.is_calib:
            hough_vis = small.copy()
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    color = (0, 0, 255) if x1 < MIDX else (255, 0, 0)
                    cv2.line(hough_vis, (x1, y1), (x2, y2), color, 2)
            self._pub_hough.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(hough_vis, 'jpg'))

        # --- Polynomial fit per side ---
        left_fit = self._polyfit_side(left_pts, self._left_fit)
        right_fit = self._polyfit_side(right_pts, self._right_fit)

        # Update moving averages
        if left_fit is not None:
            self._avg_left = np.append(
                self._avg_left, [left_fit], axis=0)[-MOV_AVG_LEN:]
        if right_fit is not None:
            self._avg_right = np.append(
                self._avg_right, [right_fit], axis=0)[-MOV_AVG_LEN:]

        # Smoothed fits
        if len(self._avg_left):
            self._left_fit = self._avg_left.mean(axis=0)
        if len(self._avg_right):
            self._right_fit = self._avg_right.mean(axis=0)

        self._publish_result(cv_image, small)

    def _split_lines(self, lines):
        """Split Hough line segments into left / right point arrays by X position."""
        left_pts = []
        right_pts = []
        if lines is None:
            return left_pts, right_pts
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Skip nearly-horizontal lines — not lane boundaries
            if abs(y2 - y1) < 5:
                continue
            if x1 < MIDX and x2 < MIDX:
                left_pts.extend([(x1, y1), (x2, y2)])
            elif x1 >= MIDX and x2 >= MIDX:
                right_pts.extend([(x1, y1), (x2, y2)])
        return left_pts, right_pts

    def _polyfit_side(self, pts, prev_fit):
        """Fit 2nd-degree poly x = f(y) to a list of (x,y) points."""
        if len(pts) < MIN_POINTS:
            return prev_fit
        arr = np.array(pts)
        try:
            return np.polyfit(arr[:, 1], arr[:, 0], 2)
        except Exception:
            return prev_fit

    def _eval_fit(self, fit, ploty):
        """Evaluate polynomial fit at y values."""
        return fit[0] * ploty ** 2 + fit[1] * ploty + fit[2]

    def _publish_result(self, orig, small):
        ploty = np.linspace(0, PROC_H - 1, PROC_H)

        has_left = self._left_fit is not None
        has_right = self._right_fit is not None

        # Lane state
        if has_left and has_right:
            state = 2
        elif has_left:
            state = 1
        elif has_right:
            state = 3
        else:
            state = 0

        state_msg = UInt8()
        state_msg.data = state
        self._pub_state.publish(state_msg)

        # Reliability (legacy compat — publish 100 if lane seen, 0 if not)
        rel = UInt8()
        rel.data = 100 if has_left else 0
        self._pub_yellow_rel.publish(rel)
        rel.data = 100 if has_right else 0
        self._pub_white_rel.publish(rel)

        if state == 0:
            return

        # Compute center X in processing-space
        if state == 2:
            left_fitx = self._eval_fit(self._left_fit, ploty)
            right_fitx = self._eval_fit(self._right_fit, ploty)
            centerx = (left_fitx + right_fitx) / 2.0
        elif state == 1:
            left_fitx = self._eval_fit(self._left_fit, ploty)
            centerx = left_fitx + HALF_LANE_PX / SCALE
        else:
            right_fitx = self._eval_fit(self._right_fit, ploty)
            centerx = right_fitx - HALF_LANE_PX / SCALE

        # Publish center X scaled back to original image space (0–1000)
        center_at_horizon = float(centerx[PROC_H // 2]) * SCALE
        msg = Float64()
        msg.data = center_at_horizon
        self._pub_lane.publish(msg)

        # Debug overlay
        if not self.is_calib:
            # Lightweight overlay on original image
            vis = orig.copy()
            cx_img = int(center_at_horizon)
            cv2.line(vis, (cx_img, 0), (cx_img, IMG_H), (0, 255, 255), 3)
            self._pub_out.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(vis, 'jpg'))
        else:
            # Full overlay: fitted curves + center line
            vis = small.copy()
            pts_y = ploty.astype(np.int32)

            if has_left:
                xs = np.clip(
                    self._eval_fit(self._left_fit, ploty), 0, PROC_W - 1
                ).astype(np.int32)
                for y, x in zip(pts_y, xs):
                    cv2.circle(vis, (x, y), 1, (0, 0, 255), -1)

            if has_right:
                xs = np.clip(
                    self._eval_fit(self._right_fit, ploty), 0, PROC_W - 1
                ).astype(np.int32)
                for y, x in zip(pts_y, xs):
                    cv2.circle(vis, (x, y), 1, (255, 0, 0), -1)

            cx_small = int(center_at_horizon / SCALE)
            cv2.line(vis, (cx_small, 0), (cx_small, PROC_H), (0, 255, 255), 2)

            self._pub_out.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(vis, 'jpg'))


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# Center-line detector for a single colored guide line in bird's-eye images.
# Input:  /detect/image_input  — IPM image 1000x600
# Output: /detect/lane         — Float64 center X in [0, 1000]
#         /detect/lane_state   — UInt8: 2=detected, 0=none
#
# Debug topics:
#   /detect/image_output_sub1/compressed  — binary color mask
#   /detect/image_output_sub2/compressed  — mask center points by rows
#   /detect/image_output/compressed       — final fitted center line overlay

import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, UInt8

IMG_W = 1000
IMG_H = 600
PROC_W = 500
PROC_H = 300
SCALE = IMG_W / PROC_W
MOV_AVG_LEN = 5
MIN_POINTS = 8


class DetectCenterline(Node):
    def __init__(self):
        super().__init__("detect_centerline")

        self.declare_parameter("blue_h_low", 90)
        self.declare_parameter("blue_h_high", 135)
        self.declare_parameter("blue_s_low", 70)
        self.declare_parameter("blue_s_high", 255)
        self.declare_parameter("blue_v_low", 40)
        self.declare_parameter("blue_v_high", 255)
        self.declare_parameter("roi_y_min", 90)
        self.declare_parameter("roi_y_max", 299)
        self.declare_parameter("morph_open", 3)
        self.declare_parameter("morph_close", 9)
        self.declare_parameter("min_row_width", 6)
        self.declare_parameter("row_step", 2)
        self.declare_parameter("is_detection_calibration_mode", False)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_params)

        self.cvBridge = CvBridge()
        self._counter = 0
        self._avg_centers = []

        self._sub = self.create_subscription(Image, "/detect/image_input", self._cb, 1)
        self._pub_lane = self.create_publisher(Float64, "/detect/lane", 1)
        self._pub_state = self.create_publisher(UInt8, "/detect/lane_state", 1)
        self._pub_out = self.create_publisher(
            CompressedImage, "/detect/image_output/compressed", 1
        )
        self._pub_mask = self.create_publisher(
            CompressedImage, "/detect/image_output_sub1/compressed", 1
        )
        self._pub_contours = self.create_publisher(
            CompressedImage, "/detect/image_output_sub2/compressed", 1
        )
        self._pub_white_rel = self.create_publisher(
            UInt8, "/detect/white_line_reliability", 1
        )
        self._pub_yellow_rel = self.create_publisher(
            UInt8, "/detect/yellow_line_reliability", 1
        )

    def _load_params(self):
        self.blue_h_low = self.get_parameter("blue_h_low").value
        self.blue_h_high = self.get_parameter("blue_h_high").value
        self.blue_s_low = self.get_parameter("blue_s_low").value
        self.blue_s_high = self.get_parameter("blue_s_high").value
        self.blue_v_low = self.get_parameter("blue_v_low").value
        self.blue_v_high = self.get_parameter("blue_v_high").value
        self.roi_y_min = self.get_parameter("roi_y_min").value
        self.roi_y_max = self.get_parameter("roi_y_max").value
        self.morph_open = self.get_parameter("morph_open").value
        self.morph_close = self.get_parameter("morph_close").value
        self.min_row_width = self.get_parameter("min_row_width").value
        self.row_step = self.get_parameter("row_step").value
        self.is_calib = self.get_parameter("is_detection_calibration_mode").value

    def _on_params(self, params):
        for p in params:
            if p.name == "blue_h_low":
                self.blue_h_low = p.value
            elif p.name == "blue_h_high":
                self.blue_h_high = p.value
            elif p.name == "blue_s_low":
                self.blue_s_low = p.value
            elif p.name == "blue_s_high":
                self.blue_s_high = p.value
            elif p.name == "blue_v_low":
                self.blue_v_low = p.value
            elif p.name == "blue_v_high":
                self.blue_v_high = p.value
            elif p.name == "roi_y_min":
                self.roi_y_min = p.value
            elif p.name == "roi_y_max":
                self.roi_y_max = p.value
            elif p.name == "morph_open":
                self.morph_open = p.value
            elif p.name == "morph_close":
                self.morph_close = p.value
            elif p.name == "min_row_width":
                self.min_row_width = p.value
            elif p.name == "row_step":
                self.row_step = p.value
            elif p.name == "is_detection_calibration_mode":
                self.is_calib = p.value
        return SetParametersResult(successful=True)

    def _cb(self, msg: Image):
        self._counter += 1
        if self._counter % 2 != 0:
            return

        orig = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
        small = cv2.resize(orig, (PROC_W, PROC_H))

        mask = self._build_blue_mask(small)
        pts = self._extract_centerline_points(mask)

        if self.is_calib:
            self._pub_mask.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg")
            )
            contour_vis = np.zeros((PROC_H, PROC_W, 3), dtype=np.uint8)
            for x, y in pts:
                cv2.circle(contour_vis, (int(x), int(y)), 1, (0, 255, 255), -1)
            cv2.line(
                contour_vis,
                (0, self.roi_y_min),
                (PROC_W - 1, self.roi_y_min),
                (0, 255, 0),
                1,
            )
            cv2.line(
                contour_vis,
                (0, self.roi_y_max),
                (PROC_W - 1, self.roi_y_max),
                (0, 255, 0),
                1,
            )
            self._pub_contours.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(contour_vis, "jpg")
            )

        state = UInt8()
        if len(pts) < MIN_POINTS:
            state.data = 0
            self._pub_state.publish(state)
            self._publish_reliability(0)
            return

        lookahead_y = int((self.roi_y_min + self.roi_y_max) / 2)
        center_small = self._center_at_lookahead(pts, lookahead_y)
        if center_small is None:
            state.data = 0
            self._pub_state.publish(state)
            self._publish_reliability(0)
            return

        self._avg_centers.append(center_small)
        self._avg_centers = self._avg_centers[-MOV_AVG_LEN:]
        center_small = float(np.mean(self._avg_centers))
        center_at_lookahead = center_small * SCALE

        state.data = 2
        self._pub_state.publish(state)
        self._publish_reliability(100)

        lane_msg = Float64()
        lane_msg.data = center_at_lookahead
        self._pub_lane.publish(lane_msg)

        self._publish_overlay(orig, small, pts, center_at_lookahead, lookahead_y)

    def _build_blue_mask(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array(
            [self.blue_h_low, self.blue_s_low, self.blue_v_low], dtype=np.uint8
        )
        upper = np.array(
            [self.blue_h_high, self.blue_s_high, self.blue_v_high], dtype=np.uint8
        )
        mask = cv2.inRange(hsv, lower, upper)

        roi = np.zeros_like(mask)
        y0 = max(0, min(PROC_H - 1, int(self.roi_y_min)))
        y1 = max(0, min(PROC_H - 1, int(self.roi_y_max)))
        if y1 < y0:
            y0, y1 = y1, y0
        roi[y0 : y1 + 1, :] = 255
        mask = cv2.bitwise_and(mask, roi)

        if self.morph_open > 1:
            k = cv2.getStructuringElement(
                cv2.MORPH_RECT, (self.morph_open, self.morph_open)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        if self.morph_close > 1:
            k = cv2.getStructuringElement(
                cv2.MORPH_RECT, (self.morph_close, self.morph_close)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def _extract_centerline_points(self, mask):
        pts = []
        row_step = max(1, int(self.row_step))
        min_row_width = max(1, int(self.min_row_width))

        for y in range(int(self.roi_y_min), int(self.roi_y_max) + 1, row_step):
            if y < 0 or y >= PROC_H:
                continue
            xs = np.flatnonzero(mask[y] > 0)
            if len(xs) < min_row_width:
                continue

            splits = np.where(np.diff(xs) > 1)[0] + 1
            runs = np.split(xs, splits)
            if not runs:
                continue

            best_run = max(runs, key=len)
            if len(best_run) < min_row_width:
                continue

            x_center = float(best_run[0] + best_run[-1]) / 2.0
            pts.append((x_center, float(y)))
        return pts

    def _center_at_lookahead(self, pts, lookahead_y):
        arr = np.array(pts, dtype=np.float32)
        ys = arr[:, 1]
        xs = arr[:, 0]
        order = np.argsort(ys)
        ys = ys[order]
        xs = xs[order]
        if lookahead_y <= ys[0]:
            return float(xs[0])
        if lookahead_y >= ys[-1]:
            return float(xs[-1])
        return float(np.interp(float(lookahead_y), ys, xs))

    def _publish_reliability(self, value):
        rel = UInt8()
        rel.data = value
        self._pub_white_rel.publish(rel)
        self._pub_yellow_rel.publish(rel)

    def _publish_overlay(self, orig, small, pts, center_at_lookahead, lookahead_y):
        if not self.is_calib:
            vis = orig.copy()
            cx_img = int(center_at_lookahead)
            cv2.circle(vis, (cx_img, IMG_H // 2), 6, (0, 255, 255), -1)
            self._pub_out.publish(self.cvBridge.cv2_to_compressed_imgmsg(vis, "jpg"))
            return

        vis = small.copy()
        line_pts = []
        for x, y in pts:
            xi = int(np.clip(x, 0, PROC_W - 1))
            yi = int(np.clip(y, 0, PROC_H - 1))
            line_pts.append((xi, yi))
            cv2.circle(vis, (xi, yi), 1, (0, 255, 255), -1)

        if len(line_pts) >= 2:
            cv2.polylines(
                vis,
                [np.array(line_pts, dtype=np.int32)],
                False,
                (0, 255, 255),
                2,
            )

        y0 = max(0, min(PROC_H - 1, int(self.roi_y_min)))
        y1 = max(0, min(PROC_H - 1, int(self.roi_y_max)))
        cv2.rectangle(vis, (0, y0), (PROC_W - 1, y1), (0, 255, 0), 1)

        cx_small = int(center_at_lookahead / SCALE)
        cv2.circle(vis, (cx_small, lookahead_y), 4, (0, 255, 255), -1)
        self._pub_out.publish(self.cvBridge.cv2_to_compressed_imgmsg(vis, "jpg"))


def main(args=None):
    rclpy.init(args=args)
    node = DetectCenterline()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

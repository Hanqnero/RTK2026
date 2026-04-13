#!/usr/bin/env python3
# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# SignDistanceNode — расстояние до знака через depth RealSense.
# Берёт центр bounding box от YOLO (/detect/sign_bbox_center) и
# сэмплирует медианную глубину в окне ROI_PX×ROI_PX вокруг этой точки.
# Если YOLO bbox недоступен — fallback на центр кадра.
#
# Topics:
#   Subscribe:
#     /detect/traffic_sign                       (UInt8)
#     /detect/sign_bbox_center                   (Point)  — bbox center от YOLO
#     /camera/aligned_depth_to_color/image_raw   (Image, 16UC1 мм)
#   Publish:
#     /detect/sign_distance                      (Float32) — метры, -1.0 если нет данных
#
# Parameters:
#   roi_px          (int)   — половина стороны ROI вокруг bbox центра, px (default: 40)
#   min_depth_m     (float) — default: 0.05
#   max_depth_m     (float) — default: 3.0
#   sign_timeout_sec(float) — default: 1.0

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


class SignDistanceNode(Node):
    def __init__(self):
        super().__init__('sign_distance')

        self.declare_parameter('roi_px',           40)
        self.declare_parameter('min_depth_m',      0.05)
        self.declare_parameter('max_depth_m',      3.0)
        self.declare_parameter('sign_timeout_sec', 1.0)

        self._roi = self.get_parameter('roi_px').value
        self._min_d = self.get_parameter('min_depth_m').value
        self._max_d = self.get_parameter('max_depth_m').value
        self._timeout = self.get_parameter('sign_timeout_sec').value

        self._bridge = CvBridge()
        self._sign_id: int = 0
        self._last_sign_t: float = 0.0
        self._bbox_cx: float = -1.0
        self._bbox_cy: float = -1.0
        self._depth_img: np.ndarray | None = None

        self.create_subscription(UInt8,  '/detect/traffic_sign',      self._cb_sign,  10)
        self.create_subscription(Point,  '/detect/sign_bbox_center',  self._cb_bbox,  10)
        self.create_subscription(Image,  '/camera/aligned_depth_to_color/image_raw',
                                 self._cb_depth, 10)

        self._pub = self.create_publisher(Float32, '/detect/sign_distance', 10)

        self.get_logger().info(
            f'sign_distance started. ROI={self._roi*2}x{self._roi*2}px, '
            f'range=[{self._min_d}, {self._max_d}] m')

    def _cb_sign(self, msg: UInt8):
        if msg.data != 0:
            self._sign_id = int(msg.data)
            self._last_sign_t = self.get_clock().now().nanoseconds * 1e-9
        self._publish()

    def _cb_bbox(self, msg: Point):
        self._bbox_cx = msg.x
        self._bbox_cy = msg.y

    def _cb_depth(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                self._depth_img = self._bridge.imgmsg_to_cv2(msg, '32FC1')
            else:
                raw = self._bridge.imgmsg_to_cv2(msg, '16UC1')
                self._depth_img = raw.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().warn(f'depth decode: {e}')
            return
        self._publish()

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_sign_t > self._timeout:
            self._sign_id = 0

        out = Float32()
        if self._sign_id == 0 or self._depth_img is None:
            out.data = -1.0
            self._pub.publish(out)
            return

        h, w = self._depth_img.shape[:2]

        # Используем bbox центр от YOLO если свежий, иначе центр кадра
        if self._bbox_cx >= 0:
            cx, cy = int(self._bbox_cx), int(self._bbox_cy)
        else:
            cx, cy = w // 2, h // 2

        r = self._roi
        y0, y1 = max(0, cy - r), min(h, cy + r)
        x0, x1 = max(0, cx - r), min(w, cx + r)

        roi = self._depth_img[y0:y1, x0:x1]
        valid = roi[(roi >= self._min_d) & (roi <= self._max_d)]

        if valid.size == 0:
            out.data = -1.0
        else:
            dist = float(np.median(valid))
            out.data = dist
            self.get_logger().info(
                f'Sign {self._sign_id}: {dist:.3f} m (bbox={cx},{cy})',
                throttle_duration_sec=0.3)

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SignDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

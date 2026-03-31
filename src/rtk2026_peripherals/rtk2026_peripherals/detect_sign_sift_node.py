#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8

from .sift_sign_detection import build_reference_signs, create_sift, detect_signs


class DetectSignSiftNode(Node):
    def __init__(self):
        super().__init__("detect_sign_sift")
        self.declare_parameter(
            "dataset_root", "/workspace/src/rtk2026_peripherals/sift_dataset"
        )
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("ratio_thresh", 0.75)
        self.declare_parameter("min_good", 6)
        self.declare_parameter("min_inliers", 6)
        self.declare_parameter("min_inlier_ratio", 0.30)
        self.declare_parameter("max_refs_per_class", 3)
        self.declare_parameter("every_n", 2)

        self._bridge = CvBridge()
        self._counter = 0
        self._every_n = int(self.get_parameter("every_n").value)

        dataset_root = Path(self.get_parameter("dataset_root").value)
        self._refs = []
        try:
            sift = create_sift()
            self._refs = build_reference_signs(
                dataset_root=dataset_root,
                sift=sift,
                max_refs_per_class=int(self.get_parameter("max_refs_per_class").value),
            )
            self.get_logger().info(
                f"SIFT sign detector: loaded {len(self._refs)} references from {dataset_root}"
            )
        except Exception as e:
            self.get_logger().warn(
                f"SIFT references are unavailable ({e}). "
                "Node will run in pass-through mode (sign_id=0) until dataset_root is fixed."
            )

        camera_topic = self.get_parameter("camera_topic").value
        self.create_subscription(Image, camera_topic, self._cb, 1)
        self._pub_sign = self.create_publisher(UInt8, "/detect/traffic_sign", 10)
        self._pub_bbox = self.create_publisher(Point, "/detect/sign_bbox_center", 10)
        self._pub_dbg = self.create_publisher(
            CompressedImage, "/detect/image_sign/compressed", 1
        )

    def _cb(self, msg: Image):
        self._counter += 1
        if self._every_n > 1 and self._counter % self._every_n != 0:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = []
        if self._refs:
            detections = detect_signs(
                frame,
                self._refs,
                ratio_thresh=float(self.get_parameter("ratio_thresh").value),
                min_good=int(self.get_parameter("min_good").value),
                min_inliers=int(self.get_parameter("min_inliers").value),
                min_inlier_ratio=float(self.get_parameter("min_inlier_ratio").value),
            )

        sign_id = 0
        best = None
        if detections:
            best = max(detections, key=lambda d: d.score)
            sign_id = int(best.sign_id)

        sign_msg = UInt8()
        sign_msg.data = sign_id
        self._pub_sign.publish(sign_msg)

        if best is not None:
            pts = best.polygon.reshape(-1, 2)
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            bbox_msg = Point()
            bbox_msg.x = cx
            bbox_msg.y = cy
            bbox_msg.z = 0.0
            self._pub_bbox.publish(bbox_msg)

            poly = pts.astype(int)
            cv2.polylines(frame, [poly], True, (0, 255, 0), 2)
            cv2.putText(
                frame,
                f"sign={sign_id} score={best.score:.2f}",
                (max(int(cx) - 80, 10), max(int(cy) - 12, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        self._pub_dbg.publish(self._bridge.cv2_to_compressed_imgmsg(frame, "jpg"))


def main(args=None):
    rclpy.init(args=args)
    node = DetectSignSiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0
#
# YOLO sign detection via ONNX Runtime (no PyTorch on Pi).
# Expects an ONNX export of a YOLOv8 model: model.export(format='onnx', imgsz=640)
#
# Topics:
#   Subscribe:
#     /camera/color/image_raw          (Image)
#   Publish:
#     /detect/traffic_sign             (UInt8)  — sign ID (0=none, 1=intersection, 2=left, 3=right)
#     /detect/sign_bbox_center         (Point)  — bbox center x,y in image pixels (for depth sampling)
#     /detect/image_sign/compressed    (CompressedImage) — debug overlay
#
# Parameters:
#   model_path   (str)   — path to .onnx file (default: ~/models/signs.onnx)
#   conf         (float) — confidence threshold (default: 0.5)
#   every_n      (int)   — run inference every N frames (default: 2)
#   input_size   (int)   — model input size (default: 640)
#
# Sign class mapping (must match your YOLO training label order, edit as needed):
#   class index 0 → sign_id 1 (intersection)
#   class index 1 → sign_id 2 (left)
#   class index 2 → sign_id 3 (right)

import os

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8

try:
    import onnxruntime as ort
    ORT_AVAILABLE = True
except ImportError:
    ORT_AVAILABLE = False


class DetectSignYoloNode(Node):
    def __init__(self):
        super().__init__('detect_sign')

        self.declare_parameter('model_path', os.path.expanduser('~/models/signs.onnx'))
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('every_n', 2)
        self.declare_parameter('input_size', 640)

        model_path = self.get_parameter('model_path').value
        self._conf = self.get_parameter('conf').value
        self._every_n = self.get_parameter('every_n').value
        self._input_size = self.get_parameter('input_size').value

        # class index (from YOLO training) → sign ID published to /detect/traffic_sign
        # Edit to match your training label order
        self._class_to_sign_id = {
            0: 1,  # intersection
            1: 2,  # left
            2: 3,  # right
        }

        self._bridge = CvBridge()
        self._counter = 0
        self._session = None

        if not ORT_AVAILABLE:
            self.get_logger().error('onnxruntime not installed! Run: pip3 install onnxruntime')
        elif not os.path.exists(model_path):
            self.get_logger().warn(
                f'Model not found at {model_path}. '
                'Inference disabled until model is placed there.')
        else:
            self._session = ort.InferenceSession(
                model_path, providers=['CPUExecutionProvider'])
            self._input_name = self._session.get_inputs()[0].name
            self.get_logger().info(
                f'ONNX model loaded: {model_path}, conf={self._conf}, '
                f'input={self._input_size}x{self._input_size}')

        self._sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._cb, 1)
        self._pub_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)
        self._pub_bbox = self.create_publisher(Point, '/detect/sign_bbox_center', 10)
        self._pub_img = self.create_publisher(
            CompressedImage, '/detect/image_sign/compressed', 1)

    def _preprocess(self, img: np.ndarray):
        """Resize to input_size, normalize to [0,1], NCHW float32."""
        sz = self._input_size
        resized = cv2.resize(img, (sz, sz))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        blob = rgb.astype(np.float32) / 255.0
        return np.transpose(blob, (2, 0, 1))[np.newaxis]  # (1, 3, H, W)

    def _postprocess(self, outputs, orig_w: int, orig_h: int):
        """
        YOLOv8 ONNX output shape: (1, num_classes+4, num_anchors).
        Returns (best_sign_id, best_conf, cx, cy) in original image coordinates.
        """
        pred = outputs[0][0]  # (num_classes+4, num_anchors)
        # rows: [x_center, y_center, w, h, cls0_conf, cls1_conf, ...]
        boxes_xywh = pred[:4].T      # (num_anchors, 4)  — normalized to input_size
        class_scores = pred[4:].T    # (num_anchors, num_classes)

        best_sign_id = 0
        best_conf = float(self._conf)
        best_cx_orig = orig_w / 2.0
        best_cy_orig = orig_h / 2.0
        best_box = None

        for i in range(class_scores.shape[0]):
            cls_idx = int(np.argmax(class_scores[i]))
            conf = float(class_scores[i, cls_idx])
            if conf < self._conf:
                continue
            sign_id = self._class_to_sign_id.get(cls_idx, 0)
            if sign_id == 0:
                continue
            if conf > best_conf:
                best_conf = conf
                best_sign_id = sign_id
                sz = self._input_size
                cx = boxes_xywh[i, 0] / sz * orig_w
                cy = boxes_xywh[i, 1] / sz * orig_h
                w  = boxes_xywh[i, 2] / sz * orig_w
                h  = boxes_xywh[i, 3] / sz * orig_h
                best_cx_orig = cx
                best_cy_orig = cy
                best_box = (int(cx - w/2), int(cy - h/2),
                            int(cx + w/2), int(cy + h/2))

        return best_sign_id, best_conf, best_cx_orig, best_cy_orig, best_box

    def _cb(self, msg: Image):
        self._counter += 1
        if self._counter % self._every_n != 0:
            return
        if self._session is None:
            return

        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        blob = self._preprocess(img)
        outputs = self._session.run(None, {self._input_name: blob})

        sign_id, conf, cx, cy, box = self._postprocess(
            outputs, img.shape[1], img.shape[0])

        sign_msg = UInt8()
        sign_msg.data = sign_id
        self._pub_sign.publish(sign_msg)

        if sign_id != 0:
            pt = Point()
            pt.x = cx
            pt.y = cy
            self._pub_bbox.publish(pt)

            if box is not None:
                x1, y1, x2, y2 = box
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'sign{sign_id} {conf:.2f}'
                cv2.putText(img, label, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self._pub_img.publish(self._bridge.cv2_to_compressed_imgmsg(img, 'jpg'))


def main(args=None):
    rclpy.init(args=args)
    node = DetectSignYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

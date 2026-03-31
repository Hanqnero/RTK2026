import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rtk2026_interfaces.msg import SignDetection
from .sift_sign_detection import (
    build_reference_signs,
    detect_signs,
    create_sift_detector,
)
from pathlib import Path


class SignDetectorNode(Node):
    def __init__(self):
        super().__init__("sign_detector")
        self.declare_parameter("dataset_root", "/path/to/yolo/dataset")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("ratio_thresh", 0.75)
        self.declare_parameter("min_good", 6)
        self.declare_parameter("min_inliers", 6)
        self.declare_parameter("min_inlier_ratio", 0.3)
        self.declare_parameter("min_ncc", 0.1)
        self.declare_parameter("nms_thresh", 0.4)
        self.declare_parameter("max_refs_per_class", 3)

        dataset_root = Path(self.get_parameter("dataset_root").value)
        sift = create_sift_detector()
        self.references = build_reference_signs(
            dataset_root,
            sift,
            max_refs_per_class=self.get_parameter("max_refs_per_class").value,
        )
        self.get_logger().info(
            f"Loaded {len(self.references)} reference crops from {dataset_root}"
        )
        self.bridge = CvBridge()

        self.pub = self.create_publisher(SignDetection, "/sign_detections", 10)
        self.sub = self.create_subscription(
            Image,
            self.get_parameter("camera_topic").value,
            self.image_cb,
            10,
        )

    def image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detections = detect_signs(
            frame,
            self.references,
            ratio_thresh=self.get_parameter("ratio_thresh").value,
            min_good=self.get_parameter("min_good").value,
            min_inliers=self.get_parameter("min_inliers").value,
            min_inlier_ratio=self.get_parameter("min_inlier_ratio").value,
            min_ncc=self.get_parameter("min_ncc").value,
            nms_thresh=self.get_parameter("nms_thresh").value,
        )

        out = SignDetection()
        out.header = msg.header
        for d in detections:
            out.class_names.append(d.class_name)
            out.class_ids.append(d.class_id)
            out.scores.append(float(d.score))
            pts = d.polygon.reshape(-1, 2)
            out.bbox_x1.append(int(pts[:, 0].min()))
            out.bbox_y1.append(int(pts[:, 1].min()))
            out.bbox_x2.append(int(pts[:, 0].max()))
            out.bbox_y2.append(int(pts[:, 1].max()))

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

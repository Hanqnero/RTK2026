import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rtk2026_interfaces.msg import SignDetection

try:
    import message_filters
    _HAS_MESSAGE_FILTERS = True
except ImportError:
    _HAS_MESSAGE_FILTERS = False

from .yolo_sign_detection import detect, load_model


class SignDetectorNode(Node):
    def __init__(self):
        super().__init__("sign_detector")

        self.declare_parameter("model_path", "")
        self.declare_parameter("class_names", rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "")          # empty = depth disabled
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("conf_thresh", 0.40)
        self.declare_parameter("iou_thresh", 0.45)
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("max_depth", 15.0)
        self.declare_parameter("num_threads", 4)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            self.get_logger().error("model_path parameter is required")
            raise RuntimeError("model_path not set")

        self.session, self.input_name = load_model(
            model_path,
            num_threads=self.get_parameter("num_threads").value,
        )
        self.class_names: list[str] = self.get_parameter("class_names").value
        self.get_logger().info(
            f"Loaded ONNX model: {model_path}  "
            f"({len(self.class_names)} classes, imgsz={self.get_parameter('imgsz').value})"
        )

        self.bridge = CvBridge()
        self.pub = self.create_publisher(SignDetection, "/sign_detections", 10)

        depth_topic: str = self.get_parameter("depth_topic").value
        color_topic: str = self.get_parameter("camera_topic").value

        if depth_topic and _HAS_MESSAGE_FILTERS:
            self.get_logger().info(f"Depth gating enabled: {depth_topic}")
            self._color_sub = message_filters.Subscriber(self, Image, color_topic)
            self._depth_sub = message_filters.Subscriber(self, Image, depth_topic)
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [self._color_sub, self._depth_sub],
                queue_size=5,
                slop=0.05,
            )
            self._sync.registerCallback(self._synced_cb)
        else:
            if depth_topic and not _HAS_MESSAGE_FILTERS:
                self.get_logger().warning(
                    "message_filters not available — depth gating disabled"
                )
            self.create_subscription(Image, color_topic, self._color_only_cb, 10)

    # ------------------------------------------------------------------
    def _color_only_cb(self, msg: Image) -> None:
        self._run(msg, depth_msg=None)

    def _synced_cb(self, color_msg: Image, depth_msg: Image) -> None:
        self._run(color_msg, depth_msg=depth_msg)

    def _run(self, color_msg: Image, depth_msg: Image | None) -> None:
        frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")

        depth_frame = None
        if depth_msg is not None:
            enc = depth_msg.encoding
            if enc == "16UC1":
                depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            else:
                depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        detections = detect(
            self.session,
            self.input_name,
            frame,
            self.class_names,
            imgsz=self.get_parameter("imgsz").value,
            conf_thresh=self.get_parameter("conf_thresh").value,
            iou_thresh=self.get_parameter("iou_thresh").value,
            depth_frame=depth_frame,
            min_depth=self.get_parameter("min_depth").value,
            max_depth=self.get_parameter("max_depth").value,
        )

        out = SignDetection()
        out.header = color_msg.header
        for d in detections:
            out.class_names.append(d.class_name)
            out.class_ids.append(d.class_id)
            out.scores.append(float(d.score))
            out.bbox_x1.append(d.x1)
            out.bbox_y1.append(d.y1)
            out.bbox_x2.append(d.x2)
            out.bbox_y2.append(d.y2)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

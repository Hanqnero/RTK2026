#!/usr/bin/env python3
# Small debug window to visualize:
# - input bird's-eye image used by detect_lane
# - output overlay image from detect_lane
# - numeric center value from /detect/lane
#
# Designed for calibration: run it alongside detect_lane (optionally in calibration mode).

import threading
from dataclasses import dataclass

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float64, UInt8


@dataclass
class Latest:
    img_input: any = None
    img_output: any = None
    center: float | None = None
    lane_state: int | None = None
    lock: threading.Lock = threading.Lock()


class LaneDebugWindow(Node):
    def __init__(
        self,
        input_topic: str = "/camera/image_projected",
        output_topic: str = "/detect/image_output/compressed",
        lane_topic: str = "/detect/lane",
        lane_state_topic: str = "/detect/lane_state",
    ):
        super().__init__("lane_debug_window")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        # Allow overriding topics via ROS params (useful during calibration).
        self.declare_parameter("input_topic", input_topic)
        self.declare_parameter("output_topic", output_topic)
        self.declare_parameter("lane_topic", lane_topic)
        self.declare_parameter("lane_state_topic", lane_state_topic)

        self.bridge = CvBridge()
        self.latest = Latest()

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.lane_topic = self.get_parameter("lane_topic").value
        self.lane_state_topic = self.get_parameter("lane_state_topic").value

        self.create_subscription(Image, self.input_topic, self.cb_input, qos)
        self.create_subscription(CompressedImage, self.output_topic, self.cb_output, qos)
        self.create_subscription(Float64, self.lane_topic, self.cb_center, qos)
        self.create_subscription(UInt8, self.lane_state_topic, self.cb_lane_state, qos)

    def cb_input(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cb_input cv_bridge failed: {e}")
            return
        with self.latest.lock:
            self.latest.img_input = img

    def cb_output(self, msg: CompressedImage):
        np_arr = __import__("numpy").frombuffer(msg.data, dtype=__import__("numpy").uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return
        with self.latest.lock:
            self.latest.img_output = img

    def cb_center(self, msg: Float64):
        with self.latest.lock:
            self.latest.center = float(msg.data)

    def cb_lane_state(self, msg: UInt8):
        with self.latest.lock:
            self.latest.lane_state = int(msg.data)

    def spin_once_with_gui(self):
        # Called in a loop: allow OpenCV window to refresh.
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.03)
            with self.latest.lock:
                img_in = self.latest.img_input
                img_out = self.latest.img_output
                center = self.latest.center
                lane_state = self.latest.lane_state

            if img_in is None or img_out is None:
                # Still show what we have (at least black placeholder).
                h = 480
                w = 600
                canvas = __import__("numpy").zeros((h, w * 2, 3), dtype=__import__("numpy").uint8)
                cv2.putText(canvas, "waiting for images...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow("Lane Debug (input + detect output)", canvas)
                key = cv2.waitKey(1)
                if key == ord("q"):
                    return
                continue

            # Make both images same height.
            target_h = max(img_in.shape[0], img_out.shape[0])
            if img_in.shape[0] != target_h:
                img_in = cv2.resize(img_in, (int(img_in.shape[1] * target_h / img_in.shape[0]), target_h))
            if img_out.shape[0] != target_h:
                img_out = cv2.resize(img_out, (int(img_out.shape[1] * target_h / img_out.shape[0]), target_h))

            canvas = cv2.hconcat([img_in, img_out])

            overlay = f"center={center:.2f}  lane_state={lane_state}" if center is not None else f"lane_state={lane_state}"
            cv2.putText(
                canvas,
                overlay,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("Lane Debug (input + detect output)", canvas)
            key = cv2.waitKey(1)
            if key == ord("q"):
                return


def main():
    rclpy.init()
    node = LaneDebugWindow()
    try:
        node.spin_once_with_gui()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


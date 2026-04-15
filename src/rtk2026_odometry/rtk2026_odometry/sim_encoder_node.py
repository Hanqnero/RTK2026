"""
ROS2 нода-адаптер для симуляции: публикует EncoderReport из JointState.

Нужна, чтобы в Gazebo проверять ту же wheel_odometry_node, что используется
на роботе: Gazebo даёт углы колёс, эта нода переводит их в тики энкодеров.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from rtk2026_interfaces.msg import EncoderReport


class SimEncoderNode(Node):

    def __init__(self) -> None:
        super().__init__("sim_encoder")

        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("left_joint_name", "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("ticks_per_meter", 1268.0)
        self.declare_parameter("wheel_radius", 0.02585)
        self.declare_parameter("left_sign", 1)
        self.declare_parameter("right_sign", 1)

        self._left_joint = self.get_parameter("left_joint_name").value
        self._right_joint = self.get_parameter("right_joint_name").value
        self._ticks_per_meter = float(self.get_parameter("ticks_per_meter").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._left_sign = int(self.get_parameter("left_sign").value)
        self._right_sign = int(self.get_parameter("right_sign").value)
        self._warned_missing_joints = False

        enc_topic = self.get_parameter("encoder_report_topic").value
        self._encoder_pub = self.create_publisher(EncoderReport, enc_topic, 10)

        joint_topic = self.get_parameter("joint_states_topic").value
        self.create_subscription(
            JointState,
            joint_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Sim encoder started: "
            f"{self._left_joint}/{self._right_joint}, "
            f"ticks_per_meter={self._ticks_per_meter}, "
            f"wheel_radius={self._wheel_radius}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Прочитать углы колёс из JointState и опубликовать EncoderReport."""
        positions = dict(zip(msg.name, msg.position))
        if self._left_joint not in positions or self._right_joint not in positions:
            if not self._warned_missing_joints:
                self.get_logger().warn(
                    f"JointState has no {self._left_joint}/{self._right_joint}; "
                    f"available joints: {list(msg.name)}"
                )
                self._warned_missing_joints = True
            return

        report = EncoderReport()
        report.header.stamp = msg.header.stamp
        if report.header.stamp.sec == 0 and report.header.stamp.nanosec == 0:
            report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = "base_link"
        report.left_count = self._angle_to_ticks(
            positions[self._left_joint],
            self._left_sign,
        )
        report.right_count = self._angle_to_ticks(
            positions[self._right_joint],
            self._right_sign,
        )
        self._encoder_pub.publish(report)

    def _angle_to_ticks(self, angle_rad: float, sign: int) -> int:
        """Перевести угол вращения колеса в накопленные тики энкодера."""
        if not math.isfinite(angle_rad):
            return 0
        distance = angle_rad * self._wheel_radius
        return int(round(sign * distance * self._ticks_per_meter))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimEncoderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

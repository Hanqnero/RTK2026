#!/usr/bin/env python3
"""Квантование симулируемых углов под разрешение реального энкодера.

Input:  /joint_states
Output: /encoder_joint_states

Use this node between Gazebo and a custom wheel-odometry implementation when
encoder quantization matters. It keeps wheel angles unwrapped and computes
velocity from quantized position increments.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass
class WheelSample:
    """Предыдущее квантованное положение колеса и его timestamp."""

    position: float
    stamp_seconds: float


class QuantizeJointStates(Node):
    """Преобразовать два wheel joint из ``/joint_states`` в дискретные ticks."""

    def __init__(self) -> None:
        """Прочитать модель энкодера и создать subscriber/publisher."""

        super().__init__("quantize_joint_states")

        self.declare_parameter("left_joint", "left_wheel_joint")
        self.declare_parameter("right_joint", "right_wheel_joint")
        self.declare_parameter("counts_per_motor_revolution", 11.0)
        self.declare_parameter("quadrature_factor", 4.0)
        self.declare_parameter("gear_ratio", 30.0)
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/encoder_joint_states")

        self._joint_names = [
            str(self.get_parameter("left_joint").value),
            str(self.get_parameter("right_joint").value),
        ]
        counts = float(self.get_parameter("counts_per_motor_revolution").value)
        quadrature = float(self.get_parameter("quadrature_factor").value)
        gear_ratio = float(self.get_parameter("gear_ratio").value)

        ticks_per_output_rev = counts * quadrature * gear_ratio
        if not math.isfinite(ticks_per_output_rev) or ticks_per_output_rev <= 0.0:
            raise ValueError("counts * quadrature_factor * gear_ratio must be > 0")
        self._radians_per_tick = 2.0 * math.pi / ticks_per_output_rev

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(JointState, output_topic, 20)
        self._subscription = self.create_subscription(
            JointState, input_topic, self._on_joint_state, 20
        )
        self._previous: dict[str, WheelSample] = {}

        self.get_logger().info(
            f"ticks/output-rev={ticks_per_output_rev:.3f}, "
            f"rad/tick={self._radians_per_tick:.9f}"
        )

    @staticmethod
    def _stamp_seconds(msg: JointState) -> float:
        """Преобразовать ROS sec/nanosec в секунды float."""

        return float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)

    def _on_joint_state(self, msg: JointState) -> None:
        """Квантовать позиции и вычислить скорость по дискретной разности."""

        index = {name: i for i, name in enumerate(msg.name)}
        if any(name not in index for name in self._joint_names):
            return

        output = JointState()
        output.header = msg.header
        output.name = list(self._joint_names)
        stamp = self._stamp_seconds(msg)

        for name in self._joint_names:
            i = index[name]
            if i >= len(msg.position):
                self.get_logger().error(f"No position value for joint {name}")
                return

            raw_position = float(msg.position[i])
            tick_count = round(raw_position / self._radians_per_tick)
            quantized_position = tick_count * self._radians_per_tick
            output.position.append(quantized_position)

            previous = self._previous.get(name)
            if previous is None or stamp <= previous.stamp_seconds:
                velocity = 0.0
            else:
                velocity = (
                    quantized_position - previous.position
                ) / (stamp - previous.stamp_seconds)
            output.velocity.append(velocity)
            self._previous[name] = WheelSample(quantized_position, stamp)

        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    """Запустить диагностическую ROS-ноду до штатного завершения."""

    rclpy.init(args=args)
    node = QuantizeJointStates()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

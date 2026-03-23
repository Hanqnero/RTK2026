#!/usr/bin/env python3
# Публикует /control/max_vel для control_lane (TurtleBot3 Autorace), чтобы снизить
# скорость и уменьшить пересечение линий при езде по полосе.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class AutoraceMaxVelPublisher(Node):
    def __init__(self):
        super().__init__("autorace_max_vel_publisher")
        self.declare_parameter("max_vel", 0.032)
        self.declare_parameter("publish_interval_sec", 0.5)
        max_vel = self.get_parameter("max_vel").value
        interval = self.get_parameter("publish_interval_sec").value
        self._pub = self.create_publisher(Float64, "/control/max_vel", 10)
        self._msg = Float64()
        self._msg.data = max_vel
        self._timer = self.create_timer(interval, self._publish)

    def _publish(self):
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoraceMaxVelPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

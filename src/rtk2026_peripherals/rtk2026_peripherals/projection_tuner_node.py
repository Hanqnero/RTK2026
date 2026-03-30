# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String


PARAM_NAMES = [
    "camera.extrinsic_camera_calibration.top_x",
    "camera.extrinsic_camera_calibration.top_y",
    "camera.extrinsic_camera_calibration.bottom_x",
    "camera.extrinsic_camera_calibration.bottom_y",
]

LIMITS = {
    PARAM_NAMES[0]: (0, 120),
    PARAM_NAMES[1]: (0, 120),
    PARAM_NAMES[2]: (0, 320),
    PARAM_NAMES[3]: (0, 320),
}

DEFAULTS = [72, 4, 259, 159]


def clamp(name: str, value: int) -> int:
    low, high = LIMITS[name]
    return max(low, min(high, int(value)))


class ProjectionTunerNode(Node):
    def __init__(self):
        super().__init__("projection_tuner")

        self.declare_parameter("target_node", "/image_projection_calib")
        self.declare_parameter("set_topic", "/camera/extrinsic_tuning/set")
        self.declare_parameter("current_topic", "/camera/extrinsic_tuning/current")
        self.declare_parameter("status_topic", "/camera/extrinsic_tuning/status")

        self._target_node = self.get_parameter("target_node").value
        self._set_topic = self.get_parameter("set_topic").value
        self._current_topic = self.get_parameter("current_topic").value
        self._status_topic = self.get_parameter("status_topic").value

        self._get_cli = self.create_client(
            GetParameters, f"{self._target_node}/get_parameters"
        )
        self._set_cli = self.create_client(
            SetParameters, f"{self._target_node}/set_parameters"
        )

        self._current_pub = self.create_publisher(Int32MultiArray, self._current_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._set_sub = self.create_subscription(
            Int32MultiArray, self._set_topic, self._set_cb, 10
        )

        self._current_values = list(DEFAULTS)
        self._pending_get = None
        self._pending_set = None

        self.create_timer(0.5, self._timer_cb)
        self._publish_status(
            "projection_tuner ready: publish [top_x, top_y, bottom_x, bottom_y] "
            f"to {self._set_topic}"
        )

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_current(self) -> None:
        self._current_pub.publish(Int32MultiArray(data=list(self._current_values)))

    def _timer_cb(self) -> None:
        self._publish_current()
        if self._pending_get is not None or not self._get_cli.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = list(PARAM_NAMES)
        self._pending_get = self._get_cli.call_async(req)
        self._pending_get.add_done_callback(self._handle_get)

    def _handle_get(self, future) -> None:
        self._pending_get = None
        try:
            resp = future.result()
        except Exception as exc:
            self._publish_status(f"get_parameters failed: {exc}")
            return

        values = []
        for idx, value_msg in enumerate(resp.values):
            if value_msg.type == ParameterType.PARAMETER_INTEGER:
                values.append(int(value_msg.integer_value))
            else:
                values.append(self._current_values[idx])
        self._current_values = values
        self._publish_current()

    def _set_cb(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != 4:
            self._publish_status(
                "invalid tuning message: expected 4 ints "
                "[top_x, top_y, bottom_x, bottom_y]"
            )
            return
        if not self._set_cli.service_is_ready():
            self._publish_status(f"target node not ready: {self._target_node}")
            return
        if self._pending_set is not None:
            self._publish_status("set ignored: previous update still in flight")
            return

        values = [
            clamp(PARAM_NAMES[0], msg.data[0]),
            clamp(PARAM_NAMES[1], msg.data[1]),
            clamp(PARAM_NAMES[2], msg.data[2]),
            clamp(PARAM_NAMES[3], msg.data[3]),
        ]

        req = SetParameters.Request()
        req.parameters = []
        for name, value in zip(PARAM_NAMES, values):
            param = ParameterMsg()
            param.name = name
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = int(value)
            req.parameters.append(param)

        self._pending_set = self._set_cli.call_async(req)
        self._pending_set.add_done_callback(
            lambda future, values=values: self._handle_set(future, values)
        )

    def _handle_set(self, future, values) -> None:
        self._pending_set = None
        try:
            resp = future.result()
        except Exception as exc:
            self._publish_status(f"set_parameters failed: {exc}")
            return

        if not all(result.successful for result in resp.results):
            reasons = [result.reason for result in resp.results if result.reason]
            self._publish_status(
                "set_parameters rejected"
                + (f": {'; '.join(reasons)}" if reasons else "")
            )
            return

        self._current_values = list(values)
        self._publish_current()
        self._publish_status(
            "updated trapezoid: "
            f"top_x={values[0]} top_y={values[1]} "
            f"bottom_x={values[2]} bottom_y={values[3]}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ProjectionTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

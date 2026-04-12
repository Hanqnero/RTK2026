# Copyright 2026 RTK2026
# SPDX-License-Identifier: Apache-2.0

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


PARAM_NAMES = [
    "camera_height_m",
    "camera_pitch_rad",
    "y_near_m",
    "y_far_m",
    "x_left_m",
    "x_right_m",
]

LIMITS = {
    "camera_height_m": (0.01, 1.0),
    "camera_pitch_rad": (-1.5708, 1.5708),
    "y_near_m": (0.01, 1.0),
    "y_far_m": (0.1, 5.0),
    "x_left_m": (-3.0, 0.0),
    "x_right_m": (0.0, 3.0),
}

DEFAULTS = [0.11, 0.35, 0.05, 0.40, -0.22, 0.22]


def clamp(name: str, value: float) -> float:
    low, high = LIMITS[name]
    return max(low, min(high, float(value)))


class IpmTunerNode(Node):
    def __init__(self):
        super().__init__("ipm_tuner")

        self.declare_parameter("target_node", "/image_relay_autorace")
        self.declare_parameter("set_topic", "/camera/ipm_tuning/set")
        self.declare_parameter("current_topic", "/camera/ipm_tuning/current")
        self.declare_parameter("status_topic", "/camera/ipm_tuning/status")

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

        self._current_pub = self.create_publisher(Float32MultiArray, self._current_topic, 10)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._set_sub = self.create_subscription(
            Float32MultiArray, self._set_topic, self._set_cb, 10
        )

        self._current_values = list(DEFAULTS)
        self._pending_get = None
        self._pending_set = None

        self.create_timer(0.5, self._timer_cb)
        self._publish_status(
            "ipm_tuner ready: publish "
            "[camera_height_m, camera_pitch_rad, y_near_m, y_far_m, x_left_m, x_right_m] "
            f"to {self._set_topic}"
        )

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_current(self) -> None:
        self._current_pub.publish(Float32MultiArray(data=list(self._current_values)))

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
            if value_msg.type == ParameterType.PARAMETER_DOUBLE:
                values.append(float(value_msg.double_value))
            else:
                values.append(self._current_values[idx])
        self._current_values = values
        self._publish_current()

    def _set_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) != 6:
            self._publish_status(
                "invalid tuning message: expected 6 floats "
                "[camera_height_m, camera_pitch_rad, y_near_m, y_far_m, x_left_m, x_right_m]"
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
            clamp(PARAM_NAMES[4], msg.data[4]),
            clamp(PARAM_NAMES[5], msg.data[5]),
        ]

        req = SetParameters.Request()
        req.parameters = []
        for name, value in zip(PARAM_NAMES, values):
            param = ParameterMsg()
            param.name = name
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(value)
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
            "updated ipm: "
            f"h={values[0]:.3f} pitch={values[1]:.3f} "
            f"near={values[2]:.3f} far={values[3]:.3f} "
            f"left={values[4]:.3f} right={values[5]:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IpmTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

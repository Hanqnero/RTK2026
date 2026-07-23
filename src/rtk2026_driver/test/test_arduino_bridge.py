import importlib
import math
import sys
import types
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
DRIVER_SRC = REPO_ROOT / "src" / "rtk2026_driver"

if str(DRIVER_SRC) not in sys.path:
    sys.path.insert(0, str(DRIVER_SRC))


BRIDGE_MODULE = "rtk2026_driver.arduino_bridge"


class _FakeTime:
    def __init__(self, nanoseconds: int):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return {"nanoseconds": self.nanoseconds}


class _FakeClock:
    def __init__(self, nanoseconds: int):
        self._nanoseconds = nanoseconds

    def now(self):
        return _FakeTime(self._nanoseconds)


class _FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []
        self.fatal_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)

    def fatal(self, message):
        self.fatal_messages.append(message)


class _FakeNode:
    def destroy_node(self):
        self._base_destroy_called = True


class _FakeTwist:
    def __init__(self):
        self.linear = types.SimpleNamespace(
            x=0.0,
            y=0.0,
            z=0.0,
        )

        self.angular = types.SimpleNamespace(
            x=0.0,
            y=0.0,
            z=0.0,
        )


class _FakeTwistStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(
            stamp=None,
            frame_id="",
        )
        self.twist = _FakeTwist()


class _FakeTransformStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(
            stamp=None,
            frame_id="",
        )

        self.child_frame_id = ""

        self.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(
                x=0.0,
                y=0.0,
                z=0.0,
            ),
            rotation=types.SimpleNamespace(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            ),
        )


class _FakeOdometry:
    def __init__(self):
        self.header = types.SimpleNamespace(
            stamp=None,
            frame_id="",
        )

        self.child_frame_id = ""

        self.pose = types.SimpleNamespace(
            covariance=[0.0] * 36,
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                ),
                orientation=types.SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0,
                ),
            ),
        )

        self.twist = types.SimpleNamespace(
            covariance=[0.0] * 36,
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                ),
                angular=types.SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                ),
            ),
        )


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FakeTransformBroadcaster:
    def __init__(self, node=None):
        self.node = node
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class _FakeTransport:
    def __init__(self):
        self.writes = []
        self.read_bytes = b""
        self.closed = False

    def write(self, payload: bytes):
        self.writes.append(payload)

    def read_available(self, max_bytes: int = 256):
        result = self.read_bytes
        self.read_bytes = b""
        return result

    def close(self):
        self.closed = True


class _FakeTelemetryPacket:
    def __init__(
        self,
        odom_x_m=0.0,
        odom_y_m=0.0,
        odom_heading_rad=0.0,
        raw_left_encoder_delta=0,
        raw_right_encoder_delta=0,
        left_pwm=0,
        right_pwm=0,
        current_linear_mps=0.0,
        current_angular_rps=0.0,
    ):
        self.odom_x_m = odom_x_m
        self.odom_y_m = odom_y_m
        self.odom_heading_rad = odom_heading_rad

        self.raw_left_encoder_delta = raw_left_encoder_delta
        self.raw_right_encoder_delta = raw_right_encoder_delta

        self.left_pwm = left_pwm
        self.right_pwm = right_pwm

        self.current_linear_mps = current_linear_mps
        self.current_angular_rps = current_angular_rps


def _register_package(monkeypatch, package_name, child_name, child_module):
    package = types.ModuleType(package_name)
    package.__path__ = []
    setattr(package, child_name, child_module)

    monkeypatch.setitem(
        sys.modules,
        package_name,
        package,
    )

    monkeypatch.setitem(
        sys.modules,
        f"{package_name}.{child_name}",
        child_module,
    )


def _load_bridge_module(monkeypatch):
    rclpy_state = {
        "shutdown_calls": 0,
    }

    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.__path__ = []

    rclpy_mod.init = lambda *args, **kwargs: None
    rclpy_mod.spin = lambda *args, **kwargs: None
    rclpy_mod.ok = lambda: True

    def _shutdown():
        rclpy_state["shutdown_calls"] += 1

    rclpy_mod.shutdown = _shutdown

    monkeypatch.setitem(
        sys.modules,
        "rclpy",
        rclpy_mod,
    )

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _FakeNode

    monkeypatch.setitem(
        sys.modules,
        "rclpy.node",
        node_mod,
    )

    executors_mod = types.ModuleType("rclpy.executors")

    class _FakeExternalShutdownException(Exception):
        pass

    executors_mod.ExternalShutdownException = (
        _FakeExternalShutdownException
    )

    monkeypatch.setitem(
        sys.modules,
        "rclpy.executors",
        executors_mod,
    )

    geometry_msg_mod = types.ModuleType("geometry_msgs.msg")
    geometry_msg_mod.TransformStamped = _FakeTransformStamped
    geometry_msg_mod.TwistStamped = _FakeTwistStamped

    _register_package(
        monkeypatch,
        "geometry_msgs",
        "msg",
        geometry_msg_mod,
    )

    nav_msg_mod = types.ModuleType("nav_msgs.msg")
    nav_msg_mod.Odometry = _FakeOdometry

    _register_package(
        monkeypatch,
        "nav_msgs",
        "msg",
        nav_msg_mod,
    )

    tf2_ros_mod = types.ModuleType("tf2_ros")
    tf2_ros_mod.TransformBroadcaster = _FakeTransformBroadcaster

    monkeypatch.setitem(
        sys.modules,
        "tf2_ros",
        tf2_ros_mod,
    )

    serial_mod = types.ModuleType("serial")

    class _FakeSerialException(Exception):
        pass

    serial_mod.SerialException = _FakeSerialException

    monkeypatch.setitem(
        sys.modules,
        "serial",
        serial_mod,
    )

    protocol_mod = types.ModuleType("rtk2026_driver.protocol")
    protocol_mod.TelemetryPacket = _FakeTelemetryPacket
    protocol_mod.pack_calls = []

    def _pack_command(
        linear_mps,
        angular_rps,
        debug_raw_encoder,
    ):
        protocol_mod.pack_calls.append(
            (
                linear_mps,
                angular_rps,
                debug_raw_encoder,
            )
        )

        return (
            f"{linear_mps:.3f},"
            f"{angular_rps:.3f},"
            f"{int(debug_raw_encoder)}"
        ).encode()

    protocol_mod.pack_command = _pack_command
    protocol_mod.pop_telemetry_packet = lambda buffer: None

    monkeypatch.setitem(
        sys.modules,
        "rtk2026_driver.protocol",
        protocol_mod,
    )

    transport_mod = types.ModuleType(
        "rtk2026_driver.serial_transport"
    )
    transport_mod.SerialTransport = _FakeTransport

    monkeypatch.setitem(
        sys.modules,
        "rtk2026_driver.serial_transport",
        transport_mod,
    )

    sys.modules.pop(BRIDGE_MODULE, None)

    module = importlib.import_module(BRIDGE_MODULE)
    module._test_rclpy_state = rclpy_state
    module._test_protocol_module = protocol_mod
    module._test_serial_exception = _FakeSerialException

    return module

# проверяет сохранение linear.x, angular.z и времени команды
def test_on_cmd_vel_saves_latest_command(monkeypatch):
    module = _load_bridge_module(monkeypatch)

    monkeypatch.setattr(
        module.time,
        "monotonic",
        lambda: 12.5,
    )

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._max_linear_mps = 1.5
    node._max_angular_rps = math.pi / 2.0
    node._target_linear_mps = 0.0
    node._target_angular_rps = 0.0
    node._last_cmd_time = None
    node.get_logger = lambda: _FakeLogger()

    message = _FakeTwistStamped()
    message.twist.linear.x = 0.42
    message.twist.angular.z = -0.17

    node._on_cmd_vel(message)

    assert node._target_linear_mps == pytest.approx(0.42)
    assert node._target_angular_rps == pytest.approx(-0.17)
    assert node._last_cmd_time == pytest.approx(12.5)

# проверяет ограничение команды заданными пределами
def test_on_cmd_vel_limits_command(monkeypatch):
    module = _load_bridge_module(monkeypatch)

    monkeypatch.setattr(
        module.time,
        "monotonic",
        lambda: 20.0,
    )

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._max_linear_mps = 1.5
    node._max_angular_rps = 1.0
    node._last_cmd_time = None
    node.get_logger = lambda: _FakeLogger()

    message = _FakeTwistStamped()
    message.twist.linear.x = 4.0
    message.twist.angular.z = -3.0

    node._on_cmd_vel(message)

    assert node._target_linear_mps == pytest.approx(1.5)
    assert node._target_angular_rps == pytest.approx(-1.0)
    assert node._last_cmd_time == pytest.approx(20.0)


@pytest.mark.parametrize(
    ("linear_mps", "angular_rps"),
    [
        (math.nan, 0.0),
        (math.inf, 0.0),
        (0.0, math.nan),
        (0.0, -math.inf),
    ],
)
# проверяет отклонение NaN и бесконечности
def test_on_cmd_vel_rejects_non_finite_values(
    monkeypatch,
    linear_mps,
    angular_rps,
):
    module = _load_bridge_module(monkeypatch)

    logger = _FakeLogger()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._max_linear_mps = 1.5
    node._max_angular_rps = 1.5

    node._target_linear_mps = 0.25
    node._target_angular_rps = -0.10
    node._last_cmd_time = 5.0

    node.get_logger = lambda: logger

    message = _FakeTwistStamped()
    message.twist.linear.x = linear_mps
    message.twist.angular.z = angular_rps

    node._on_cmd_vel(message)

    assert node._target_linear_mps == pytest.approx(0.25)
    assert node._target_angular_rps == pytest.approx(-0.10)
    assert node._last_cmd_time == pytest.approx(5.0)
    assert len(logger.error_messages) == 1

# проверяет отправку актуальной команды.
def test_send_command_writes_fresh_command(monkeypatch):
    module = _load_bridge_module(monkeypatch)

    monkeypatch.setattr(
        module.time,
        "monotonic",
        lambda: 10.2,
    )

    transport = _FakeTransport()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._transport = transport
    node._target_linear_mps = 0.12
    node._target_angular_rps = -0.34
    node._last_cmd_time = 10.0
    node._drop_stale_cmd_after_sec = 0.30
    node._debug_raw_encoder = True

    node._send_command()

    assert module._test_protocol_module.pack_calls == [
        (0.12, -0.34, True)
    ]

    assert transport.writes == [
        b"0.120,-0.340,1"
    ]

# проверяет dead-man после тайм-аута
def test_send_command_writes_zero_when_command_is_stale(
    monkeypatch,
):
    module = _load_bridge_module(monkeypatch)

    monkeypatch.setattr(
        module.time,
        "monotonic",
        lambda: 11.0,
    )

    transport = _FakeTransport()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._transport = transport
    node._target_linear_mps = 0.8
    node._target_angular_rps = 0.4
    node._last_cmd_time = 10.0
    node._drop_stale_cmd_after_sec = 0.30
    node._debug_raw_encoder = False

    node._send_command()

    assert module._test_protocol_module.pack_calls == [
        (0.0, 0.0, False)
    ]

    assert transport.writes == [
        b"0.000,0.000,0"
    ]

# проверяет остановку до получения первой команды
def test_send_command_writes_zero_before_first_cmd_vel(
    monkeypatch,
):
    module = _load_bridge_module(monkeypatch)

    monkeypatch.setattr(
        module.time,
        "monotonic",
        lambda: 15.0,
    )

    transport = _FakeTransport()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._transport = transport
    node._target_linear_mps = 0.8
    node._target_angular_rps = 0.4
    node._last_cmd_time = None
    node._drop_stale_cmd_after_sec = 0.30
    node._debug_raw_encoder = False

    node._send_command()

    assert module._test_protocol_module.pack_calls == [
        (0.0, 0.0, False)
    ]

# проверяет добавление байтов в буфер и обработку нескольких пакетов
def test_read_telemetry_publishes_all_received_packets(
    monkeypatch,
):
    module = _load_bridge_module(monkeypatch)

    first_packet = _FakeTelemetryPacket(
        odom_x_m=1.0,
    )

    second_packet = _FakeTelemetryPacket(
        odom_x_m=2.0,
    )

    parser_results = iter(
        [
            first_packet,
            second_packet,
            None,
        ]
    )

    parser_buffers = []

    def _pop_packet(buffer):
        parser_buffers.append(bytes(buffer))
        return next(parser_results)

    monkeypatch.setattr(
        module,
        "pop_telemetry_packet",
        _pop_packet,
    )

    transport = _FakeTransport()
    transport.read_bytes = b"received telemetry"

    published_packets = []

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._transport = transport
    node._receive_buffer = bytearray(b"old ")
    node._publish_odometry = published_packets.append

    node._read_telemetry()

    assert parser_buffers[0] == b"old received telemetry"

    assert published_packets == [
        first_packet,
        second_packet,
    ]

# проверяет поля /odom, quaternion и TF
def test_publish_odometry_publishes_ros_message_and_tf(
    monkeypatch,
):
    module = _load_bridge_module(monkeypatch)

    publisher = _FakePublisher()
    broadcaster = _FakeTransformBroadcaster()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._odom_frame = "odom"
    node._base_frame = "base_footprint"
    node._odom_publisher = publisher
    node._tf_broadcaster = broadcaster
    node._publish_odom_tf = True
    node._pose_covariance_diagonal = (
        0.02,
        0.02,
        1.0e6,
        1.0e6,
        1.0e6,
        0.05,
    )
    node._twist_covariance_diagonal = (
        0.01,
        0.01,
        1.0e6,
        1.0e6,
        1.0e6,
        0.03,
    )
    node.get_clock = lambda: _FakeClock(1_000_000_000)

    telemetry = _FakeTelemetryPacket(
        odom_x_m=1.25,
        odom_y_m=-0.40,
        odom_heading_rad=math.pi / 2.0,
        current_linear_mps=0.35,
        current_angular_rps=-0.20,
    )

    node._publish_odometry(telemetry)

    assert len(publisher.messages) == 1
    assert len(broadcaster.transforms) == 1

    odom = publisher.messages[0]
    transform = broadcaster.transforms[0]

    expected_z = math.sin(math.pi / 4.0)
    expected_w = math.cos(math.pi / 4.0)

    assert odom.header.stamp == {
        "nanoseconds": 1_000_000_000
    }

    assert odom.header.frame_id == "odom"
    assert odom.child_frame_id == "base_footprint"

    assert odom.pose.pose.position.x == pytest.approx(1.25)
    assert odom.pose.pose.position.y == pytest.approx(-0.40)
    assert odom.pose.pose.position.z == pytest.approx(0.0)

    assert (
        odom.pose.pose.orientation.z
        == pytest.approx(expected_z)
    )

    assert (
        odom.pose.pose.orientation.w
        == pytest.approx(expected_w)
    )

    assert (
        odom.twist.twist.linear.x
        == pytest.approx(0.35)
    )

    assert (
        odom.twist.twist.angular.z
        == pytest.approx(-0.20)
    )
    assert odom.pose.covariance[0] == pytest.approx(0.02)
    assert odom.pose.covariance[35] == pytest.approx(0.05)
    assert odom.twist.covariance[0] == pytest.approx(0.01)
    assert odom.twist.covariance[35] == pytest.approx(0.03)

    assert transform.header.stamp == odom.header.stamp
    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base_footprint"

    assert (
        transform.transform.translation.x
        == pytest.approx(1.25)
    )

    assert (
        transform.transform.translation.y
        == pytest.approx(-0.40)
    )

    assert (
        transform.transform.rotation.z
        == pytest.approx(expected_z)
    )

    assert (
        transform.transform.rotation.w
        == pytest.approx(expected_w)
    )


# проверяет, что при включённом EKF bridge не создаёт второй odom TF
def test_publish_odometry_does_not_publish_tf_when_disabled(
    monkeypatch,
):
    module = _load_bridge_module(monkeypatch)

    publisher = _FakePublisher()
    broadcaster = _FakeTransformBroadcaster()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._odom_frame = "odom"
    node._base_frame = "base_footprint"
    node._odom_publisher = publisher
    node._tf_broadcaster = broadcaster
    node._publish_odom_tf = False
    node._pose_covariance_diagonal = (0.0,) * 6
    node._twist_covariance_diagonal = (0.0,) * 6
    node.get_clock = lambda: _FakeClock(1_000_000_000)

    node._publish_odometry(_FakeTelemetryPacket())

    assert len(publisher.messages) == 1
    assert broadcaster.transforms == []

# проверяет реакцию на потерю serial-порта.
def test_handle_serial_error_closes_transport_and_shutdowns_ros(
    monkeypatch,
):
    module = _load_bridge_module(monkeypatch)

    logger = _FakeLogger()
    transport = _FakeTransport()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._transport = transport
    node.get_logger = lambda: logger

    exception = RuntimeError("device disconnected")

    node._handle_serial_error(
        "read",
        exception,
    )

    assert transport.closed is True

    assert module._test_rclpy_state["shutdown_calls"] == 1

    assert logger.fatal_messages == [
        "Serial read failed: device disconnected"
    ]

# проверяет отправку нулевой команды перед завершением
def test_destroy_node_sends_stop_and_closes_transport(
    monkeypatch,
):
    module = _load_bridge_module(monkeypatch)

    transport = _FakeTransport()

    node = module.ArduinoBridgeNode.__new__(
        module.ArduinoBridgeNode
    )

    node._transport = transport
    node._base_destroy_called = False

    node.destroy_node()

    assert module._test_protocol_module.pack_calls == [
        (0.0, 0.0, False)
    ]

    assert transport.writes == [
        b"0.000,0.000,0"
    ]

    assert transport.closed is True
    assert node._base_destroy_called is True

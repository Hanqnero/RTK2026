import importlib
import sys
import types
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
DRIVER_SRC = REPO_ROOT / "src" / "rtk2026_driver"
if str(DRIVER_SRC) not in sys.path:
    sys.path.insert(0, str(DRIVER_SRC))


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


class _FakeNode:
    pass


class _FakeEncoderReport:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id=None)
        self.left_count = None
        self.right_count = None


class _FakeTransport:
    def __init__(self):
        self.writes = []
        self.read_bytes = b""

    def write(self, payload: bytes):
        self.writes.append(payload)

    def read(self, size: int = 256):
        data, self.read_bytes = self.read_bytes, b""
        return data


def _load_bridge_module(monkeypatch):
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = lambda *args, **kwargs: None
    rclpy_mod.shutdown = lambda *args, **kwargs: None
    rclpy_mod.spin = lambda *args, **kwargs: None
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_mod)

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _FakeNode
    monkeypatch.setitem(sys.modules, "rclpy.node", node_mod)

    exec_mod = types.ModuleType("rclpy.executors")

    class _FakeExternalShutdownException(Exception):
        pass

    exec_mod.ExternalShutdownException = _FakeExternalShutdownException
    monkeypatch.setitem(sys.modules, "rclpy.executors", exec_mod)

    geometry_msg_mod = types.ModuleType("geometry_msgs.msg")

    class _FakeTwist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.0)
            self.angular = types.SimpleNamespace(z=0.0)

    geometry_msg_mod.Twist = _FakeTwist
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msg_mod)

    interfaces_mod = types.ModuleType("rtk2026_interfaces.msg")
    interfaces_mod.EncoderReport = _FakeEncoderReport
    monkeypatch.setitem(sys.modules, "rtk2026_interfaces.msg", interfaces_mod)

    protocol_mod = types.ModuleType("rtk2026_driver.protocol")

    class _FakeInvalidChecksumError(Exception):
        pass

    protocol_mod.InvalidChecksumError = _FakeInvalidChecksumError
    protocol_mod._pack_calls = []

    def _pack_command(linear_mps, angular_rps):
        protocol_mod._pack_calls.append((linear_mps, angular_rps))
        return f"{linear_mps:.3f},{angular_rps:.3f}".encode()

    protocol_mod.pack_command = _pack_command
    protocol_mod.parse_telemetry = lambda buf: None
    monkeypatch.setitem(sys.modules, "rtk2026_driver.protocol", protocol_mod)

    transport_mod = types.ModuleType("rtk2026_driver.transport")

    class _FakeHandshakeTimeoutError(Exception):
        pass

    transport_mod.HandshakeTimeoutError = _FakeHandshakeTimeoutError
    transport_mod.SerialTransport = _FakeTransport
    monkeypatch.setitem(sys.modules, "rtk2026_driver.transport", transport_mod)

    sys.modules.pop("rtk2026_driver.arduino_bridge_node", None)
    return importlib.import_module("rtk2026_driver.arduino_bridge_node")


def test_on_cmd_vel_updates_latest_command(monkeypatch):
    module = _load_bridge_module(monkeypatch)
    node = module.ArduinoBridgeNode.__new__(module.ArduinoBridgeNode)
    node.get_clock = lambda: _FakeClock(123_000_000)

    twist = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.42),
        angular=types.SimpleNamespace(z=-0.17),
    )

    node._on_cmd_vel(twist)

    assert node._last_linear_mps == 0.42
    assert node._last_angular_rps == -0.17
    assert node._last_cmd_time_ns == 123_000_000


def test_current_command_returns_zero_when_stale(monkeypatch):
    module = _load_bridge_module(monkeypatch)
    node = module.ArduinoBridgeNode.__new__(module.ArduinoBridgeNode)
    node._last_linear_mps = 0.5
    node._last_angular_rps = 0.1
    node._last_cmd_time_ns = 100_000_000
    node._drop_stale_cmd_after_sec = 0.30
    node.get_clock = lambda: _FakeClock(500_000_000)

    assert node._current_command() == (0.0, 0.0)


def test_send_command_writes_latest_linear_and_angular(monkeypatch):
    module = _load_bridge_module(monkeypatch)
    transport = _FakeTransport()
    node = module.ArduinoBridgeNode.__new__(module.ArduinoBridgeNode)
    node._transport = transport
    node._current_command = lambda: (0.12, -0.34)

    module.ArduinoBridgeNode._send_command(node)

    assert transport.writes == [b"0.120,-0.340"]


def test_stop_motors_sends_zero_command(monkeypatch):
    module = _load_bridge_module(monkeypatch)
    transport = _FakeTransport()
    node = module.ArduinoBridgeNode.__new__(module.ArduinoBridgeNode)
    node._transport = transport

    module.ArduinoBridgeNode._stop_motors(node)

    assert transport.writes == [b"0.000,0.000"]


def test_read_telemetry_publishes_encoder_report(monkeypatch):
    module = _load_bridge_module(monkeypatch)
    published = []
    transport = _FakeTransport()
    transport.read_bytes = b"telemetry"

    frame = types.SimpleNamespace(left_count=10, right_count=-20)
    parse_calls = {"count": 0}

    def _parse_telemetry(buf):
        parse_calls["count"] += 1
        if parse_calls["count"] == 1:
            return frame
        return None

    monkeypatch.setattr(module, "parse_telemetry", _parse_telemetry)

    node = module.ArduinoBridgeNode.__new__(module.ArduinoBridgeNode)
    node._transport = transport
    node._rx_buf = bytearray()
    node._left_enc_sign = -1
    node._right_enc_sign = 1
    node._encoder_pub = types.SimpleNamespace(publish=lambda msg: published.append(msg))
    node.get_clock = lambda: _FakeClock(1_000_000_000)
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda *a, **k: None, error=lambda *a, **k: None)

    module.ArduinoBridgeNode._read_telemetry(node)

    assert len(published) == 1
    assert published[0].left_count == -10
    assert published[0].right_count == -20
    assert published[0].header.frame_id == "base_link"

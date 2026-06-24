import importlib.util
import math
import os
import sys
from pathlib import Path
from types import ModuleType
from typing import Optional

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class DryRunArm:
    def __init__(self, logger, kinematics: Optional[ModuleType]) -> None:
        self._logger = logger
        self._kinematics = kinematics
        self.x = 0.0
        self.y = 100.0
        self.z = 50.0

    def begin(self, block: int = 0, address: int = 0x40) -> None:
        self._logger.info(
            f"dry_run: initialized virtual meArm at block={block}, address=0x{address:02x}"
        )

    def isReachable(self, x: float, y: float, z: float) -> bool:
        if self._kinematics is None:
            return True
        return bool(self._kinematics.solve(x, y, z, [0.0, 0.0, 0.0]))

    def gotoPoint(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z

    def openGripper(self) -> None:
        return None

    def closeGripper(self) -> None:
        return None


class MearmNode(Node):
    def __init__(self) -> None:
        super().__init__("mearm_node")

        self.declare_parameter("dry_run", True)
        self.declare_parameter("block", 0)
        self.declare_parameter("address", 0x40)
        self.declare_parameter("target_topic", "/mearm/target")
        self.declare_parameter("gripper_status_topic", "/mearm/gripper_status")
        self.declare_parameter("queue_depth", 10)
        self.declare_parameter("vendor_path", "")

        self._dry_run = self._bool_parameter("dry_run")
        self._block = self._int_parameter("block")
        self._address = self._int_parameter("address")
        self._target_topic = str(self.get_parameter("target_topic").value)
        self._gripper_status_topic = str(
            self.get_parameter("gripper_status_topic").value
        )
        queue_depth = max(1, self._int_parameter("queue_depth"))

        configured_vendor_path = str(self.get_parameter("vendor_path").value)
        self._vendor_path = self._resolve_vendor_path(configured_vendor_path)
        self._arm = self._create_arm()
        self._arm.begin(self._block, self._address)

        self._target_sub = self.create_subscription(
            Point,
            self._target_topic,
            self._on_target,
            queue_depth,
        )
        self._gripper_sub = self.create_subscription(
            Bool,
            self._gripper_status_topic,
            self._on_gripper_status,
            queue_depth,
        )

        self.get_logger().info(
            "meArm node ready: "
            f"target_topic={self._target_topic}, "
            f"gripper_status_topic={self._gripper_status_topic}, "
            f"dry_run={self._dry_run}"
        )

    def _int_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def _bool_parameter(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _resolve_vendor_path(self, configured_path: str) -> Optional[Path]:
        if configured_path:
            return Path(configured_path).expanduser().resolve()

        env_path = os.environ.get("RTK2026_MEARM_VENDOR_PATH")
        if env_path:
            return Path(env_path).expanduser().resolve()

        for root in [Path.cwd(), *Path(__file__).resolve().parents]:
            candidate = root / "vendor" / "mearm"
            if (candidate / "kinematics.py").is_file():
                return candidate.resolve()

        try:
            share_dir = Path(get_package_share_directory("rtk2026_mearm"))
        except PackageNotFoundError:
            return None

        candidate = share_dir / "vendor" / "mearm"
        if (candidate / "kinematics.py").is_file():
            return candidate.resolve()

        return None

    def _create_arm(self):
        if self._vendor_path is not None and self._vendor_path.is_dir():
            sys.path.insert(0, str(self._vendor_path))
            self.get_logger().info(f"Using meArm vendor path: {self._vendor_path}")
        elif self._dry_run:
            self.get_logger().warning(
                "meArm vendor path was not found; dry_run will accept all target points"
            )
        else:
            raise RuntimeError(
                "meArm vendor path was not found. Set vendor_path or "
                "RTK2026_MEARM_VENDOR_PATH."
            )

        if self._dry_run:
            kinematics = self._load_vendor_module("kinematics.py", "mearm_kinematics")
            if kinematics is None:
                self.get_logger().warning(
                    "dry_run reachability checks are disabled because kinematics.py "
                    "could not be loaded"
                )
            return DryRunArm(self.get_logger(), kinematics)

        mearm_module = self._load_vendor_module("meArm.py", "meArm")
        if mearm_module is None:
            raise RuntimeError("Could not import vendor/mearm/meArm.py")
        return mearm_module.meArm()

    def _load_vendor_module(
        self, filename: str, module_name: str
    ) -> Optional[ModuleType]:
        if self._vendor_path is None:
            return None

        module_path = self._vendor_path / filename
        if not module_path.is_file():
            return None

        spec = importlib.util.spec_from_file_location(module_name, module_path)
        if spec is None or spec.loader is None:
            return None

        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as exc:
            if self._dry_run:
                self.get_logger().warning(
                    f"Could not load {module_path} in dry_run mode: {exc}"
                )
                return None
            raise
        return module

    def _on_target(self, msg: Point) -> None:
        x = float(msg.x)
        y = float(msg.y)
        z = float(msg.z)

        if not all(math.isfinite(value) for value in (x, y, z)):
            self.get_logger().warning(
                f"Rejected non-finite meArm target: x={x}, y={y}, z={z}"
            )
            return

        if not self._arm.isReachable(x, y, z):
            self.get_logger().warning(
                f"Rejected unreachable meArm target: x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )
            return

        try:
            self._arm.gotoPoint(x, y, z)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to move meArm to x={x:.3f}, y={y:.3f}, z={z:.3f}: {exc}"
            )
            return

        if self._dry_run:
            self.get_logger().info(
                f"dry_run: accepted target x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )
        else:
            self.get_logger().info(
                f"Moved meArm to x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )

    def _on_gripper_status(self, msg: Bool) -> None:
        should_close = bool(msg.data)

        try:
            if should_close:
                self._arm.closeGripper()
            else:
                self._arm.openGripper()
        except Exception as exc:
            action = "close" if should_close else "open"
            self.get_logger().error(f"Failed to {action} meArm gripper: {exc}")
            return

        if should_close:
            self.get_logger().info(
                "dry_run: close gripper" if self._dry_run else "Closed gripper"
            )
        else:
            self.get_logger().info(
                "dry_run: open gripper" if self._dry_run else "Opened gripper"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MearmNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

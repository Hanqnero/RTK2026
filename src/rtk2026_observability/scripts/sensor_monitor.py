#!/usr/bin/env python3
"""Содержательная диагностика лидара, IMU и колёсной обратной связи.

Частота и наличие topics проверяются отдельным ``topic_monitor``. Эта нода
проверяет сами данные: геометрию LaserScan, конечность и ковариации IMU,
наличие wheel joints и соответствие направления измеренного движения
фактически применённой команде контроллера.
"""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
from threading import Lock
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
import yaml


def _finite(values) -> bool:
    """Проверить, что все значения конечны."""

    return all(math.isfinite(float(value)) for value in values)


def _mean(values: list[float]) -> float:
    """Среднее непустого или пустого списка."""

    return sum(values) / len(values) if values else 0.0


def _stddev(values: list[float]) -> float:
    """Стандартное отклонение без зависимости от NumPy."""

    if len(values) < 2:
        return 0.0
    average = _mean(values)
    return math.sqrt(_mean([(value - average) ** 2 for value in values]))


class LidarTask(DiagnosticTask):
    """Проверить содержимое последнего LaserScan."""

    def __init__(self, monitor: "SensorMonitor") -> None:
        super().__init__("Sensors/Lidar")
        self._monitor = monitor

    def run(self, status):
        data = self._monitor.lidar_snapshot()
        config = self._monitor.config["lidar"]
        if data is None:
            status.summary(DiagnosticStatus.STALE, "LaserScan пока не получен")
            return status

        status.add("Rays", str(data["rays"]))
        status.add("Expected rays", str(config["expected_rays"]))
        status.add("Finite ratio", f"{data['finite_ratio']:.3f}")
        status.add("Inf ratio", f"{data['inf_ratio']:.3f}")
        status.add("NaN ratio", f"{data['nan_ratio']:.3f}")
        status.add("Valid minimum (m)", f"{data['valid_min']:.3f}")
        status.add("Valid maximum (m)", f"{data['valid_max']:.3f}")
        status.add("Outside declared range", str(data["outside_count"]))
        status.add("Unchanged scans", str(data["unchanged_count"]))

        problems: list[tuple[int, str]] = []
        if data["rays"] != int(config["expected_rays"]):
            problems.append((DiagnosticStatus.ERROR, "Неверное число лучей"))
        if data["nan_ratio"] > float(config["max_nan_ratio"]):
            problems.append((DiagnosticStatus.ERROR, "Слишком много NaN"))
        if data["finite_ratio"] < float(config["min_finite_ratio"]):
            problems.append((DiagnosticStatus.WARN, "Мало конечных дальностей"))
        if data["outside_count"]:
            problems.append(
                (DiagnosticStatus.ERROR, "Дальности выходят за объявленный диапазон")
            )
        expected_span = float(config.get("expected_angle_span", 0.0))
        if expected_span and abs(data["angle_span"] - expected_span) > 0.02:
            problems.append((DiagnosticStatus.ERROR, "Неверный угловой диапазон"))
        if data["unchanged_count"] >= int(config["unchanged_warn_count"]):
            problems.append((DiagnosticStatus.WARN, "LaserScan долго не изменяется"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "Данные лидара корректны")
        return status


class ImuTask(DiagnosticTask):
    """Проверить IMU, её ковариации и шум на стоянке."""

    def __init__(self, monitor: "SensorMonitor") -> None:
        super().__init__("Sensors/IMU")
        self._monitor = monitor

    def run(self, status):
        data = self._monitor.imu_snapshot()
        config = self._monitor.config["imu"]
        if data is None:
            status.summary(DiagnosticStatus.STALE, "IMU пока не получена")
            return status

        status.add("Gyroscope norm (rad/s)", f"{data['gyro_norm']:.4f}")
        status.add("Acceleration norm (m/s^2)", f"{data['accel_norm']:.4f}")
        status.add("Orientation quaternion norm", f"{data['quaternion_norm']:.5f}")
        status.add("Stationary samples", str(data["stationary_samples"]))
        status.add("Stationary gyro Z mean (rad/s)", f"{data['gyro_z_mean']:.6f}")
        status.add("Stationary gyro Z stddev (rad/s)", f"{data['gyro_z_stddev']:.6f}")
        status.add(
            "Angular velocity covariance diagonal",
            ", ".join(f"{value:.8g}" for value in data["gyro_covariance"]),
        )

        problems: list[tuple[int, str]] = []
        if not data["finite"]:
            problems.append((DiagnosticStatus.ERROR, "IMU содержит NaN или Inf"))
        if data["gyro_norm"] > float(config["gyro_saturation"]):
            problems.append((DiagnosticStatus.ERROR, "Гироскоп насыщен"))
        if data["accel_norm"] > float(config["accel_saturation"]):
            problems.append((DiagnosticStatus.ERROR, "Акселерометр насыщен"))
        if not data["covariance_valid"]:
            problems.append((DiagnosticStatus.ERROR, "Некорректна ковариация IMU"))
        if data["orientation_available"] and abs(data["quaternion_norm"] - 1.0) > 0.02:
            problems.append((DiagnosticStatus.ERROR, "Quaternion IMU не нормирован"))
        if data["stationary_samples"] >= int(config["stationary_min_samples"]):
            if abs(data["gyro_z_mean"]) > float(config["stationary_bias_warn"]):
                problems.append((DiagnosticStatus.WARN, "Смещение gyro Z на стоянке"))
            if data["gyro_z_stddev"] > float(config["stationary_noise_warn"]):
                problems.append((DiagnosticStatus.WARN, "Высокий шум gyro Z на стоянке"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "Данные IMU корректны")
        return status


class DriveTask(DiagnosticTask):
    """Проверить wheel joint feedback и направление отклика привода."""

    def __init__(self, monitor: "SensorMonitor") -> None:
        super().__init__("Sensors/Drive")
        self._monitor = monitor

    def run(self, status):
        data = self._monitor.drive_snapshot()
        config = self._monitor.config["drive"]
        status.add("Left joint", config["left_joint"])
        status.add("Right joint", config["right_joint"])
        status.add("Joint state received", str(data["joint_received"]))
        status.add("Wheel odometry received", str(data["odom_received"]))
        status.add("Applied command received", str(data["command_received"]))
        status.add("Left velocity (rad/s)", f"{data['left_velocity']:.4f}")
        status.add("Right velocity (rad/s)", f"{data['right_velocity']:.4f}")
        status.add("Command linear X (m/s)", f"{data['command_linear']:.4f}")
        status.add("Command angular Z (rad/s)", f"{data['command_angular']:.4f}")
        status.add("Measured linear X (m/s)", f"{data['odom_linear']:.4f}")
        status.add("Measured angular Z (rad/s)", f"{data['odom_angular']:.4f}")
        status.add("Direction checks", str(data["checks"]))
        status.add("Direction mismatches", str(data["mismatches"]))

        problems: list[tuple[int, str]] = []
        if not data["joint_received"] or not data["odom_received"]:
            problems.append((DiagnosticStatus.STALE, "Нет колёсной обратной связи"))
        elif not data["joints_present"]:
            problems.append((DiagnosticStatus.ERROR, "Wheel joints отсутствуют"))
        elif not data["finite"]:
            problems.append((DiagnosticStatus.ERROR, "Joint feedback содержит NaN/Inf"))

        if data["checks"] >= int(config["minimum_checks"]):
            ratio = data["mismatches"] / data["checks"]
            status.add("Direction mismatch ratio", f"{ratio:.3f}")
            if ratio > float(config["max_mismatch_ratio"]):
                problems.append(
                    (DiagnosticStatus.ERROR, "Направление feedback не совпадает с командой")
                )

        if data["command_received"]:
            command_age = data["command_age"]
            status.add("Applied command age (s)", f"{command_age:.3f}")
            if (
                command_age < float(config["command_timeout"])
                and data["command_moving"]
                and not data["feedback_moving"]
            ):
                problems.append((DiagnosticStatus.WARN, "Есть команда, но робот не движется"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "Колёсная обратная связь корректна")
        return status


class SensorMonitor(Node):
    """Собирать диагностические метрики сенсоров и привода."""

    def __init__(self) -> None:
        super().__init__("sensor_monitor")
        self.declare_parameter("config_file", "")
        config_file = self.get_parameter("config_file").value
        if not config_file:
            raise ValueError("parameter config_file is required")
        with Path(config_file).open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)

        self._lock = Lock()
        self._lidar: dict[str, Any] | None = None
        self._last_scan_signature: tuple[float, ...] | None = None
        self._unchanged_scans = 0
        self._imu: dict[str, Any] | None = None
        window_size = int(self.config["imu"]["stationary_window"])
        self._stationary_gyro_z: deque[float] = deque(maxlen=window_size)

        self._joint_received = False
        self._joints_present = False
        self._joint_finite = False
        self._left_velocity = 0.0
        self._right_velocity = 0.0
        self._odom_received = False
        self._odom_linear = 0.0
        self._odom_angular = 0.0
        self._command_received = False
        self._command_receive_ns = 0
        self._command_linear = 0.0
        self._command_angular = 0.0
        self._direction_results: deque[bool] = deque(
            maxlen=int(self.config["drive"]["direction_window"])
        )

        self.create_subscription(
            LaserScan,
            self.config["lidar"]["topic"],
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.config["imu"]["topic"],
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self.config["drive"]["joint_topic"],
            self._on_joint_state,
            10,
        )
        self.create_subscription(
            Odometry,
            self.config["drive"]["odometry_topic"],
            self._on_odometry,
            10,
        )
        self.create_subscription(
            TwistStamped,
            self.config["drive"]["applied_command_topic"],
            self._on_command,
            10,
        )

        self._updater = Updater(self)
        self._updater.setHardwareID("rtk2026-sensors")
        self._updater.add(LidarTask(self))
        self._updater.add(ImuTask(self))
        self._updater.add(DriveTask(self))

    def _on_scan(self, message: LaserScan) -> None:
        ranges = list(message.ranges)
        finite = [float(value) for value in ranges if math.isfinite(value)]
        nan_count = sum(math.isnan(float(value)) for value in ranges)
        inf_count = sum(math.isinf(float(value)) for value in ranges)
        outside = sum(
            value < message.range_min or value > message.range_max
            for value in finite
        )
        # 32 равномерно выбранных значений достаточно, чтобы заметить
        # замерший поток, не копируя весь scan в память.
        step = max(1, len(ranges) // 32)
        signature = tuple(round(float(value), 3) for value in ranges[::step])
        with self._lock:
            if signature == self._last_scan_signature:
                self._unchanged_scans += 1
            else:
                self._unchanged_scans = 0
            self._last_scan_signature = signature
            count = max(1, len(ranges))
            self._lidar = {
                "rays": len(ranges),
                "finite_ratio": len(finite) / count,
                "inf_ratio": inf_count / count,
                "nan_ratio": nan_count / count,
                "valid_min": min(finite) if finite else math.nan,
                "valid_max": max(finite) if finite else math.nan,
                "outside_count": outside,
                "angle_span": float(message.angle_max - message.angle_min),
                "unchanged_count": self._unchanged_scans,
            }

    def _on_imu(self, message: Imu) -> None:
        gyro = message.angular_velocity
        accel = message.linear_acceleration
        quaternion = message.orientation
        values = [gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z]
        orientation_available = message.orientation_covariance[0] >= 0.0
        covariance = [
            float(message.angular_velocity_covariance[index])
            for index in (0, 4, 8)
        ]
        covariance_valid = _finite(covariance) and all(
            value >= 0.0 for value in covariance
        )
        with self._lock:
            stationary = (
                abs(self._odom_linear)
                < float(self.config["imu"]["stationary_linear_threshold"])
                and abs(self._odom_angular)
                < float(self.config["imu"]["stationary_angular_threshold"])
            )
            if stationary:
                self._stationary_gyro_z.append(float(gyro.z))
            else:
                self._stationary_gyro_z.clear()
            samples = list(self._stationary_gyro_z)
            self._imu = {
                "finite": _finite(values),
                "gyro_norm": math.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2),
                "accel_norm": math.sqrt(accel.x**2 + accel.y**2 + accel.z**2),
                "quaternion_norm": math.sqrt(
                    quaternion.x**2
                    + quaternion.y**2
                    + quaternion.z**2
                    + quaternion.w**2
                ),
                "orientation_available": orientation_available,
                "covariance_valid": covariance_valid,
                "gyro_covariance": covariance,
                "stationary_samples": len(samples),
                "gyro_z_mean": _mean(samples),
                "gyro_z_stddev": _stddev(samples),
            }

    def _on_joint_state(self, message: JointState) -> None:
        config = self.config["drive"]
        indexes = {name: index for index, name in enumerate(message.name)}
        left_index = indexes.get(config["left_joint"])
        right_index = indexes.get(config["right_joint"])
        present = (
            left_index is not None
            and right_index is not None
            and left_index < len(message.velocity)
            and right_index < len(message.velocity)
        )
        with self._lock:
            self._joint_received = True
            self._joints_present = present
            if present:
                self._left_velocity = float(message.velocity[left_index])
                self._right_velocity = float(message.velocity[right_index])
                self._joint_finite = _finite(
                    [self._left_velocity, self._right_velocity]
                )
                self._record_direction_check()

    def _on_odometry(self, message: Odometry) -> None:
        with self._lock:
            self._odom_received = True
            self._odom_linear = float(message.twist.twist.linear.x)
            self._odom_angular = float(message.twist.twist.angular.z)

    def _on_command(self, message: TwistStamped) -> None:
        with self._lock:
            self._command_received = True
            self._command_receive_ns = time.monotonic_ns()
            self._command_linear = float(message.twist.linear.x)
            self._command_angular = float(message.twist.angular.z)

    def _record_direction_check(self) -> None:
        config = self.config["drive"]
        if not self._command_received:
            return
        if (time.monotonic_ns() - self._command_receive_ns) / 1e9 > float(
            config["command_timeout"]
        ):
            return
        radius = float(config["wheel_radius"])
        half_separation = float(config["wheel_separation"]) / 2.0
        expected = (
            (self._command_linear - self._command_angular * half_separation)
            / radius,
            (self._command_linear + self._command_angular * half_separation)
            / radius,
        )
        actual = (self._left_velocity, self._right_velocity)
        threshold = float(config["direction_velocity_threshold"])
        checked = [
            math.copysign(1.0, expected_value)
            == math.copysign(1.0, actual_value)
            for expected_value, actual_value in zip(expected, actual)
            if abs(expected_value) > threshold and abs(actual_value) > threshold
        ]
        if checked:
            self._direction_results.append(all(checked))

    def lidar_snapshot(self):
        """Вернуть копию метрик лидара."""

        with self._lock:
            return None if self._lidar is None else dict(self._lidar)

    def imu_snapshot(self):
        """Вернуть копию метрик IMU."""

        with self._lock:
            return None if self._imu is None else dict(self._imu)

    def drive_snapshot(self):
        """Вернуть согласованный снимок drive-метрик."""

        with self._lock:
            command_age = (
                (time.monotonic_ns() - self._command_receive_ns) / 1e9
                if self._command_received
                else math.inf
            )
            threshold = float(self.config["drive"]["motion_threshold"])
            return {
                "joint_received": self._joint_received,
                "joints_present": self._joints_present,
                "finite": self._joint_finite,
                "left_velocity": self._left_velocity,
                "right_velocity": self._right_velocity,
                "odom_received": self._odom_received,
                "odom_linear": self._odom_linear,
                "odom_angular": self._odom_angular,
                "command_received": self._command_received,
                "command_age": command_age,
                "command_linear": self._command_linear,
                "command_angular": self._command_angular,
                "command_moving": (
                    abs(self._command_linear) > threshold
                    or abs(self._command_angular) > threshold
                ),
                "feedback_moving": (
                    abs(self._odom_linear) > threshold
                    or abs(self._odom_angular) > threshold
                ),
                "checks": len(self._direction_results),
                "mismatches": sum(not value for value in self._direction_results),
            }


def main(args=None) -> None:
    """Запустить sensor monitor."""

    rclpy.init(args=args)
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

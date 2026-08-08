#!/usr/bin/env python3
"""Диагностика EKF, SLAM и AMCL по стандартным ROS 2 сообщениям.

Нода не вмешивается в оценивание состояния. Она проверяет ковариации,
согласованность скоростей wheel odometry, EKF и IMU, качество occupancy grid,
а в localization-режиме — распределение частиц AMCL.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from threading import Lock
import time
from typing import Any

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticTask, Updater
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu
import yaml


def _stamp_ns(message: Any) -> int:
    """Преобразовать ``header.stamp`` в наносекунды."""

    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def _yaw(quaternion: Any) -> float:
    """Получить planar yaw из quaternion."""

    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def _normalize_angle(angle: float) -> float:
    """Привести угол к диапазону ``[-pi, pi]``."""

    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class MotionSample:
    """Последняя скорость и время одного источника."""

    receive_ns: int
    stamp_ns: int
    linear_x: float
    angular_z: float


@dataclass
class CovarianceSample:
    """Последняя pose covariance одного оценивателя."""

    receive_ns: int
    stamp_ns: int
    frame_id: str
    covariance: list[float]


@dataclass
class PoseSample:
    """Последняя planar pose одного источника."""

    receive_ns: int
    stamp_ns: int
    x: float
    y: float
    yaw: float


def _relative_pose(sample: PoseSample, origin: PoseSample) -> PoseSample:
    """Выразить движение от первой позы в её локальной системе координат.

    Wheel odometry и EKF обычно начинают с ``(0, 0, 0)``, а Gazebo ground
    truth содержит world-позу spawn. Сравнивать их абсолютные координаты
    нельзя. После удаления начального SE(2)-преобразования остаётся только
    накопленная ошибка движения.
    """

    delta_x = sample.x - origin.x
    delta_y = sample.y - origin.y
    cos_yaw = math.cos(origin.yaw)
    sin_yaw = math.sin(origin.yaw)
    return PoseSample(
        receive_ns=sample.receive_ns,
        stamp_ns=sample.stamp_ns,
        x=cos_yaw * delta_x + sin_yaw * delta_y,
        y=-sin_yaw * delta_x + cos_yaw * delta_y,
        yaw=_normalize_angle(sample.yaw - origin.yaw),
    )


def _covariance_metrics(covariance: list[float]) -> dict[str, Any]:
    """Рассчитать диагностические метрики матрицы 6x6."""

    finite = all(math.isfinite(value) for value in covariance)
    diagonal = [covariance[index * 6 + index] for index in range(6)]
    nonnegative = all(value >= 0.0 for value in diagonal)
    max_asymmetry = max(
        abs(covariance[row * 6 + column] - covariance[column * 6 + row])
        for row in range(6)
        for column in range(6)
    )
    matrix = numpy.asarray(covariance, dtype=float).reshape((6, 6))
    symmetric_matrix = (matrix + matrix.T) / 2.0
    minimum_eigenvalue = (
        float(numpy.linalg.eigvalsh(symmetric_matrix).min())
        if finite
        else math.nan
    )

    def correlation(first: int, second: int) -> float:
        denominator = math.sqrt(
            max(0.0, covariance[first * 6 + first])
            * max(0.0, covariance[second * 6 + second])
        )
        if denominator == 0.0:
            return 0.0
        return covariance[first * 6 + second] / denominator

    return {
        "finite": finite,
        "nonnegative": nonnegative,
        "max_asymmetry": max_asymmetry,
        "minimum_eigenvalue": minimum_eigenvalue,
        "diagonal": diagonal,
        "sigma_x": math.sqrt(max(0.0, diagonal[0])),
        "sigma_y": math.sqrt(max(0.0, diagonal[1])),
        "sigma_yaw": math.sqrt(max(0.0, diagonal[5])),
        "correlation_xy": correlation(0, 1),
        "correlation_x_yaw": correlation(0, 5),
        "correlation_y_yaw": correlation(1, 5),
    }


class CovarianceTask(DiagnosticTask):
    """Проверка pose covariance выбранного источника."""

    def __init__(
        self,
        monitor: "LocalizationMonitor",
        key: str,
        title: str,
        required: bool,
    ) -> None:
        super().__init__(f"Localization/Covariance/{title}")
        self._monitor = monitor
        self._key = key
        self._required = required

    def run(self, status):
        sample = self._monitor.covariance_snapshot(self._key)
        config = self._monitor.config["covariance"]
        if sample is None:
            level = DiagnosticStatus.STALE if self._required else DiagnosticStatus.OK
            status.summary(level, "Оценка позы пока не получена")
            return status

        metrics = _covariance_metrics(sample.covariance)
        age = (time.monotonic_ns() - sample.receive_ns) / 1e9
        status.add("Frame", sample.frame_id)
        status.add("Receive age (s)", f"{age:.3f}")
        status.add("Sigma X (m)", f"{metrics['sigma_x']:.5f}")
        status.add("Sigma Y (m)", f"{metrics['sigma_y']:.5f}")
        status.add("Sigma yaw (rad)", f"{metrics['sigma_yaw']:.5f}")
        status.add("Correlation X-Y", f"{metrics['correlation_xy']:.4f}")
        status.add("Correlation X-yaw", f"{metrics['correlation_x_yaw']:.4f}")
        status.add("Correlation Y-yaw", f"{metrics['correlation_y_yaw']:.4f}")
        status.add("Maximum asymmetry", f"{metrics['max_asymmetry']:.3e}")
        status.add(
            "Minimum covariance eigenvalue",
            f"{metrics['minimum_eigenvalue']:.3e}",
        )

        problems: list[tuple[int, str]] = []
        if not metrics["finite"]:
            problems.append((DiagnosticStatus.ERROR, "Матрица содержит NaN/Inf"))
        if not metrics["nonnegative"]:
            problems.append((DiagnosticStatus.ERROR, "Отрицательная дисперсия"))
        if metrics["max_asymmetry"] > float(config["symmetry_tolerance"]):
            problems.append((DiagnosticStatus.ERROR, "Матрица несимметрична"))
        if (
            metrics["finite"]
            and metrics["minimum_eigenvalue"]
            < -float(config["positive_semidefinite_tolerance"])
        ):
            problems.append(
                (DiagnosticStatus.ERROR, "Матрица не положительно полуопределена")
            )
        if max(abs(value) for value in sample.covariance) == 0.0:
            problems.append((DiagnosticStatus.WARN, "Ковариация полностью нулевая"))
        # У локальной odometry без абсолютных x/y/yaw-измерений covariance
        # обязана расти со временем. Поэтому числовые пределы применяются
        # только к источникам, явно перечисленным в uncertainty_limits.
        limits = config.get("uncertainty_limits", {}).get(self._key)
        if limits is not None:
            sigma_xy = max(metrics["sigma_x"], metrics["sigma_y"])
            if sigma_xy > float(limits["sigma_xy_error"]):
                problems.append(
                    (DiagnosticStatus.ERROR, "Критически велика sigma XY")
                )
            elif sigma_xy > float(limits["sigma_xy_warn"]):
                problems.append((DiagnosticStatus.WARN, "Велика sigma XY"))
            if metrics["sigma_yaw"] > float(limits["sigma_yaw_error"]):
                problems.append(
                    (DiagnosticStatus.ERROR, "Критически велика sigma yaw")
                )
            elif metrics["sigma_yaw"] > float(limits["sigma_yaw_warn"]):
                problems.append((DiagnosticStatus.WARN, "Велика sigma yaw"))
        # /pose у SLAM Toolbox событийный: после остановки робота последняя
        # корректная оценка может долго не обновляться. Возраст обязателен
        # только для непрерывных источников wheel odometry, EKF и активного
        # AMCL, а для необязательных pose остаётся информационной метрикой.
        if (
            self._key in {"wheel", "filtered"}
            and age > float(config["max_age"])
        ):
            problems.append((DiagnosticStatus.STALE, "Оценка позы устарела"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "Ковариация корректна")
        return status


class ConsistencyTask(DiagnosticTask):
    """Сравнить движение wheel odometry, EKF и IMU."""

    def __init__(self, monitor: "LocalizationMonitor") -> None:
        super().__init__("Localization/Consistency")
        self._monitor = monitor

    def run(self, status):
        samples = self._monitor.motion_snapshot()
        config = self._monitor.config["consistency"]
        wheel = samples.get("wheel")
        filtered = samples.get("filtered")
        imu = samples.get("imu")
        if wheel is None or filtered is None or imu is None:
            status.summary(DiagnosticStatus.STALE, "Не все источники движения получены")
            return status

        wheel_filtered_skew = abs(wheel.stamp_ns - filtered.stamp_ns) / 1e9
        imu_filtered_skew = abs(imu.stamp_ns - filtered.stamp_ns) / 1e9
        linear_difference = abs(wheel.linear_x - filtered.linear_x)
        angular_difference = abs(imu.angular_z - filtered.angular_z)
        status.add("Wheel linear X (m/s)", f"{wheel.linear_x:.4f}")
        status.add("EKF linear X (m/s)", f"{filtered.linear_x:.4f}")
        status.add("IMU angular Z (rad/s)", f"{imu.angular_z:.4f}")
        status.add("EKF angular Z (rad/s)", f"{filtered.angular_z:.4f}")
        status.add("Linear difference (m/s)", f"{linear_difference:.4f}")
        status.add("Angular difference (rad/s)", f"{angular_difference:.4f}")
        status.add("Wheel-EKF timestamp skew (s)", f"{wheel_filtered_skew:.4f}")
        status.add("IMU-EKF timestamp skew (s)", f"{imu_filtered_skew:.4f}")

        problems: list[tuple[int, str]] = []
        if wheel_filtered_skew > float(config["max_timestamp_skew"]):
            problems.append((DiagnosticStatus.WARN, "Wheel odom и EKF рассинхронизированы"))
        if imu_filtered_skew > float(config["max_timestamp_skew"]):
            problems.append((DiagnosticStatus.WARN, "IMU и EKF рассинхронизированы"))
        if linear_difference > float(config["linear_difference_warn"]):
            problems.append((DiagnosticStatus.WARN, "EKF не согласован с wheel velocity"))
        if angular_difference > float(config["angular_difference_warn"]):
            problems.append((DiagnosticStatus.WARN, "EKF не согласован с gyro Z"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "Источники движения согласованы")
        return status


class MapTask(DiagnosticTask):
    """Проверить OccupancyGrid от SLAM Toolbox или Map Server."""

    def __init__(self, monitor: "LocalizationMonitor") -> None:
        super().__init__("Localization/Map")
        self._monitor = monitor

    def run(self, status):
        data = self._monitor.slam_snapshot()
        if data["map"] is None:
            status.summary(DiagnosticStatus.STALE, "Карта пока не получена")
            return status

        grid = data["map"]
        age = (time.monotonic_ns() - grid["receive_ns"]) / 1e9
        status.add("Map frame", grid["frame_id"])
        status.add("Map size", f"{grid['width']} x {grid['height']}")
        status.add("Resolution (m/cell)", f"{grid['resolution']:.4f}")
        status.add("Known cells ratio", f"{grid['known_ratio']:.3f}")
        status.add("Occupied cells ratio", f"{grid['occupied_ratio']:.3f}")
        status.add("Map receive age (s)", f"{age:.3f}")
        status.add("SLAM pose received", str(data["pose"] is not None))

        problems: list[tuple[int, str]] = []
        config = self._monitor.config["slam"]
        if grid["frame_id"] != config["map_frame"]:
            problems.append((DiagnosticStatus.ERROR, "У карты неверный frame_id"))
        if grid["width"] == 0 or grid["height"] == 0:
            problems.append((DiagnosticStatus.ERROR, "Карта пуста"))
        if grid["known_ratio"] < float(config["minimum_known_ratio"]):
            problems.append((DiagnosticStatus.WARN, "Слишком мало известных ячеек"))
        if (
            self._monitor.config["mode"] == "mapping"
            and age > float(config["max_map_age"])
        ):
            problems.append((DiagnosticStatus.STALE, "Карта не обновляется"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "OccupancyGrid пригодна")
        return status


class GroundTruthTask(DiagnosticTask):
    """Сравнить wheel odometry и EKF с идеальной позой Gazebo."""

    def __init__(self, monitor: "LocalizationMonitor") -> None:
        super().__init__("Localization/Ground truth")
        self._monitor = monitor

    def run(self, status):
        poses = self._monitor.relative_pose_snapshot()
        if not all(key in poses for key in ("ground_truth", "wheel", "filtered")):
            status.summary(DiagnosticStatus.STALE, "Не все pose для сравнения получены")
            return status

        truth = poses["ground_truth"]
        wheel = poses["wheel"]
        filtered = poses["filtered"]

        def errors(sample: PoseSample) -> tuple[float, float]:
            position = math.hypot(sample.x - truth.x, sample.y - truth.y)
            yaw_error = abs(
                math.atan2(
                    math.sin(sample.yaw - truth.yaw),
                    math.cos(sample.yaw - truth.yaw),
                )
            )
            return position, yaw_error

        wheel_position, wheel_yaw = errors(wheel)
        filtered_position, filtered_yaw = errors(filtered)
        status.add("Wheel position error (m)", f"{wheel_position:.4f}")
        status.add("Wheel yaw error (rad)", f"{wheel_yaw:.4f}")
        status.add("EKF position error (m)", f"{filtered_position:.4f}")
        status.add("EKF yaw error (rad)", f"{filtered_yaw:.4f}")
        status.add(
            "Wheel-truth timestamp skew (s)",
            f"{abs(wheel.stamp_ns - truth.stamp_ns) / 1e9:.4f}",
        )
        status.add(
            "EKF-truth timestamp skew (s)",
            f"{abs(filtered.stamp_ns - truth.stamp_ns) / 1e9:.4f}",
        )

        config = self._monitor.config["ground_truth"]
        problems: list[tuple[int, str]] = []
        if wheel_position > float(config["position_error_warn"]):
            problems.append((DiagnosticStatus.WARN, "Wheel odometry уходит от truth"))
        if filtered_position > float(config["position_error_warn"]):
            problems.append((DiagnosticStatus.WARN, "EKF уходит от truth"))
        if wheel_yaw > float(config["yaw_error_warn"]):
            problems.append((DiagnosticStatus.WARN, "Yaw wheel odometry уходит от truth"))
        if filtered_yaw > float(config["yaw_error_warn"]):
            problems.append((DiagnosticStatus.WARN, "Yaw EKF уходит от truth"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "Одометрия согласована с Gazebo truth")
        return status


class AmclTask(DiagnosticTask):
    """Проверить pose и распределение частиц AMCL."""

    def __init__(self, monitor: "LocalizationMonitor") -> None:
        super().__init__("Localization/AMCL")
        self._monitor = monitor

    def run(self, status):
        data = self._monitor.amcl_snapshot()
        required = self._monitor.config["mode"] == "localization"
        if data["pose"] is None and data["particles"] is None:
            level = DiagnosticStatus.STALE if required else DiagnosticStatus.OK
            status.summary(level, "AMCL не используется в mapping-режиме")
            return status

        status.add("AMCL pose received", str(data["pose"] is not None))
        status.add("Particles received", str(data["particles"] is not None))
        problems: list[tuple[int, str]] = []
        if data["particles"] is not None:
            particles = data["particles"]
            status.add("Particle count", str(particles["count"]))
            status.add("Effective sample size", f"{particles['effective_size']:.1f}")
            status.add("Weighted sigma X (m)", f"{particles['sigma_x']:.3f}")
            status.add("Weighted sigma Y (m)", f"{particles['sigma_y']:.3f}")
            status.add("Circular yaw concentration", f"{particles['yaw_concentration']:.3f}")
            minimum = int(self._monitor.config["amcl"]["minimum_particles"])
            if particles["count"] < minimum:
                problems.append((DiagnosticStatus.WARN, "Слишком мало частиц"))
            if particles["effective_size"] < minimum * 0.25:
                problems.append((DiagnosticStatus.WARN, "Веса частиц вырождены"))
        # Particle cloud публикуется только при обновлении particle filter.
        # Если монитор стартовал позже или робот стоит, отсутствие сообщения
        # не означает отказ AMCL: обязательны active lifecycle, pose и TF.

        if data["pose"] is None and required:
            problems.append((DiagnosticStatus.STALE, "AMCL pose не получена"))

        if problems:
            level, message = max(problems, key=lambda item: item[0])
            status.summary(level, message)
        else:
            status.summary(DiagnosticStatus.OK, "AMCL работает")
        return status


class LocalizationMonitor(Node):
    """Собирать метрики локализации из стандартных ROS interfaces."""

    def __init__(self) -> None:
        super().__init__("localization_monitor")
        self.declare_parameter("config_file", "")
        self.declare_parameter("mode", "")
        config_file = self.get_parameter("config_file").value
        if not config_file:
            raise ValueError("parameter config_file is required")
        with Path(config_file).open(encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        mode_override = self.get_parameter("mode").value
        if mode_override:
            self.config["mode"] = mode_override
        if self.config["mode"] not in {"mapping", "localization"}:
            raise ValueError("mode must be 'mapping' or 'localization'")

        self._lock = Lock()
        self._motion: dict[str, MotionSample] = {}
        self._poses: dict[str, PoseSample] = {}
        self._initial_poses: dict[str, PoseSample] = {}
        self._covariances: dict[str, CovarianceSample] = {}
        self._map: dict[str, Any] | None = None
        self._slam_pose_received_ns: int | None = None
        self._amcl_pose_received_ns: int | None = None
        self._particles: dict[str, Any] | None = None

        topics = self.config["topics"]
        self.create_subscription(
            Odometry, topics["wheel_odometry"], self._on_wheel_odometry, 10
        )
        self.create_subscription(
            Odometry, topics["filtered_odometry"], self._on_filtered_odometry, 10
        )
        self.create_subscription(
            Odometry, topics["ground_truth"], self._on_ground_truth, 10
        )
        self.create_subscription(
            Imu, topics["imu"], self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            topics["slam_pose"],
            self._on_slam_pose,
            10,
        )
        amcl_pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            topics["amcl_pose"],
            self._on_amcl_pose,
            amcl_pose_qos,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, topics["map"], self._on_map, map_qos
        )
        # Nav2 AMCL публикует particle_cloud с BEST_EFFORT QoS.
        self.create_subscription(
            ParticleCloud,
            topics["particle_cloud"],
            self._on_particles,
            qos_profile_sensor_data,
        )

        self._updater = Updater(self)
        self._updater.setHardwareID("rtk2026-localization")
        self._updater.add(CovarianceTask(self, "wheel", "Wheel odometry", True))
        self._updater.add(CovarianceTask(self, "filtered", "EKF", True))
        self._updater.add(
            # /pose у SLAM Toolbox событийный: карта может исправно
            # обновляться, пока неподвижный робот не получает новую pose.
            CovarianceTask(self, "slam", "SLAM pose", False)
        )
        self._updater.add(
            CovarianceTask(
                self,
                "amcl",
                "AMCL pose",
                self.config["mode"] == "localization",
            )
        )
        self._updater.add(ConsistencyTask(self))
        self._updater.add(GroundTruthTask(self))
        self._updater.add(MapTask(self))
        self._updater.add(AmclTask(self))

    def _odometry(self, key: str, message: Odometry) -> None:
        pose = message.pose.pose
        with self._lock:
            self._motion[key] = MotionSample(
                time.monotonic_ns(),
                _stamp_ns(message),
                float(message.twist.twist.linear.x),
                float(message.twist.twist.angular.z),
            )
            self._covariances[key] = CovarianceSample(
                time.monotonic_ns(),
                _stamp_ns(message),
                message.header.frame_id.lstrip("/"),
                list(message.pose.covariance),
            )
            sample = PoseSample(
                time.monotonic_ns(),
                _stamp_ns(message),
                float(pose.position.x),
                float(pose.position.y),
                _yaw(pose.orientation),
            )
            self._poses[key] = sample
            self._initial_poses.setdefault(key, sample)

    def _on_wheel_odometry(self, message: Odometry) -> None:
        self._odometry("wheel", message)

    def _on_filtered_odometry(self, message: Odometry) -> None:
        self._odometry("filtered", message)

    def _on_ground_truth(self, message: Odometry) -> None:
        pose = message.pose.pose
        with self._lock:
            sample = PoseSample(
                time.monotonic_ns(),
                _stamp_ns(message),
                float(pose.position.x),
                float(pose.position.y),
                _yaw(pose.orientation),
            )
            self._poses["ground_truth"] = sample
            self._initial_poses.setdefault("ground_truth", sample)

    def _on_imu(self, message: Imu) -> None:
        with self._lock:
            self._motion["imu"] = MotionSample(
                time.monotonic_ns(),
                _stamp_ns(message),
                0.0,
                float(message.angular_velocity.z),
            )

    def _pose(self, key: str, message: PoseWithCovarianceStamped) -> None:
        with self._lock:
            now_ns = time.monotonic_ns()
            self._covariances[key] = CovarianceSample(
                now_ns,
                _stamp_ns(message),
                message.header.frame_id.lstrip("/"),
                list(message.pose.covariance),
            )
            if key == "slam":
                self._slam_pose_received_ns = now_ns
            else:
                self._amcl_pose_received_ns = now_ns

    def _on_slam_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._pose("slam", message)

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._pose("amcl", message)

    def _on_map(self, message: OccupancyGrid) -> None:
        count = max(1, len(message.data))
        known = sum(value >= 0 for value in message.data)
        occupied = sum(value >= 50 for value in message.data)
        with self._lock:
            self._map = {
                "receive_ns": time.monotonic_ns(),
                "frame_id": message.header.frame_id.lstrip("/"),
                "width": int(message.info.width),
                "height": int(message.info.height),
                "resolution": float(message.info.resolution),
                "known_ratio": known / count,
                "occupied_ratio": occupied / count,
            }

    def _on_particles(self, message: ParticleCloud) -> None:
        particles = list(message.particles)
        weights = [max(0.0, float(item.weight)) for item in particles]
        total = sum(weights)
        if total <= 0.0 and particles:
            weights = [1.0 / len(particles)] * len(particles)
        elif total > 0.0:
            weights = [value / total for value in weights]
        mean_x = sum(
            weight * particle.pose.position.x
            for weight, particle in zip(weights, particles)
        )
        mean_y = sum(
            weight * particle.pose.position.y
            for weight, particle in zip(weights, particles)
        )
        sigma_x = math.sqrt(
            sum(
                weight * (particle.pose.position.x - mean_x) ** 2
                for weight, particle in zip(weights, particles)
            )
        )
        sigma_y = math.sqrt(
            sum(
                weight * (particle.pose.position.y - mean_y) ** 2
                for weight, particle in zip(weights, particles)
            )
        )
        cos_yaw = sum(
            weight * math.cos(_yaw(particle.pose.orientation))
            for weight, particle in zip(weights, particles)
        )
        sin_yaw = sum(
            weight * math.sin(_yaw(particle.pose.orientation))
            for weight, particle in zip(weights, particles)
        )
        effective_size = (
            1.0 / sum(weight**2 for weight in weights) if weights else 0.0
        )
        with self._lock:
            self._particles = {
                "count": len(particles),
                "effective_size": effective_size,
                "sigma_x": sigma_x,
                "sigma_y": sigma_y,
                "yaw_concentration": math.hypot(cos_yaw, sin_yaw),
            }

    def covariance_snapshot(self, key: str) -> CovarianceSample | None:
        """Вернуть копию covariance sample."""

        with self._lock:
            sample = self._covariances.get(key)
            return None if sample is None else CovarianceSample(**vars(sample))

    def motion_snapshot(self) -> dict[str, MotionSample]:
        """Вернуть копию последних скоростей."""

        with self._lock:
            return {
                key: MotionSample(**vars(sample))
                for key, sample in self._motion.items()
            }

    def relative_pose_snapshot(self) -> dict[str, PoseSample]:
        """Вернуть движение каждого источника относительно его первой позы."""

        with self._lock:
            return {
                key: _relative_pose(sample, self._initial_poses[key])
                for key, sample in self._poses.items()
                if key in self._initial_poses
            }

    def slam_snapshot(self) -> dict[str, Any]:
        """Вернуть состояние карты и SLAM pose."""

        with self._lock:
            return {
                "map": None if self._map is None else dict(self._map),
                "pose": self._slam_pose_received_ns,
            }

    def amcl_snapshot(self) -> dict[str, Any]:
        """Вернуть состояние AMCL pose и particle cloud."""

        with self._lock:
            return {
                "pose": self._amcl_pose_received_ns,
                "particles": (
                    None if self._particles is None else dict(self._particles)
                ),
            }


def main(args=None) -> None:
    """Запустить localization monitor."""

    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

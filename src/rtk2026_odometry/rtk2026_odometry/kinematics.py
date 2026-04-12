"""
Кинематика дифференциального привода.

Чистая математика без зависимостей от ROS —
можно тестировать и использовать независимо от стека.

Система координат: правая, x вперёд, y влево, theta против часовой стрелки.
"""

import math
from dataclasses import dataclass


@dataclass
class Pose2D:
    """Поза робота в плоскости."""
    x: float     # метры
    y: float     # метры
    theta: float # радианы


@dataclass
class Twist2D:
    """Скорость робота в плоскости."""
    linear: float   # м/с, вдоль оси x
    angular: float  # рад/с, вокруг оси z


class DiffDriveKinematics:
    """
    Кинематика дифференциального привода.

    Параметры задаются один раз при создании и берутся из odometry.yaml.

    Формулы:
      d_left  = delta_left_ticks  / ticks_per_meter
      d_right = delta_right_ticks / ticks_per_meter

      d_center = (d_left + d_right) / 2        — смещение вперёд
      d_theta  = (d_right - d_left) / wheel_sep — поворот

      x     += d_center * cos(theta)   — применяем ДО обновления theta
      y     += d_center * sin(theta)
      theta += d_theta
    """

    def __init__(self, ticks_per_meter: float, wheel_separation: float) -> None:
        """
        ticks_per_meter  — тиков на метр (калибруется физически)
        wheel_separation — расстояние между колёсами, метры
        """
        self._tpm = ticks_per_meter
        self._wheel_sep = wheel_separation

    def ticks_to_twist(
        self,
        delta_left: int,
        delta_right: int,
        dt: float,
    ) -> Twist2D:
        """
        Преобразовать разницу тиков за период dt в скорость робота.

        delta_left, delta_right — разница тиков за период (тики)
        dt                      — длительность периода (секунды)
        """
        if dt <= 0.0:
            return Twist2D(linear=0.0, angular=0.0)

        d_left  = delta_left  / self._tpm
        d_right = delta_right / self._tpm

        linear  = (d_left + d_right) / 2.0 / dt
        angular = (d_right - d_left) / self._wheel_sep / dt

        return Twist2D(linear=linear, angular=angular)

    def integrate_pose(
        self,
        pose: Pose2D,
        delta_left: int,
        delta_right: int,
    ) -> Pose2D:
        """
        Интегрировать позу робота по разнице тиков.

        Порядок важен: смещение применяется с текущим theta,
        затем theta обновляется — иначе накапливается геометрическая ошибка.
        """
        d_left  = delta_left  / self._tpm
        d_right = delta_right / self._tpm

        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self._wheel_sep

        # применяем смещение с текущим theta (до обновления)
        x     = pose.x + d_center * math.cos(pose.theta)
        y     = pose.y + d_center * math.sin(pose.theta)
        theta = pose.theta + d_theta

        # нормализуем theta в [-pi, pi]
        theta = math.atan2(math.sin(theta), math.cos(theta))

        return Pose2D(x=x, y=y, theta=theta)

    def twist_to_wheel_tps(self, linear: float, angular: float):
        """
        Преобразовать скорость робота в целевые скорости колёс (тики/сек).

        Используется в base_controller для конвертации cmd_vel → WheelVelocityCommand.
        Возвращает (left_tps, right_tps).
        """
        v_left  = linear - angular * self._wheel_sep / 2.0
        v_right = linear + angular * self._wheel_sep / 2.0

        left_tps  = int(v_left  * self._tpm)
        right_tps = int(v_right * self._tpm)

        return left_tps, right_tps

"""Отправка цепочки поз в Nav2 и разбор её исхода.

Всё, что знает про действие ``navigate_through_poses``, собрано здесь: сама
цель, её отмена, перевод поз в сообщения и путь для RViz. Автомат про Nav2
не знает вовсе, а нода — только через этот класс.

Исход не возвращается вызовом, а сообщается обратными вызовами: результат
цели приходит асинхронно. Ноде они кладут событие в очередь, а не двигают
автомат — см. :mod:`rtk2026_city_nav.node`.
"""

from __future__ import annotations

import math
from collections.abc import Callable

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from rtk2026_city_nav.lane import LanePose


class Nav2Goals:
    """Клиент действия Nav2 для проезда по цепочке поз."""

    def __init__(
        self,
        node: Node,
        *,
        action_name: str,
        frame_id: str,
        server_timeout_s: float,
        on_arrived: Callable[[], None],
        on_failed: Callable[[str], None],
        path_topic: str = "~/leg_path",
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._timeout_s = server_timeout_s
        self._on_arrived = on_arrived
        self._on_failed = on_failed

        self._client = ActionClient(node, NavigateThroughPoses, action_name)
        self._path_publisher = node.create_publisher(PathMsg, path_topic, 1)
        self._goal: ClientGoalHandle | None = None

        #: Сколько целей Nav2 отклонил либо не вернул результата.
        self.rejected = 0

    def send(self, poses: tuple[LanePose, ...]) -> None:
        """Отправить цепочку поз.

        Путь публикуется до отправки, чтобы он был виден в RViz и тогда,
        когда цель не принята: смотреть на позы полезно как раз в этом случае.
        """
        stamped = tuple(self._to_pose_stamped(pose) for pose in poses)
        self._publish_path(stamped)

        if not self._client.wait_for_server(timeout_sec=self._timeout_s):
            self._node.get_logger().warn("сервер navigate_through_poses недоступен")
            self._on_failed("nav2_unavailable")
            return

        goal = NavigateThroughPoses.Goal()
        goal.poses = list(stamped)

        self._client.send_goal_async(goal).add_done_callback(self._on_response)

    def cancel(self) -> None:
        """Отменить текущую цель, если она есть."""
        if self._goal is not None:
            self._goal.cancel_goal_async()
            self._goal = None

    # -- Обратные вызовы действия -------------------------------------------

    def _on_response(self, future) -> None:
        handle: ClientGoalHandle | None = future.result()

        if handle is None or not handle.accepted:
            self.rejected += 1
            self._node.get_logger().warn("Nav2 отклонил цель")
            self._on_failed("goal_rejected")
            return

        self._goal = handle
        handle.get_result_async().add_done_callback(
            lambda done, owner=handle: self._on_result(done, owner)
        )

    def _on_result(self, future, owner: ClientGoalHandle) -> None:
        # Результат отменённой цели приходит и после того, как отправлена
        # новая. Своя она или чужая, видно по владельцу.
        if self._goal is not owner:
            return

        self._goal = None
        result = future.result()

        if result is None:
            self.rejected += 1
            self._on_failed("no_result")
            return

        status = int(result.status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._on_arrived()
            return

        self._on_failed(f"nav2_status_{status}")

    # -- Сообщения ----------------------------------------------------------

    def _to_pose_stamped(self, pose: LanePose) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self._frame_id
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.position.x = float(pose.x)
        message.pose.position.y = float(pose.y)
        message.pose.orientation.z = math.sin(0.5 * float(pose.yaw))
        message.pose.orientation.w = math.cos(0.5 * float(pose.yaw))
        return message

    def _publish_path(self, poses: tuple[PoseStamped, ...]) -> None:
        message = PathMsg()
        message.header.frame_id = self._frame_id
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.poses = list(poses)
        self._path_publisher.publish(message)
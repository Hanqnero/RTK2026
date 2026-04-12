"""
ROS2 нода: колёсная одометрия дифференциального привода.

Отвечает за:
  - подписку на /encoder_report
  - вычисление позы и скорости робота (через DiffDriveKinematics)
  - публикацию /odom (nav_msgs/Odometry)
  - публикацию TF odom -> base_link (если publish_tf: true)

Не отвечает за:
  - чтение энкодеров (rtk2026_driver)
  - управление моторами (rtk2026_driver)
  - фьюжн с IMU (robot_localization EKF — отдельный пакет)

Все параметры берутся из config/odometry.yaml.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from rtk2026_interfaces.msg import EncoderReport
from rtk2026_odometry.kinematics import DiffDriveKinematics, Pose2D


class WheelOdometryNode(Node):

    def __init__(self) -> None:
        super().__init__("wheel_odometry")

        # --- параметры из odometry.yaml ---
        self.declare_parameter("ticks_per_meter",      1268.0)
        self.declare_parameter("wheel_separation",     0.25)
        self.declare_parameter("encoder_report_topic", "encoder_report")
        self.declare_parameter("odom_topic",           "odom")
        self.declare_parameter("odom_frame",           "odom")
        self.declare_parameter("base_frame",           "base_link")
        self.declare_parameter("publish_tf",           True)

        tpm       = self.get_parameter("ticks_per_meter").value
        wheel_sep = self.get_parameter("wheel_separation").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._publish_tf = self.get_parameter("publish_tf").value

        # --- кинематика ---
        self._kinematics = DiffDriveKinematics(
            ticks_per_meter=tpm,
            wheel_separation=wheel_sep,
        )

        # --- состояние одометрии ---
        self._pose = Pose2D(x=0.0, y=0.0, theta=0.0)
        self._prev_left: int | None  = None
        self._prev_right: int | None = None
        self._prev_stamp = None

        # --- публикатор /odom ---
        odom_topic = self.get_parameter("odom_topic").value
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # --- TF broadcaster ---
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        # --- подписка на /encoder_report ---
        enc_topic = self.get_parameter("encoder_report_topic").value
        self.create_subscription(EncoderReport, enc_topic, self._on_encoder_report, 10)

        self.get_logger().info(
            f"Wheel odometry started: ticks_per_meter={tpm}, wheel_sep={wheel_sep}m"
        )

    def _on_encoder_report(self, msg: EncoderReport) -> None:
        """
        Колбэк: получаем EncoderReport, обновляем позу, публикуем /odom и TF.

        Первый вызов: инициализируем счётчики, ничего не публикуем —
        нет предыдущего значения для вычисления дельты.
        """
        stamp = msg.header.stamp
        now   = rclpy.time.Time.from_msg(stamp)

        if self._prev_left is None:
            # первый фрейм — запоминаем начальное состояние
            self._prev_left  = msg.left_count
            self._prev_right = msg.right_count
            self._prev_stamp = now
            return

        # вычисляем dt из timestamp сообщений
        dt = (now - self._prev_stamp).nanoseconds / 1e9
        if dt <= 0.0:
            # защита от дубликатов или обратного хода времени
            return

        delta_left  = msg.left_count  - self._prev_left
        delta_right = msg.right_count - self._prev_right

        # обновляем позу
        self._pose = self._kinematics.integrate_pose(
            self._pose, delta_left, delta_right
        )

        # вычисляем скорость для twist части Odometry
        twist = self._kinematics.ticks_to_twist(delta_left, delta_right, dt)

        # сохраняем для следующего вызова
        self._prev_left  = msg.left_count
        self._prev_right = msg.right_count
        self._prev_stamp = now

        self._publish_odom(stamp, twist)
        if self._publish_tf:
            self._publish_transform(stamp)

    def _publish_odom(self, stamp, twist) -> None:
        """Собрать и опубликовать nav_msgs/Odometry."""
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id  = self._base_frame

        # поза
        msg.pose.pose.position.x = self._pose.x
        msg.pose.pose.position.y = self._pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _yaw_to_quaternion(self._pose.theta)

        # скорость
        msg.twist.twist.linear.x  = twist.linear
        msg.twist.twist.angular.z = twist.angular

        self._odom_pub.publish(msg)

    def _publish_transform(self, stamp) -> None:
        """Опубликовать TF odom -> base_link."""
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id  = self._base_frame

        t.transform.translation.x = self._pose.x
        t.transform.translation.y = self._pose.y
        t.transform.translation.z = 0.0
        t.transform.rotation = _yaw_to_quaternion(self._pose.theta)

        self._tf_broadcaster.sendTransform(t)


def _yaw_to_quaternion(yaw: float):
    """Преобразовать угол yaw (рад) в quaternion для плоского движения."""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

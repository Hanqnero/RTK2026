#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import time


class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.declare_parameter('Kp', 0.0025)
        self.declare_parameter('Kd', 0.007)
        self.declare_parameter('K_heading', 0.55)  # вклад траектории линии (heading)
        self.declare_parameter('lane_topic', '/detect/lane')
        self.declare_parameter('heading_topic', '/detect/lane_heading')
        self.declare_parameter('Ki_turn', 0.0006)  # I-компонента только для режима поворота
        self.declare_parameter('i_limit', 250.0)   # антиwindup для интеграла
        self.declare_parameter('max_vel', 0.1)
        self.declare_parameter('turn_linear_vel_mps', 0.06)  # линейная скорость в резком повороте
        self.declare_parameter('turn_error_px', 90.0)  # порог перехода в режим поворота
        self.declare_parameter('lane_center_px', 500.0)  # целевой центр в пикселях (0-1000)
        self.declare_parameter('min_turn_radius_m', 0.25)  # минимальный радиус поворота
        self.declare_parameter('curvature_ref_lin_vel_mps', 0.03)  # референс v для лимита omega
        self.declare_parameter('max_ang_vel', 1.2)  # общий лимит угловой скорости
        self.declare_parameter('min_turn_omega', 0.3)  # минимальная угловая в режиме поворота

        self.add_on_set_parameters_callback(self._on_params)

        lane_topic = self.get_parameter('lane_topic').value
        self.sub_lane = self.create_subscription(
            Float64,
            lane_topic,
            self.callback_follow_lane,
            1
        )
        heading_topic = self.get_parameter('heading_topic').value
        self.sub_heading = self.create_subscription(
            Float64,
            heading_topic,
            self.callback_heading,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        # PD control related variables
        self.last_error = 0
        self.integral_error = 0.0
        self.MAX_VEL = self.get_parameter('max_vel').value
        self.Kp = self.get_parameter('Kp').value
        self.Kd = self.get_parameter('Kd').value
        self.K_heading = self.get_parameter('K_heading').value
        self.Ki_turn = self.get_parameter('Ki_turn').value
        self.i_limit = self.get_parameter('i_limit').value
        self.turn_linear_vel = self.get_parameter('turn_linear_vel_mps').value
        self.turn_error_px = self.get_parameter('turn_error_px').value
        self.lane_center = self.get_parameter('lane_center_px').value
        self.min_turn_radius = self.get_parameter('min_turn_radius_m').value
        self.curv_ref_v = self.get_parameter('curvature_ref_lin_vel_mps').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.min_turn_omega = self.get_parameter('min_turn_omega').value
        self.heading_error = 0.0
        self.heading_stamp = 0.0

        # Avoidance mode related variables
        self.avoid_active = False
        self.avoid_twist = Twist()

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'Kp':
                self.Kp = p.value
            elif p.name == 'Kd':
                self.Kd = p.value
            elif p.name == 'K_heading':
                self.K_heading = p.value
            elif p.name == 'Ki_turn':
                self.Ki_turn = p.value
            elif p.name == 'i_limit':
                self.i_limit = p.value
            elif p.name == 'max_vel':
                self.MAX_VEL = p.value
            elif p.name == 'turn_linear_vel_mps':
                self.turn_linear_vel = p.value
            elif p.name == 'turn_error_px':
                self.turn_error_px = p.value
            elif p.name == 'lane_center_px':
                self.lane_center = p.value
            elif p.name == 'min_turn_radius_m':
                self.min_turn_radius = p.value
            elif p.name == 'curvature_ref_lin_vel_mps':
                self.curv_ref_v = p.value
            elif p.name == 'max_ang_vel':
                self.max_ang_vel = p.value
            elif p.name == 'min_turn_omega':
                self.min_turn_omega = p.value
        return SetParametersResult(successful=True)

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_heading(self, heading_msg):
        self.heading_error = float(heading_msg.data)
        self.heading_stamp = time.monotonic()

    def callback_follow_lane(self, desired_center):
        if self.avoid_active:
            return

        error = desired_center.data - self.lane_center
        d_error = (error - self.last_error)
        in_turn_mode = abs(error) >= float(self.turn_error_px)

        # Интеграл накапливаем только в повороте (где нужен доворот),
        # на прямой сбрасываем, чтобы не тянуть bias.
        if in_turn_mode:
            self.integral_error += error
            if self.integral_error > self.i_limit:
                self.integral_error = self.i_limit
            elif self.integral_error < -self.i_limit:
                self.integral_error = -self.i_limit
        else:
            self.integral_error = 0.0

        # Drop stale heading input to avoid "ghost" steering.
        if (time.monotonic() - self.heading_stamp) > 0.35:
            heading_term = 0.0
        else:
            heading_term = self.K_heading * self.heading_error

        angular_z = (
            self.Kp * error
            + self.Kd * d_error
            + heading_term
            + (self.Ki_turn * self.integral_error if in_turn_mode else 0.0)
        )
        self.last_error = error

        twist = Twist()
        if in_turn_mode:
            twist.linear.x = min(self.turn_linear_vel, self.MAX_VEL)
        else:
            twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), self.MAX_VEL)

        # Base PD yaw command with sign aligned to robot convention
        yaw_cmd = -max(min(angular_z, self.max_ang_vel), -self.max_ang_vel)

        # В резком повороте не даем угловой скорости стать слишком маленькой.
        if in_turn_mode and abs(yaw_cmd) < self.min_turn_omega:
            yaw_cmd = self.min_turn_omega if error < 0 else -self.min_turn_omega

        # Hard limit curvature: |omega| <= |v_ref| / Rmin
        # Use reference speed floor so robot can still rotate sufficiently
        # in tight turns when linear speed temporarily drops.
        rmin = max(float(self.min_turn_radius), 1e-3)
        v_ref = max(abs(twist.linear.x), float(self.curv_ref_v))
        omega_lim = v_ref / rmin
        if yaw_cmd > omega_lim:
            yaw_cmd = omega_lim
        elif yaw_cmd < -omega_lim:
            yaw_cmd = -omega_lim

        twist.angular.z = yaw_cmd
        self.pub_cmd_vel.publish(twist)

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

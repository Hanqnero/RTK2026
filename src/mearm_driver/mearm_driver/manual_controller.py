#!/usr/bin/env python3
"""
Manual Controller for MeArm - allows keyboard/gamepad control of manipulator
Publishes servo commands for interactive control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
import sys
import os


class MeArmManualController(Node):
    """
    Manual control interface for MeArm manipulator.
    
    Supports:
    1. Keyboard input (manual mode)
    2. Gamepad/Joystick (via Joy message)
    
    Publishes:
    - /mearm/servo_commands: Normalized servo positions
    - /mearm/preset_select: String with preset name
    """
    
    def __init__(self, use_gamepad: bool = True):
        super().__init__('mearm_manual_controller')
        
        self.declare_parameter('gamepad_enabled', use_gamepad)
        self.declare_parameter('control_mode', 'gamepad')  # 'gamepad' or 'keyboard'
        
        self.control_mode = self.get_parameter('control_mode').value
        self.servo_positions = [0.0, 0.0, 0.0, 0.0]  # Current servo positions [-1, 1]
        self.servo_speed = 0.1  # Change speed per update
        self.base_linear_speed = 0.35
        self.base_angular_speed = 0.8
        
        # Publishers
        self.servo_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/mearm/servo_commands',
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriptions
        if 'gamepad' in self.control_mode.lower():
            self.joy_sub = self.create_subscription(
                Joy,
                '/joy',
                self._joy_callback,
                10
            )
        
        # Timer for periodic updates
        self.create_timer(0.1, self._publish_servo_commands)
        
        self.get_logger().info(
            f'MeArm Manual Controller started in {self.control_mode} mode'
        )
        
        if 'keyboard' in self.control_mode.lower():
            self._start_keyboard_thread()
    
    def _joy_callback(self, msg: Joy):
        """
        Handle gamepad input
        
        Button mapping:
        - Right stick X/Y: Joint 1/2 control
        - Left stick X/Y: Joint 0/3 control
        - LT/RT: Fine adjustment
        - A/B: Preset positions
        """
        # Get analog values from gamepad
        if len(msg.axes) >= 4:
            # Right stick: Joint 1 (Y) and Joint 2 (X)
            self.servo_positions[1] = msg.axes[3]  # Right stick Y (inverted)
            self.servo_positions[2] = msg.axes[2]  # Right stick X
            
            # Left stick: Joint 0 (X) and fine control (Y)
            self.servo_positions[0] = msg.axes[0]  # Left stick X (base rotation)
            
            # Gripper control with triggers
            if len(msg.axes) >= 5:
                trigger_value = (msg.axes[4] + msg.axes[5]) / 2  # Average both triggers
                self.servo_positions[3] = trigger_value
    
    def _start_keyboard_thread(self):
        """Start keyboard input thread"""
        thread = threading.Thread(target=self._keyboard_input, daemon=True)
        thread.start()
    
    def _keyboard_input(self):
        """Keyboard control interface"""
        self.get_logger().info(
            '\nKeyboard Control:\n'
            '  Q/A: Joint 0 (base) +/-\n'
            '  W/S: Joint 1 (right arm) +/-\n'
            '  E/D: Joint 2 (left arm) +/-\n'
            '  R/F: Joint 3 (gripper) +/-\n'
            '  I/K: Robot forward/backward\n'
            '  J/L: Robot turn left/right\n'
            '  Space: Reset to center\n'
            '  X or Esc: Quit\n'
        )

        if not sys.stdin.isatty():
            self.get_logger().error(
                'Keyboard mode requires an interactive terminal (TTY).'
            )
            return

        try:
            while rclpy.ok():
                key = self._read_key().lower()
                
                if key in ('x', '\x1b'):
                    rclpy.shutdown()
                    break
                elif key == 'a':
                    self.servo_positions[0] -= self.servo_speed
                elif key == 'q':
                    self.servo_positions[0] += self.servo_speed
                elif key == 's':
                    self.servo_positions[1] -= self.servo_speed
                elif key == 'w':
                    self.servo_positions[1] += self.servo_speed
                elif key == 'd':
                    self.servo_positions[2] -= self.servo_speed
                elif key == 'e':
                    self.servo_positions[2] += self.servo_speed
                elif key == 'f':
                    self.servo_positions[3] -= self.servo_speed
                elif key == 'r':
                    self.servo_positions[3] += self.servo_speed
                elif key == ' ':
                    self.servo_positions = [0.0, 0.0, 0.0, 0.0]
                    self._publish_base_command(0.0, 0.0)
                elif key == 'i':
                    self._publish_base_command(self.base_linear_speed, 0.0)
                elif key == 'k':
                    self._publish_base_command(-self.base_linear_speed, 0.0)
                elif key == 'j':
                    self._publish_base_command(0.0, self.base_angular_speed)
                elif key == 'l':
                    self._publish_base_command(0.0, -self.base_angular_speed)
                
                # Clamp values to [-1, 1]
                self.servo_positions = [max(-1.0, min(1.0, pos)) for pos in self.servo_positions]
                if key in ('q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 'i', 'j', 'k', 'l', ' '):
                    values = ', '.join(f'{pos:+.1f}' for pos in self.servo_positions)
                    self.get_logger().info(
                        f'Key {key!r} accepted; joints=[{values}]'
                    )
        
        except Exception as e:
            self.get_logger().error(f'Keyboard input error: {e}')

    def _publish_base_command(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    @staticmethod
    def _read_key() -> str:
        """Read one key without requiring Enter on Windows or POSIX."""
        if os.name == 'nt':
            import msvcrt
            return msvcrt.getwch()

        import termios
        import tty

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    def _publish_servo_commands(self):
        """Publish current servo positions"""
        msg = Float32MultiArray()
        msg.data = self.servo_positions
        self.servo_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Determine control mode from arguments
    use_gamepad = 'gamepad' in sys.argv if len(sys.argv) > 1 else True
    
    controller = MeArmManualController(use_gamepad=use_gamepad)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

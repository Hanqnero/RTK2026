#!/usr/bin/env python3
"""
MeArm Servo Controller Node - ROS2 driver for Arduino-based servo control
Handles communication with Arduino for 4-servo MeArm arm control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Float64, Int16MultiArray
from sensor_msgs.msg import JointState
import serial
import time
from typing import Optional, List
import yaml


class MeArmServoController(Node):
    """
    Controls MeArm manipulator via Arduino serial communication.
    
    Subscribes to:
    - /mearm/servo_commands: Float32MultiArray with normalized servo angles [-1.0, 1.0]
    - /mearm/raw_servo_commands: Int16MultiArray with raw servo PWM values [0-180]
    
    Publishes to:
    - /mearm/servo_feedback: Int16MultiArray with actual servo positions
    - /joint_states: JointState for visualization
    """
    
    def __init__(self):
        super().__init__('mearm_servo_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('servo_config_file', '')
        self.declare_parameter('num_servos', 4)
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('dry_run', False)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.config_file = self.get_parameter('servo_config_file').value
        self.num_servos = self.get_parameter('num_servos').value
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.publish_joint_states = self.get_parameter('publish_joint_states').value
        self.dry_run = self.get_parameter('dry_run').value
        
        # Serial communication
        self.serial_conn: Optional[serial.Serial] = None
        self.servo_positions = [90] * self.num_servos  # Mid position for all servos
        self.servo_min_max = {
            0: (45, 135),      # Base rotation
            1: (0, 180),       # Right arm
            2: (0, 180),       # Left arm
            3: (0, 180),       # Gripper
        }
        
        # Load servo configuration if provided
        if self.config_file:
            self._load_servo_config()
        
        # Subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.servo_cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/mearm/servo_commands',
            self._servo_cmd_callback,
            qos_profile
        )
        
        self.raw_servo_sub = self.create_subscription(
            Int16MultiArray,
            '/mearm/raw_servo_commands',
            self._raw_servo_callback,
            qos_profile
        )
        
        # Publishers
        self.servo_feedback_pub = self.create_publisher(
            Int16MultiArray,
            '/mearm/servo_feedback',
            10
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.gz_position_pubs = [
            self.create_publisher(
                Float64,
                f'/model/rtk2026/joint/joint_{joint_id}/0/cmd_pos',
                10
            )
            for joint_id in range(self.num_servos)
        ]
        
        # Timer for periodic updates
        self.create_timer(1.0 / self.publish_freq, self._publish_states)
        
        # Initialize serial connection
        if not self.dry_run:
            self._init_serial()
        
        self.get_logger().info(
            f'MeArm Servo Controller initialized on {self.serial_port} '
            f'at {self.serial_baud} baud (dry_run={self.dry_run})'
        )
    
    def _init_serial(self) -> bool:
        """Initialize serial connection to Arduino"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baud,
                timeout=1.0
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to {self.serial_port}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            return False
    
    def _load_servo_config(self):
        """Load servo configuration from YAML file"""
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
                if 'servo_limits' in config:
                    for servo_id, limits in config['servo_limits'].items():
                        self.servo_min_max[servo_id] = (limits['min'], limits['max'])
            self.get_logger().info(f'Loaded servo config from {self.config_file}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load servo config: {e}')
    
    def _servo_cmd_callback(self, msg: Float32MultiArray):
        """
        Handle normalized servo commands [-1.0, 1.0]
        Converts to [0-180] range and sends to Arduino
        """
        if len(msg.data) != self.num_servos:
            self.get_logger().warn(
                f'Expected {self.num_servos} servo commands, got {len(msg.data)}'
            )
            return
        
        servo_values = []
        for i, value in enumerate(msg.data):
            # Clamp value to [-1.0, 1.0]
            clamped = max(-1.0, min(1.0, value))
            min_angle, max_angle = self.servo_min_max[i]
            servo_angle = int(round(
                min_angle + (clamped + 1.0) * (max_angle - min_angle) / 2.0
            ))
            servo_values.append(servo_angle)
        
        self._send_servo_command(servo_values)
    
    def _raw_servo_callback(self, msg: Int16MultiArray):
        """Handle raw servo commands [0-180]"""
        if len(msg.data) != self.num_servos:
            self.get_logger().warn(
                f'Expected {self.num_servos} servo commands, got {len(msg.data)}'
            )
            return
        
        servo_values = [
            max(self.servo_min_max[i][0], min(self.servo_min_max[i][1], int(val)))
            for i, val in enumerate(msg.data)
        ]
        self._send_servo_command(servo_values)
    
    def _send_servo_command(self, servo_values: List[int]):
        """Send servo command to Arduino"""
        self.servo_positions = servo_values[:self.num_servos]
        self._publish_gazebo_positions()

        if self.dry_run:
            return

        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection not available')
            return
        
        try:
            # Arduino protocol: 4 bytes for 4 servos
            # Format: servo1 servo2 servo3 servo4
            command = bytes([0xFF] + servo_values[:self.num_servos])  # 0xFF = start marker
            self.serial_conn.write(command)
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
            # Try to reconnect
            self._init_serial()

    def _joint_positions(self) -> List[float]:
        joint_ranges = [
            (-0.7853, 0.7853),
            (-1.57, 1.0),
            (0.0, 1.57),
            (0.0, 1.57),
        ]
        positions = []
        for servo_angle, (min_rad, max_rad) in zip(
            self.servo_positions, joint_ranges
        ):
            normalized = (servo_angle - 90) / 90
            positions.append(
                min_rad + (normalized + 1) * (max_rad - min_rad) / 2
            )
        return positions

    def _publish_gazebo_positions(self):
        for publisher, position in zip(
            self.gz_position_pubs, self._joint_positions()
        ):
            msg = Float64()
            msg.data = position
            publisher.publish(msg)
    
    def _publish_states(self):
        """Publish servo feedback and joint states"""
        # Publish raw servo feedback
        feedback_msg = Int16MultiArray()
        feedback_msg.data = self.servo_positions
        self.servo_feedback_pub.publish(feedback_msg)
        
        # Publish as JointState for visualization
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint_0', 'joint_1', 'joint_2', 'joint_3']
        
        # Convert servo angles to radians
        # Assuming servo range is 0-180 degrees, map to joint limits
        joint_ranges = [
            (-0.7853, 0.7853),   # joint_0: ±45° (base rotation)
            (-1.57, 1.0),         # joint_1: -90° to 57° (right arm)
            (0.0, 1.57),          # joint_2: 0° to 90° (left arm)
            (0.0, 1.57),          # joint_3: 0° to 90° (gripper)
        ]
        
        joint_state_msg.position = []
        for servo_angle, (min_rad, max_rad) in zip(self.servo_positions, joint_ranges):
            # Normalize servo angle to [-1, 1]
            normalized = (servo_angle - 90) / 90
            # Map to joint range
            joint_rad = min_rad + (normalized + 1) * (max_rad - min_rad) / 2
            joint_state_msg.position.append(joint_rad)
        
        joint_state_msg.velocity = [0.0] * self.num_servos
        joint_state_msg.effort = [0.0] * self.num_servos
        
        if self.publish_joint_states:
            self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = MeArmServoController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if controller.serial_conn:
            controller.serial_conn.close()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

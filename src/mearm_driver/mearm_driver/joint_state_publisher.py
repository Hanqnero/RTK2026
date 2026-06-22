#!/usr/bin/env python3
"""
Joint State Publisher for MeArm - maintains and publishes arm joint states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
import math


class MeArmJointStatePublisher(Node):
    """Publishes joint states for MeArm visualization"""
    
    def __init__(self):
        super().__init__('mearm_joint_state_publisher')
        
        # Joint configuration
        self.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3']
        self.joint_limits = [
            (-45, 45),    # joint_0: Base rotation ±45°
            (-90, 57),    # joint_1: Right arm -90° to 57°
            (0, 90),      # joint_2: Left arm 0° to 90°
            (0, 90),      # joint_3: Gripper 0° to 90°
        ]
        
        self.servo_angles = [90, 90, 90, 90]  # Current servo positions in degrees
        
        # Subscription to servo feedback
        self.servo_feedback_sub = self.create_subscription(
            Int16MultiArray,
            '/mearm/servo_feedback',
            self._servo_feedback_callback,
            10
        )
        
        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Timer for periodic publishing
        self.create_timer(0.05, self._publish_joint_state)
        
        self.get_logger().info('MeArm Joint State Publisher initialized')
    
    def _servo_feedback_callback(self, msg: Int16MultiArray):
        """Update servo angles from feedback"""
        if len(msg.data) == 4:
            self.servo_angles = list(msg.data)
    
    def _degrees_to_radians(self, degrees: float) -> float:
        """Convert degrees to radians"""
        return math.radians(degrees)
    
    def _servo_to_joint_position(self, servo_id: int, servo_angle: float) -> float:
        """Convert servo angle to joint position in radians"""
        min_deg, max_deg = self.joint_limits[servo_id]
        
        # Servo range is typically 0-180 degrees
        # Map to joint limits
        normalized = (servo_angle - 90) / 90  # Normalize servo to [-1, 1]
        
        # Map to joint range
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        
        joint_rad = min_rad + (normalized + 1) * (max_rad - min_rad) / 2
        
        return max(min_rad, min(max_rad, joint_rad))
    
    def _publish_joint_state(self):
        """Publish current joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.name = self.joint_names
        msg.position = []
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        # Convert servo angles to joint positions
        for servo_id, servo_angle in enumerate(self.servo_angles):
            joint_position = self._servo_to_joint_position(servo_id, servo_angle)
            msg.position.append(joint_position)
        
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    publisher = MeArmJointStatePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Test script for GR00T ROS integration
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
import threading
import time


class GR00TTestNode(Node):
    """Test node that publishes fake sensor data for testing GR00T integration"""
    
    def __init__(self):
        super().__init__('gr00t_test_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.lang_pub = self.create_publisher(String, '/language_instruction', 10)
        
        # Create timers
        self.image_timer = self.create_timer(0.1, self.publish_fake_image)  # 10 Hz
        self.joint_timer = self.create_timer(0.02, self.publish_fake_joint_state)  # 50 Hz
        self.lang_timer = self.create_timer(5.0, self.publish_language_instruction)  # Every 5 seconds
        
        self.get_logger().info('GR00T test node started - publishing fake sensor data')
        
        # Test data
        self.joint_names = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex', 
            'wrist_flex', 'wrist_roll', 'gripper'
        ]
        self.time_offset = 0.0
    
    def publish_fake_image(self):
        """Publish a fake camera image"""
        # Create a simple test pattern
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Add some visual pattern
        cv2.rectangle(img, (100, 100), (500, 350), (0, 255, 0), 2)
        cv2.circle(img, (320, 240), 50, (255, 0, 0), -1)
        
        # Add timestamp text
        timestamp = self.get_clock().now().nanoseconds / 1e9
        cv2.putText(img, f'Time: {timestamp:.2f}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Convert to ROS message
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        
        self.image_pub.publish(img_msg)
    
    def publish_fake_joint_state(self):
        """Publish fake joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.name = self.joint_names
        
        # Generate smooth sinusoidal joint positions
        self.time_offset += 0.02
        positions = []
        velocities = []
        
        for i, name in enumerate(self.joint_names):
            if 'gripper' in name:
                # Gripper position (0-1)
                pos = 0.5 + 0.3 * np.sin(self.time_offset * 0.5 + i)
                vel = 0.3 * 0.5 * np.cos(self.time_offset * 0.5 + i)
            else:
                # Joint position (radians)
                pos = 0.5 * np.sin(self.time_offset + i * 0.5)
                vel = 0.5 * np.cos(self.time_offset + i * 0.5)
            
            positions.append(pos)
            velocities.append(vel)
        
        msg.position = positions
        msg.velocity = velocities
        msg.effort = [0.0] * len(self.joint_names)  # No effort feedback
        
        self.joint_state_pub.publish(msg)
    
    def publish_language_instruction(self):
        """Publish a language instruction"""
        instructions = [
            "Pick up the red object and place it in the box",
            "Navigate to the table and clean the surface", 
            "Grab the bottle and pour water into the cup",
            "Move the robot to the charging station",
            "Organize the objects on the shelf"
        ]
        
        # Cycle through instructions
        idx = int(self.time_offset / 5.0) % len(instructions)
        instruction = instructions[idx]
        
        msg = String()
        msg.data = instruction
        
        self.lang_pub.publish(msg)
        self.get_logger().info(f'Published instruction: {instruction}')


def main(args=None):
    rclpy.init(args=args)
    
    node = GR00TTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

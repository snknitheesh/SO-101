#!/usr/bin/env python3
"""
GR00T Inference Client ROS Node

This node provides a ROS interface to the GR00T inference server.
It subscribes to robot observations and publishes actions.
"""

import sys
import os
from typing import Dict, Any, List

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge

# Add the parent directory to Python path to import gr00t modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from gr00t.eval.service import ExternalRobotInferenceClient

from gr00t_msgs.srv import GetAction
from gr00t_msgs.msg import RobotObservation, RobotAction
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String


class GR00TInferenceClientNode(Node):
    """ROS node that communicates with GR00T inference server"""
    
    def __init__(self):
        super().__init__('gr00t_inference_client')
        
        # Declare parameters
        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('camera_topics', ['/camera/image_raw'])
        self.declare_parameter('camera_names', ['ego_view'])
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('language_instruction_topic', '/language_instruction')
        self.declare_parameter('action_topic', '/robot_action')
        self.declare_parameter('action_horizon', 8)
        self.declare_parameter('control_frequency', 50.0)
        
        # Get parameters
        self.server_host = self.get_parameter('server_host').get_parameter_value().string_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.camera_names = self.get_parameter('camera_names').get_parameter_value().string_array_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.language_instruction_topic = self.get_parameter('language_instruction_topic').get_parameter_value().string_value
        self.action_topic = self.get_parameter('action_topic').get_parameter_value().string_value
        self.action_horizon = self.get_parameter('action_horizon').get_parameter_value().integer_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # Validate camera configuration
        if len(self.camera_topics) != len(self.camera_names):
            self.get_logger().error('Number of camera topics must match number of camera names')
            raise ValueError('Camera configuration mismatch')
        
        self.get_logger().info(f'Connecting to GR00T server at {self.server_host}:{self.server_port}')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Initialize inference client
        self.inference_client = ExternalRobotInferenceClient(
            host=self.server_host,
            port=self.server_port
        )
        
        # Check server connection
        try:
            if self.inference_client.ping():
                self.get_logger().info('Successfully connected to GR00T inference server')
            else:
                self.get_logger().warn('Failed to ping GR00T inference server')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to server: {str(e)}')
        
        # Initialize data storage
        self.latest_images: Dict[str, np.ndarray] = {}
        self.latest_joint_state = None
        self.latest_language_instruction = "Follow the instructions"
        self.current_action_chunk = None
        self.action_index = 0
        
        # Create subscribers
        self.image_subscribers = []
        for topic, name in zip(self.camera_topics, self.camera_names):
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self.image_callback(msg, camera_name),
                10
            )
            self.image_subscribers.append(sub)
            self.get_logger().info(f'Subscribed to camera: {topic} -> {name}')
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10
        )
        
        self.language_instruction_subscriber = self.create_subscription(
            String,
            self.language_instruction_topic,
            self.language_instruction_callback,
            10
        )
        
        # Create publishers
        self.action_publisher = self.create_publisher(
            RobotAction,
            self.action_topic,
            10
        )
        
        # Create service client for direct inference requests
        self.get_action_client = self.create_client(GetAction, 'get_action')
        
        # Create timer for action execution
        self.action_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.action_timer_callback
        )
        
        self.get_logger().info('GR00T inference client node initialized')
    
    def image_callback(self, msg: Image, camera_name: str):
        """Store latest image from camera"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
            self.latest_images[camera_name] = cv_image
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image from {camera_name}: {str(e)}')
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state"""
        self.latest_joint_state = msg
    
    def language_instruction_callback(self, msg: String):
        """Store latest language instruction"""
        self.latest_language_instruction = msg.data
        self.get_logger().info(f'Updated language instruction: {self.latest_language_instruction}')
        # Reset action chunk when instruction changes
        self.current_action_chunk = None
        self.action_index = 0
    
    def action_timer_callback(self):
        """Main control loop - get and execute actions"""
        try:
            # Check if we have all required data
            if not self._has_required_data():
                return
            
            # Get new action chunk if needed
            if self.current_action_chunk is None or self.action_index >= self.action_horizon:
                self.current_action_chunk = self._get_action_chunk()
                self.action_index = 0
            
            # Execute current action from chunk
            if self.current_action_chunk is not None:
                current_action = self._extract_action_from_chunk(self.action_index)
                self.action_publisher.publish(current_action)
                self.action_index += 1
                
        except Exception as e:
            self.get_logger().error(f'Error in action timer: {str(e)}')
    
    def _has_required_data(self) -> bool:
        """Check if we have all required sensor data"""
        if len(self.latest_images) != len(self.camera_names):
            return False
        if self.latest_joint_state is None:
            return False
        return True
    
    def _get_action_chunk(self) -> RobotAction:
        """Get new action chunk from inference server"""
        try:
            # Create observation message
            obs_msg = self._create_observation_message()
            
            # Call inference via ZMQ (faster)
            obs_dict = self._ros_to_gr00t_observation(obs_msg)
            action_result = self.inference_client.get_action(obs_dict)
            action_msg = self._gr00t_to_ros_action(action_result)
            
            self.get_logger().debug('Retrieved new action chunk')
            return action_msg
            
        except Exception as e:
            self.get_logger().error(f'Failed to get action chunk: {str(e)}')
            return None
    
    def _create_observation_message(self) -> RobotObservation:
        """Create ROS observation message from current sensor data"""
        obs_msg = RobotObservation()
        obs_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add images
        obs_msg.images = []
        obs_msg.camera_names = []
        for camera_name in self.camera_names:
            if camera_name in self.latest_images:
                image_msg = self.cv_bridge.cv2_to_imgmsg(
                    self.latest_images[camera_name], 
                    "rgb8"
                )
                obs_msg.images.append(image_msg)
                obs_msg.camera_names.append(camera_name)
        
        # Add joint state
        if self.latest_joint_state:
            obs_msg.joint_positions = list(self.latest_joint_state.position)
            obs_msg.joint_velocities = list(self.latest_joint_state.velocity)
        
        # Add language instruction
        obs_msg.language_instruction = self.latest_language_instruction
        
        return obs_msg
    
    def _ros_to_gr00t_observation(self, ros_obs: RobotObservation) -> Dict[str, Any]:
        """Convert ROS observation to GR00T format"""
        obs_dict = {}
        
        # Convert images
        for i, (image_msg, camera_name) in enumerate(zip(ros_obs.images, ros_obs.camera_names)):
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
                # Add batch dimension and ensure correct format
                cv_image = cv_image[np.newaxis, :, :, :]  # (1, H, W, 3)
                obs_dict[f"video.{camera_name}"] = cv_image
            except Exception as e:
                self.get_logger().warn(f'Failed to convert image {camera_name}: {str(e)}')
        
        # Add robot state
        if len(ros_obs.joint_positions) > 0:
            # Split into different state components based on your robot configuration
            joint_positions = np.array(ros_obs.joint_positions)[np.newaxis, :]  # (1, n_joints)
            
            # Example for dual arm robot - adapt this to your robot configuration
            n_joints = len(ros_obs.joint_positions)
            if n_joints >= 14:  # Dual arm example
                obs_dict["state.left_arm"] = joint_positions[:, :7]
                obs_dict["state.right_arm"] = joint_positions[:, 7:14]
                if n_joints > 14:
                    obs_dict["state.gripper"] = joint_positions[:, 14:]
            elif n_joints >= 7:  # Single arm example
                obs_dict["state.single_arm"] = joint_positions[:, :6]
                obs_dict["state.gripper"] = joint_positions[:, 6:]
            else:
                obs_dict["state.joint_positions"] = joint_positions
        
        # Add language instruction
        if ros_obs.language_instruction:
            obs_dict["annotation.human.action.task_description"] = [ros_obs.language_instruction]
        
        return obs_dict
    
    def _gr00t_to_ros_action(self, gr00t_action: Dict[str, Any]) -> RobotAction:
        """Convert GR00T action to ROS format"""
        ros_action = RobotAction()
        ros_action.header.stamp = self.get_clock().now().to_msg()
        
        # Extract action data
        action_keys = []
        joint_actions = []
        gripper_actions = []
        additional_actions = []
        
        for key, value in gr00t_action.items():
            action_keys.append(key)
            if 'arm' in key or 'joint' in key:
                if hasattr(value, 'flatten'):
                    joint_actions.extend(value.flatten().tolist())
                else:
                    joint_actions.extend([float(value)])
            elif 'gripper' in key:
                if hasattr(value, 'flatten'):
                    gripper_actions.extend(value.flatten().tolist())
                else:
                    gripper_actions.extend([float(value)])
            else:
                if hasattr(value, 'flatten'):
                    additional_actions.extend(value.flatten().tolist())
                else:
                    additional_actions.extend([float(value)])
        
        ros_action.action_keys = action_keys
        ros_action.joint_actions = joint_actions
        ros_action.gripper_actions = gripper_actions
        ros_action.additional_actions = additional_actions
        ros_action.is_action_chunk = True
        
        # Set action horizon
        if gr00t_action:
            first_action = next(iter(gr00t_action.values()))
            if hasattr(first_action, 'shape') and len(first_action.shape) > 0:
                ros_action.action_horizon = int(first_action.shape[0])
            else:
                ros_action.action_horizon = 1
        
        return ros_action
    
    def _extract_action_from_chunk(self, index: int) -> RobotAction:
        """Extract a single action from the action chunk"""
        if self.current_action_chunk is None:
            return RobotAction()
        
        # Create new action message for single timestep
        action_msg = RobotAction()
        action_msg.header.stamp = self.get_clock().now().to_msg()
        action_msg.action_keys = self.current_action_chunk.action_keys
        action_msg.is_action_chunk = False
        action_msg.action_horizon = 1
        
        # Extract actions for this timestep
        horizon = self.current_action_chunk.action_horizon
        n_joint_actions = len(self.current_action_chunk.joint_actions) // horizon
        n_gripper_actions = len(self.current_action_chunk.gripper_actions) // horizon
        n_additional_actions = len(self.current_action_chunk.additional_actions) // horizon
        
        start_joint = index * n_joint_actions
        end_joint = start_joint + n_joint_actions
        action_msg.joint_actions = self.current_action_chunk.joint_actions[start_joint:end_joint]
        
        start_gripper = index * n_gripper_actions
        end_gripper = start_gripper + n_gripper_actions
        action_msg.gripper_actions = self.current_action_chunk.gripper_actions[start_gripper:end_gripper]
        
        start_additional = index * n_additional_actions
        end_additional = start_additional + n_additional_actions
        action_msg.additional_actions = self.current_action_chunk.additional_actions[start_additional:end_additional]
        
        return action_msg


def main(args=None):
    """Main entry point for the node"""
    rclpy.init(args=args)
    
    try:
        node = GR00TInferenceClientNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

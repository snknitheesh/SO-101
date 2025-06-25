#!/usr/bin/env python3
"""
GR00T Inference Server ROS Node

This node wraps the existing GR00T RobotInferenceServer to provide ROS integration.
It loads a finetuned model and serves inference requests via both ROS services and ZMQ.
"""

import sys
import os
import threading
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
import cv2
from cv_bridge import CvBridge

# Add the parent directory to Python path to import gr00t modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))))

from gr00t.model.policy import Gr00tPolicy
from gr00t.eval.robot import RobotInferenceServer
from gr00t.experiment.data_config import DATA_CONFIG_MAP
from gr00t.data.embodiment_tags import EMBODIMENT_TAG_MAPPING

from gr00t_msgs.srv import GetAction
from gr00t_msgs.msg import RobotObservation, RobotAction

try:
    from functools import partial
    import os
    # Import TensorRT setup functions if available
    from deployment_scripts.trt_model_forward import setup_tensorrt_engines
    TENSORRT_AVAILABLE = True
except ImportError:
    TENSORRT_AVAILABLE = False


class GR00TInferenceServerNode(Node):
    """ROS node that runs GR00T inference server"""
    
    def __init__(self):
        super().__init__('gr00t_inference_server')
        
        # Declare parameters
        self.declare_parameter('model_path', 'nvidia/GR00T-N1.5-3B')
        self.declare_parameter('embodiment_tag', 'gr1')
        self.declare_parameter('data_config', 'fourier_gr1_arms_only')
        self.declare_parameter('denoising_steps', 4)
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('inference_mode', 'pytorch')  # 'pytorch' or 'tensorrt'
        self.declare_parameter('trt_engine_path', 'gr00t_engine')
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.embodiment_tag = self.get_parameter('embodiment_tag').get_parameter_value().string_value
        self.data_config = self.get_parameter('data_config').get_parameter_value().string_value
        self.denoising_steps = self.get_parameter('denoising_steps').get_parameter_value().integer_value
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.inference_mode = self.get_parameter('inference_mode').get_parameter_value().string_value
        self.trt_engine_path = self.get_parameter('trt_engine_path').get_parameter_value().string_value
        
        self.get_logger().info(f'Loading GR00T model: {self.model_path}')
        self.get_logger().info(f'Embodiment tag: {self.embodiment_tag}')
        self.get_logger().info(f'Data config: {self.data_config}')
        self.get_logger().info(f'Inference mode: {self.inference_mode}')
        if self.inference_mode == 'tensorrt':
            self.get_logger().info(f'TensorRT engine path: {self.trt_engine_path}')
        
        # Initialize CV bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Load the policy
        self.policy = self._load_policy()
        
        # Create ROS service
        self.get_action_service = self.create_service(
            GetAction, 
            'get_action', 
            self.get_action_callback
        )
        
        # Start ZMQ server in background thread
        self.zmq_server = RobotInferenceServer(
            self.policy, 
            port=self.server_port
        )
        self.zmq_thread = threading.Thread(target=self.zmq_server.run, daemon=True)
        self.zmq_thread.start()
        
        self.get_logger().info(f'GR00T inference server started on port {self.server_port}')
        self.get_logger().info('ROS service available at: /get_action')
    
    def _load_policy(self) -> Gr00tPolicy:
        """Load the GR00T policy with specified configuration"""
        try:
            # Get data configuration
            if self.data_config in DATA_CONFIG_MAP:
                data_config = DATA_CONFIG_MAP[self.data_config]
                modality_config = data_config.modality_config()
                modality_transform = data_config.transform()
            else:
                self.get_logger().warn(f'Unknown data config: {self.data_config}, using default')
                data_config = DATA_CONFIG_MAP['fourier_gr1_arms_only']
                modality_config = data_config.modality_config()
                modality_transform = data_config.transform()
            
            # Load policy
            policy = Gr00tPolicy(
                model_path=self.model_path,
                modality_config=modality_config,
                modality_transform=modality_transform,
                embodiment_tag=self.embodiment_tag,
                denoising_steps=self.denoising_steps,
                device=self.device,
            )
            
            self.get_logger().info('Policy loaded successfully')
            
            # Setup TensorRT if requested
            if self.inference_mode == 'tensorrt':
                self._setup_tensorrt(policy)
                self.get_logger().info('TensorRT engines loaded successfully')
            
            return policy
            
        except Exception as e:
            self.get_logger().error(f'Failed to load policy: {str(e)}')
            raise
    
    def _setup_tensorrt(self, policy: Gr00tPolicy):
        """Setup TensorRT engines for the policy"""
        if not TENSORRT_AVAILABLE:
            raise ImportError("TensorRT modules not available. Please ensure deployment_scripts are accessible.")
        
        if not os.path.exists(self.trt_engine_path):
            raise FileNotFoundError(f"TensorRT engine path not found: {self.trt_engine_path}")
        
        required_engines = [
            "vit.engine", "llm.engine", "vlln_vl_self_attention.engine",
            "action_encoder.engine", "action_decoder.engine", "DiT.engine", "state_encoder.engine"
        ]
        
        missing_engines = []
        for engine in required_engines:
            engine_path = os.path.join(self.trt_engine_path, engine)
            if not os.path.exists(engine_path):
                missing_engines.append(engine)
        
        if missing_engines:
            raise FileNotFoundError(f"Missing TensorRT engines: {missing_engines}. Please build engines first using deployment_scripts/export_onnx.py and deployment_scripts/build_engine.sh")
        
        self.get_logger().info("Setting up TensorRT engines...")
        setup_tensorrt_engines(policy, self.trt_engine_path)
        self.get_logger().info("TensorRT setup complete")
    
    def get_action_callback(self, request, response):
        """Handle ROS service requests for action inference"""
        try:
            # Convert ROS message to GR00T observation format
            obs_dict = self._ros_to_gr00t_observation(request.observation)
            
            # Get action from policy
            action_result = self.policy.get_action(obs_dict)
            
            # Convert result to ROS message
            response.action = self._gr00t_to_ros_action(action_result)
            response.success = True
            response.message = "Action computed successfully"
            
            self.get_logger().debug('Action computed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error computing action: {str(e)}')
            response.success = False
            response.message = str(e)
            response.action = RobotAction()  # Empty action
        
        return response
    
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
            # Convert to numpy arrays with batch dimension
            joint_positions = np.array(ros_obs.joint_positions)[np.newaxis, :]  # (1, n_joints)
            obs_dict["state.joint_positions"] = joint_positions
        
        if len(ros_obs.gripper_positions) > 0:
            gripper_positions = np.array(ros_obs.gripper_positions)[np.newaxis, :]  # (1, n_grippers)
            obs_dict["state.gripper"] = gripper_positions
        
        # Add additional state
        for i, key in enumerate(ros_obs.state_keys):
            if i < len(ros_obs.additional_state):
                obs_dict[f"state.{key}"] = np.array([ros_obs.additional_state[i]])[np.newaxis, :]
        
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
        
        # Set action horizon (assuming first action tensor gives us the horizon)
        if gr00t_action:
            first_action = next(iter(gr00t_action.values()))
            if hasattr(first_action, 'shape') and len(first_action.shape) > 0:
                ros_action.action_horizon = int(first_action.shape[0])
            else:
                ros_action.action_horizon = 1
        
        return ros_action


def main(args=None):
    """Main entry point for the node"""
    rclpy.init(args=args)
    
    try:
        node = GR00TInferenceServerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

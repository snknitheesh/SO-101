#!/usr/bin/env python3
"""
Launch file for GR00T inference server
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'model_path',
            default_value='nvidia/GR00T-N1.5-3B',
            description='Path to the GR00T model'
        ),
        DeclareLaunchArgument(
            'embodiment_tag',
            default_value='gr1',
            description='Embodiment tag for the robot'
        ),
        DeclareLaunchArgument(
            'data_config',
            default_value='fourier_gr1_arms_only',
            description='Data configuration to use'
        ),
        DeclareLaunchArgument(
            'denoising_steps',
            default_value='4',
            description='Number of denoising steps'
        ),
        DeclareLaunchArgument(
            'server_port',
            default_value='5555',
            description='ZMQ server port'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda',
            description='Device to run inference on (cuda/cpu)'
        ),
        DeclareLaunchArgument(
            'inference_mode',
            default_value='pytorch',
            description='Inference mode: pytorch or tensorrt'
        ),
        DeclareLaunchArgument(
            'trt_engine_path',
            default_value='gr00t_engine',
            description='Path to TensorRT engines (for tensorrt mode)'
        ),
        
        # Launch the server node
        Node(
            package='gr00t_inference',
            executable='gr00t_inference_server',
            name='gr00t_inference_server',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'embodiment_tag': LaunchConfiguration('embodiment_tag'),
                'data_config': LaunchConfiguration('data_config'),
                'denoising_steps': LaunchConfiguration('denoising_steps'),
                'server_port': LaunchConfiguration('server_port'),
                'device': LaunchConfiguration('device'),
                'inference_mode': LaunchConfiguration('inference_mode'),
                'trt_engine_path': LaunchConfiguration('trt_engine_path'),
            }],
            respawn=True,
            respawn_delay=5.0,
        ),
    ])

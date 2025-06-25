#!/usr/bin/env python3
"""
Launch file for GR00T inference client
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'server_host',
            default_value='localhost',
            description='Hostname/IP of the GR00T inference server'
        ),
        DeclareLaunchArgument(
            'server_port',
            default_value='5555',
            description='Port of the GR00T inference server'
        ),
        DeclareLaunchArgument(
            'camera_topics',
            default_value='["/camera/image_raw"]',
            description='List of camera topics to subscribe to'
        ),
        DeclareLaunchArgument(
            'camera_names',
            default_value='["ego_view"]',
            description='List of camera names corresponding to topics'
        ),
        DeclareLaunchArgument(
            'joint_state_topic',
            default_value='/joint_states',
            description='Topic for robot joint states'
        ),
        DeclareLaunchArgument(
            'language_instruction_topic',
            default_value='/language_instruction',
            description='Topic for language instructions'
        ),
        DeclareLaunchArgument(
            'action_topic',
            default_value='/robot_action',
            description='Topic to publish robot actions'
        ),
        DeclareLaunchArgument(
            'action_horizon',
            default_value='8',
            description='Number of actions to execute from action chunk'
        ),
        DeclareLaunchArgument(
            'control_frequency',
            default_value='50.0',
            description='Control frequency in Hz'
        ),
        
        # Launch the client node
        Node(
            package='gr00t_inference',
            executable='gr00t_inference_client',
            name='gr00t_inference_client',
            output='screen',
            parameters=[{
                'server_host': LaunchConfiguration('server_host'),
                'server_port': LaunchConfiguration('server_port'),
                'camera_topics': LaunchConfiguration('camera_topics'),
                'camera_names': LaunchConfiguration('camera_names'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'language_instruction_topic': LaunchConfiguration('language_instruction_topic'),
                'action_topic': LaunchConfiguration('action_topic'),
                'action_horizon': LaunchConfiguration('action_horizon'),
                'control_frequency': LaunchConfiguration('control_frequency'),
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])

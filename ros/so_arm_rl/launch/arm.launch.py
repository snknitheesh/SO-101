from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='so_arm_rl',
            executable='so101',
            name='so101_node',
            output='screen'
        )
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("fr3_franka_hand", package_name="franka_moveit")
        .robot_description(file_path="coonfig/fr3_franka_hand.urdf.xacro")
        .robot_description_semantic(file_path="config/fr3_franka_hand.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=[
                "ompl",
                "chomp"
            ]
        )
        .to_moveit_configs()
    )
    
    ros2_controllers_path = os.path.join(
        get_package_share_directory("franka_moveit"),
        "config",
        "ros2_controllers.yaml",
    )
    with open(ros2_controllers_path, "r") as f:
        ros2_controllers_config = yaml.safe_load(f)
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_config["controller_manager"]["ros__parameters"]  # <-- Only pass this section
        ],
        output="screen",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    arm_cotroller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    robot_description = moveit_config.robot_description['robot_description']

    with open("/tmp/expanded_robot.urdf", "w") as f:
        f.write(robot_description)
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict()],
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            "info",
        ],
    )
    
    rviz_config_path = os.path.join(
        get_package_share_directory("franka_moveit"),
        "config",
        "moveit.rviz",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic, moveit_config.planning_pipelines, moveit_config.robot_description_kinematics],  # noqa: E501
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )
    
    return LaunchDescription(
        [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_cotroller_spawner,
            gripper_controller_spawner,
            move_group_node,
            rviz_node,
            robot_state_publisher_node,
        ]
    )
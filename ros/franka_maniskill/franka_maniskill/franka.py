import rclpy
from rclpy.node import Node
import franka_maniskill.franka_agent  # noqa: F401
import gymnasium as gym
import numpy as np
from sensor_msgs.msg import JointState

class frankaSimNode(Node):
    def __init__(self):
        super().__init__("franka_sim")

        self.env = gym.make(
            "Empty-v1",
            obs_mode="none",
            enable_shadow=True,
            control_mode="pd_joint_pos",
            robot_uids="franka",
            render_mode="human",
            sim_config=dict(
                sim_freq=2000,
                control_freq=200,
                scene_config=dict(
                    gravity=np.array([0, 0, -9.81]),
                    cpu_workers=0,
                ),
            ),
            sim_backend="gpu",
        )
        
        self.get_logger().info("Franka simulation environment created successfully.")
        self.env.reset(seed=0)
        self.get_logger().info("Franka simulation environment reset successfully.")
        robot = self.env.unwrapped.agent.robot
        
        for joint in robot.get_active_joints():
            name = joint.name
            if "fr3_joint1" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
            elif "fr3_joint2" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
            elif "fr3_joint3" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
            elif "fr3_joint4" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
            elif "fr3_joint5" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
            elif "fr3_joint6" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
            elif "fr3_joint7" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
            elif "fr3_finger_joint1" in name:
                joint.set_drive_properties(
                    stiffness=1e3,
                    damping=1e2,
                    force_limit=100.0,
                )
        
        self.active_joints = robot.get_active_joints()
        self.active_joint_names = [joint.name for joint in self.active_joints]
        self.get_logger().info(f"Active joints: {self.active_joint_names}")
        
        self.joint_state_pub = self.create_publisher(
            JointState, "franka_arm_joint_states", 10
        )
        
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            "franka_arm_joint_commands",
            self.joint_command_callback,
            10
        )
        
        self.current_action = np.zeros(len(self.active_joint_names))
        self.timer = self.create_timer(0.05, self.step_sim)
        
    def publish_joint_states(self):
        """
        Publish joint states that will be consumed by the topic-based ROS2 control hardware interfaces
        """
        robot = self.env.unwrapped.agent.robot
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        qpos = robot.get_qpos().cpu().numpy().flatten()
        qvel = robot.get_qvel().cpu().numpy().flatten()
        msg.name = self.active_joint_names
        msg.position = [float(x) for x in qpos]
        msg.velocity = [float(x) for x in qvel]
        msg.effort = [0.0] * len(msg.name)
        self.joint_state_pub.publish(msg)
        
    def joint_command_callback(self, msg: JointState):
        """
        Update current_action based on incoming JointState command.
        """
        self.get_logger().info(f"Received JointState command: {msg}")
        name_to_idx = {name: i for i, name in enumerate(self.active_joint_names)}
        for i, joint_name in enumerate(msg.name):
            if joint_name in name_to_idx and i < len(msg.position):
                idx = name_to_idx[joint_name]
                self.current_action[idx] = msg.position[i]
        self.get_logger().info(f"Updated current_action: {self.current_action}")
    
    def step_sim(self):
        if self.env is not None:
            self.obs, _, _, _, _ = self.env.step(self.current_action)
            self.publish_joint_states()
            self.env.render()

def main(args=None):
    rclpy.init(args=args)
    node = frankaSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

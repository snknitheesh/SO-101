import rclpy
from rclpy.node import Node
import so_arm_rl.so101agent  # noqa: F401
# import so_arm_rl.so101env  # noqa: F401
import gymnasium as gym
import numpy as np
from sensor_msgs.msg import JointState

class SO101SimNode(Node):
    def __init__(self):
        super().__init__("so101Sim")

        self.env = gym.make(
            "Empty-v1",
            obs_mode="none",
            enable_shadow=True,
            control_mode="pd_joint_pos",
            robot_uids="so101agent",
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

        self.obs = self.env.reset(seed=0)[0]
        robot = self.env.unwrapped.agent.robot
        
        self.active_joints = robot.get_active_joints()
        self.active_joint_names = [joint.name for joint in self.active_joints]
        
        self.joint_state_pub = self.create_publisher(
            JointState, "arm101_joint_states", 10
        )
        
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            "arm101_joint_command",
            self.joint_command_callback,
            10
        )
        
        self.current_action = np.zeros(len(self.active_joint_names), dtype=np.float32)
        self.step_timer = self.create_timer(0.02, self.step_sim)
    
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
            self.get_logger().info("Stepping simulation")
            self.get_logger().info(f"Current action: {self.current_action}")
            action = self.env.action_space.sample()  
            self.obs, _, _, _, _ = self.env.step(self.current_action)
            self.publish_joint_states()
            self.env.render()

def main(args=None):
    rclpy.init(args=args)
    node = SO101SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
import so_arm_rl.so101agent  # noqa: F401
import so_arm_rl.so101env  # noqa: F401
import gymnasium as gym
import numpy as np
from sensor_msgs.msg import JointState

class SO101SimNode(Node):
    def __init__(self):
        super().__init__("so101Sim")

        self.env = gym.make(
            "so101Env-v0",
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
        self.current_action = np.zeros(len(self.active_joint_names), dtype=np.float32)
        
        self.joint_state_pub = self.create_publisher(
            JointState, "arm101_joint_states", 10
        )
        
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

    
    def step_sim(self):
        if self.env is not None:
            # action = self.env.action_space.sample()  
            # action, _ = model.predict(self.obs, deterministic=True)
            self.obs, _, _, _, _ = self.env.step(action)
            self.publish_joint_states()
            self.env.render()

def main(args=None):
    rclpy.init(args=args)
    node = SO101SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

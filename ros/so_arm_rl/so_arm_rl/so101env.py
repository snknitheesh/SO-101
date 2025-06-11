import sapien
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.utils.registration import register_env
import numpy as np

@register_env("so101Env-v0", max_episode_steps=100)
class RedCubeEnv(BaseEnv):
    SUPPORTED_ROBOTS = ["so101agent"]

    def __init__(self, *args, robot_uids="so101agent", **kwargs):
        super().__init__(*args, robot_uids=robot_uids, **kwargs)
        
    def _setup(self):
        self._load_scene({})
        self._load_agent({})

    def _load_agent(self, options: dict):
        super()._load_agent(options, sapien.Pose(p=[0, 0, 0]))

    def _load_scene(self, options: dict):
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[2, 2, 0.01])
        builder.add_box_visual(
            half_size=[2, 2, 0.01],
            material=sapien.render.RenderMaterial(base_color=[0.7, 0.7, 0.7, 1]),
        )
        builder.initial_pose = sapien.Pose(p=[0, 0, -0.01])
        self.ground = builder.build_static(name="ground")

        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.025, 0.025, 0.025])
        builder.add_box_visual(
            half_size=[0.025, 0.025, 0.025],
            material=sapien.render.RenderMaterial(base_color=[1, 0, 0, 1]),
        )
        builder.initial_pose = sapien.Pose(p=[0.0, -0.3, 0.025])
        self.cube = builder.build(name="red_cube")
        self.goal_pos = np.array([0.3, 0.3, 0.025])
    
    def _reset_scene(self, options: dict = None):
        super()._reset_scene(options)
        self.cube.set_pose(sapien.Pose(p=[0.0, -0.3, 0.025]))

    def _reset_agent(self):
        self.cube.set_pose(sapien.Pose(p=[0.0, -0.3, 0.025]))
        
    def compute_normalized_dense_reward(self, obs, action, info):
        cube_pos = np.array(self.cube.pose.p).squeeze()
        tcp_pos = np.array(self.agent.tcp.pose.p).squeeze()
        dist_to_goal = np.linalg.norm(cube_pos[:2] - self.goal_pos[:2])
        tcp_to_cube = np.linalg.norm(tcp_pos[:2] - cube_pos[:2])
        reward = -dist_to_goal
        if tcp_to_cube < 0.05:
            reward += 1.0
        if dist_to_goal < 0.03:
            reward += 10.0
        return reward / 11.0

    def is_done(self):
        cube_pos = np.array(self.cube.pose.p).squeeze()
        dist_to_goal = np.linalg.norm(cube_pos[:2] - self.goal_pos[:2])
        return dist_to_goal < 0.03

    
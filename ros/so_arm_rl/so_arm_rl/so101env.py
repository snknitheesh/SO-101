import sapien
from mani_skill.utils import sapien_utils, common
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.utils.registration import register_env
import numpy as np
import torch
import so101agent  # noqa: F401

@register_env("so101Env-v0", max_episode_steps=100)
class RedCubeEnv(BaseEnv):
    SUPPORTED_ROBOTS = ["so101agent"]  # Use your custom agent

    def __init__(self, *args, robot_uids="so101agent", **kwargs):
        super().__init__(*args, robot_uids=robot_uids, **kwargs)

    def _load_agent(self, options: dict):
        # Place the robot at the origin
        super()._load_agent(options, sapien.Pose(p=[0, 0, 0]))

    def _load_scene(self, options: dict):
        # Ground plane
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[2, 2, 0.01])
        builder.add_box_visual(
            half_size=[2, 2, 0.01],
            material=sapien.render.RenderMaterial(base_color=[0.7, 0.7, 0.7, 1]),
        )
        builder.initial_pose = sapien.Pose(p=[0, 0, -0.01], q=[1, 0, 0, 0])
        self.ground = builder.build_static(name="ground")

        # Red cube: spawn above ground and away from robot base
        builder = self.scene.create_actor_builder()
        builder.add_box_collision(half_size=[0.025, 0.025, 0.025])
        builder.add_box_visual(
            half_size=[0.025, 0.025, 0.025],
            material=sapien.render.RenderMaterial(base_color=[1, 0, 0, 1]),
        )
        # Place the cube at x=0.3, y=0.0, z=0.025 (on ground, away from base)
        builder.initial_pose = sapien.Pose(p=[0.3, 0.0, 0.025], q=[1, 0, 0, 0])
        self.cube = builder.build(name="red_cube")
        self.goal_pos = np.array([0.3, 0.3, 0.025])
        
    def compute_normalized_dense_reward(self, obs, action, info):
        # Reward: negative distance from cube to goal
        cube_pos = self.cube.pose.p[0].cpu().numpy()
        dist_to_goal = np.linalg.norm(cube_pos[:2] - self.goal_pos[:2])
        return -dist_to_goal
    
    def _get_obs_extra(self, info):
        # Ensure both are torch tensors with shape [1, N]
        cube_pose = self.cube.pose.raw_pose  # shape [1, 7]
        goal_pos = torch.tensor(self.goal_pos, dtype=cube_pose.dtype, device=cube_pose.device).reshape(1, 3)
        obs = {
            "cube_pose": cube_pose,
            "goal_pos": goal_pos,
        }
        return obs
        
import os

from gymnasium.envs.registration import register

__version__ = "0.0.1"

ASSETS_PATH = os.path.join(os.path.dirname(__file__), "assets", "low_cost_robot_6dof")

register(
    id="PickPlaceCube-v0",
    entry_point=".:so101envmujoco",
    max_episode_steps=50,
)
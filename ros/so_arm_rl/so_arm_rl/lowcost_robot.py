import gymnasium as gym
import gym_lowcostrobot 
from sb3_contrib import TQC
import numpy as np

# Create the environment and wrap it
env = gym.make("PickPlaceCube-v0", render_mode="human")
# env = AddCubePosObsWrapper(env, cube_pos_shape=(3,))  # adjust shape if needed

model = TQC.load("logs/tqc/LiftCube-v0_1/best_model.zip")

obs, info = env.reset()
print("Observation keys:", obs.keys())
while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()
    if terminated or truncated:
        obs, info = env.reset()
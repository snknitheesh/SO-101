import gymnasium as gym
from stable_baselines3 import PPO
import so101agent  # noqa: F401
from gymnasium.wrappers import FlattenObservation
import so101env  # noqa: F401

env = gym.make("so101Env-v0", obs_mode="state", render_mode=None)
env = FlattenObservation(env)

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=500_000)
model.save("so101_push_cube_ppo")


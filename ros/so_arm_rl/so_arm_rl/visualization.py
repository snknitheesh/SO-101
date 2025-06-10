import gymnasium as gym
from stable_baselines3 import PPO
import so101agent  # noqa: F401
import so101env    # noqa: F401

# Create the environment with rendering enabled
env = gym.make("so101Env-v0", obs_mode="state", render_mode="human")

# Load the trained model
model = PPO.load("so101_push_cube_ppo")

obs, _ = env.reset()
while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()
    if terminated or truncated:
        obs, _ = env.reset()
import gym
from stable_baselines3 import PPO
from global_env import GlobalEnv
from xplane_client import XPlaneClient

# Create client and environment (same setup as before)
client = XPlaneClient()
env = GlobalEnv(client=client, max_episode_steps=1000)

# Load the previously trained model
model = PPO.load("ppo_xplane_model_2", env=env)

# Optional: Set env again in case loading doesn't automatically bind it
model.set_env(env)

# Continue training
model.learn(
    total_timesteps=500000,  # additional steps
    reset_num_timesteps=False,  # continue timestep count
    progress_bar=True,
    tb_log_name="ppo_xplane_run_continued"
)

# Save the updated model
model.save("ppo_xplane_model_heading")  # Save as a new version

# Cleanup
env._close()


# # train_rl_model.py
# import gym
# from stable_baselines3 import PPO
# from s_env import XplaneEnv
# from xplane_client import XPlaneClient

# # Create client and environment
# client = XPlaneClient()
# env = XplaneEnv(client=client, max_episode_steps=10000)

# # Wrap environment if needed (e.g., for vectorization)
# from stable_baselines3.common.vec_env import DummyVecEnv
# env = DummyVecEnv([lambda: env])

# # Train with PPO
# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_xplane_tensorboard/")
# model.learn(total_timesteps=200000, progress_bar=True, tb_log_name="ppo_xplane_run")

# # Save model
# model.save("ppo_xplane_model_1M")

# # Cleanup
# env._close()
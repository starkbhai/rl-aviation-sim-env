# test_rl_model.py
from stable_baselines3 import PPO
from global_env import GlobalEnv
from xplane_client import XPlaneClient
import time

# Create client and environment
client = XPlaneClient()
env = GlobalEnv(client=client, max_episode_steps=10000)

# Load the trained model
model = PPO.load("ppo_xplane_model_2", env=env)

# Run one or more episodes
obs = env.reset()
done = False
while True:
    action, _ = model.predict(obs, deterministic=True)  # use deterministic=True for consistent output
    obs, reward, done, info = env.step(action)
    print(f"Action: {action}, Reward: {reward}, Done: {done}")
    time.sleep(0.05)  # Slight delay for visualization or smoother control
    if done:
        obs = env.reset()

# Cleanup
env._close()


# import gym
# from stable_baselines3 import PPO
# from stable_baselines3.common.callbacks import BaseCallback

# from xplane_envBase import XplaneEnv
# from xplane_client import XPlaneClient

# # ✅ 1️⃣ Define a simple episode counter callback
# class EpisodeCounterCallback(BaseCallback):
#     """
#     Custom callback to count and print the number of episodes completed.
#     """
#     def __init__(self, verbose=0):
#         super(EpisodeCounterCallback, self).__init__(verbose)
#         self.episode_num = 0

#     def _on_step(self) -> bool:
#         # SB3 handles vector envs → check for done flags in array
#         done_array = self.locals.get("dones")
#         if done_array is not None:
#             for done in done_array:
#                 if done:
#                     self.episode_num += 1
#                     print(f"✅ Episode ended: {self.episode_num}")
#         return True

# # ✅ 2️⃣ Create X-Plane client and environment
# client = XPlaneClient()
# env = XplaneEnv(client=client, max_episode_steps=1000)

# # ✅ 3️⃣ Create PPO model
# model = PPO(
#     "MlpPolicy",
#     env,
#     verbose=1,
#     tensorboard_log="./ppo_xplane_tensorboard/"
# )

# # ✅ 4️⃣ Create the episode counter callback
# episode_counter = EpisodeCounterCallback()

# # ✅ 5️⃣ Train PPO agent with the callback
# model.learn(
#     total_timesteps=10_000_000,
#     callback=episode_counter
# )

# # ✅ 6️⃣ Save trained model
# model.save("ppo_xplane_model")

# # ✅ 7️⃣ Close connection to X-Plane
# env._close()



# # train_rl_model.py
# import gym
# from stable_baselines3 import PPO
# from s_env import XplaneEnv
# from xplane_client import XPlaneClient

# # Create client and environment
# client = XPlaneClient()
# env = XplaneEnv(client=client, max_episode_steps=1000)

# # Wrap environment if needed (e.g., for vectorization)
# from stable_baselines3.common.vec_env import DummyVecEnv
# env = DummyVecEnv([lambda: env])

# # Train with PPO
# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_xplane_tensorboard/")
# model.learn(total_timesteps=1000000, progress_bar=True, tb_log_name="ppo_xplane_run")

# # Save model
# model.save("ppo_xplane_model_1M")

# # Cleanup
# env._close()
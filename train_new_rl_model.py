import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
import torch
import os
from global_env import GlobalEnv
from xplane_client import XPlaneClient
from interface.physics_engine_interface import create_flight_interface
from datetime import datetime
import atexit
import signal
import sys

# Import the flight data logger
try:
    from flight_data_logger import FlightDataLogger
    LOGGING_AVAILABLE = True
    print("📊 Flight data logging system available!")
except ImportError:
    LOGGING_AVAILABLE = False
    print("⚠️ Flight data logging not available - install matplotlib and pandas")

class LoggingWrapper(gym.Wrapper):
    """
    Simple wrapper that adds flight data logging to any environment
    without modifying the original environment
    """
    
    def __init__(self, env, enable_logging=True, log_dir="flight_logs"):
        super().__init__(env)
        
        self.episode_count = 0
        self.step_count = 0
        
        # Initialize logger if available
        if enable_logging and LOGGING_AVAILABLE:
            try:
                self.flight_logger = FlightDataLogger(log_dir=log_dir)
                self.logging_enabled = True
                print("✅ Flight data logging enabled")
            except Exception as e:
                print(f"⚠️ Failed to initialize flight logger: {e}")
                self.flight_logger = None
                self.logging_enabled = False
        else:
            self.flight_logger = None
            self.logging_enabled = False
    
    def step(self, action):
        # Call the original environment step
        obs, reward, terminated, truncated, info = self.env.step(action)
        
        self.step_count += 1
        
        # Log the data if logging is enabled
        if self.logging_enabled and self.flight_logger:
            try:
                # obs format: [altitude, pitch, roll, heading, latitude, longitude, airspeed]
                if len(obs) >= 7:
                    self.flight_logger.log_step(
                        observations=obs,
                        reward=reward,
                        episode=self.episode_count,
                        total_reward=0  # We don't track total reward in wrapper
                    )
            except Exception as e:
                if self.step_count % 100 == 0:  # Print errors occasionally
                    print(f"[WARNING] Flight logging error: {e}")
        
        return obs, reward, terminated, truncated, info
    
    def reset(self, **kwargs):
        # Call the original environment reset
        obs, info = self.env.reset(**kwargs)
        
        self.episode_count += 1
        
        if self.episode_count % 10 == 0:  # Print episode progress
            print(f"📊 Episode {self.episode_count} started (Step {self.step_count})")
        
        return obs, info
    
    def close(self):
        # Generate graphs when environment closes
        if self.logging_enabled and self.flight_logger:
            print("\n📊 Generating flight data analysis...")
            try:
                self.flight_logger.save_and_generate_graphs()
            except Exception as e:
                print(f"⚠️ Error generating flight analysis: {e}")
        
        # Close the original environment
        self.env.close()

# Global variables for cleanup
env = None

def cleanup_and_generate_graphs():
    """Cleanup function that runs when program exits"""
    global env
    
    print("\n🔄 Program cleanup started...")
    
    try:
        if env:
            env.close()
    except Exception as e:
        print(f"⚠️ Error during cleanup: {e}")
    
    print("✅ Cleanup completed!")

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print(f"\n⚠️ Received signal {signum}. Starting graceful shutdown...")
    cleanup_and_generate_graphs()
    sys.exit(0)

# Register cleanup functions
atexit.register(cleanup_and_generate_graphs)
signal.signal(signal.SIGINT, signal_handler)

print("🚀 Starting Fresh PPO Training for JSBSim with Flight Data Analysis")
print("=" * 60)

# Create directories
os.makedirs("./models", exist_ok=True)
os.makedirs("./logs", exist_ok=True)
if LOGGING_AVAILABLE:
    os.makedirs("./flight_logs", exist_ok=True)

# Create client and environment (EXACTLY as your original script)
print("[INFO] Creating JSBSim environment...")
client = create_flight_interface("xplane")
env = GlobalEnv(client=client, max_episode_steps=1000)

print(f"[INFO] Observation space: {env.observation_space}")
print(f"[INFO] Action space: {env.action_space}")

# Add logging wrapper (this is the ONLY change)
if LOGGING_AVAILABLE:
    env = LoggingWrapper(env, enable_logging=True, log_dir="./flight_logs")
    print("📊 Flight data logging: ✅ ENABLED")
else:
    print("📊 Flight data logging: ❌ NOT AVAILABLE")

# Test environment (same as your original)
print("[INFO] Testing environment...")
try:
    obs, info = env.reset()
    print(f"[INFO] ✅ Reset successful. Observation shape: {obs.shape}")
    print(f"[INFO] Sample observation: {obs}")
except Exception as e:
    print(f"[ERROR] ❌ Environment test failed: {e}")
    cleanup_and_generate_graphs()
    exit(1)

# Wrap environment (same as your original)
print("[INFO] Wrapping environment...")
env = Monitor(env, "./logs/")
env = DummyVecEnv([lambda: env])

# Create new PPO model from scratch (same as your original)
print("[INFO] Creating new PPO model...")
model = PPO(
    policy="MlpPolicy",
    env=env,
    learning_rate=3e-4,
    n_steps=2048,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    clip_range=0.2,
    ent_coef=0.01,
    verbose=1,
    tensorboard_log="./logs/",
    device="auto"
)

print(f"[INFO] Model created successfully!")
print(f"[INFO] Using device: {model.device}")

# Training configuration (same as your original)
total_timesteps = 100000  # Start with smaller number for testing
print(f"[INFO] Training for {total_timesteps} timesteps...")

if LOGGING_AVAILABLE:
    print(f"📊 Flight data will be collected automatically")
    print(f"📈 Graphs will be generated when training completes")

# Start training (EXACTLY same as your original)
training_successful = False
try:
    print("\n🎯 Starting training...")
    model.learn(
        total_timesteps=total_timesteps,
        progress_bar=True,
        tb_log_name="ppo_jsbsim_fresh"
    )
    print("✅ Training completed successfully!")
    training_successful = True
    
except KeyboardInterrupt:
    print("⚠️ Training interrupted by user")
    
except Exception as e:
    print(f"❌ Training failed: {e}")
    import traceback
    traceback.print_exc()

# Save the model (same as your original)
print("\n[INFO] Saving model...")
model_name = "ppo_xplane_fresh_model"
model.save(f"./models/{model_name}")
print(f"✅ Model saved as: ./models/{model_name}")

# Save with timestamp (same as your original)
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
timestamped_name = f"ppo_xplane_{timestamp}"
model.save(f"./models/{timestamped_name}")
print(f"✅ Timestamped model saved as: ./models/{timestamped_name}")

# Quick evaluation (same as your original)
if training_successful:
    print("\n[INFO] Running quick evaluation...")
    try:
        obs = env.reset()
        total_reward = 0
        steps = 0
        for _ in range(50):  # Short evaluation
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward[0] if isinstance(reward, (list, tuple)) else reward
            steps += 1
            if done:
                break
        print(f"✅ Evaluation: {steps} steps, Total reward: {total_reward:.2f}")
    except Exception as e:
        print(f"⚠️ Evaluation failed: {e}")

# Cleanup (this will generate graphs)
print("\n📊 Finalizing and generating flight data analysis...")
cleanup_and_generate_graphs()

print("\n" + "=" * 50)
print("🎉 TRAINING COMPLETED!")
print("=" * 50)
print(f"📊 Total timesteps: {total_timesteps}")
print(f"🧠 Model saved: ./models/{model_name}")
print(f"📝 Logs saved: ./logs/")
print(f"🔍 View training: tensorboard --logdir=./logs/")

if LOGGING_AVAILABLE:
    print(f"📊 Flight data: ./flight_logs/")
    print(f"📈 Flight graphs: ./flight_logs/flight_graphs_*.png")

print("=" * 50)
print("\n✨ Script finished successfully! ✨")
print("🎯 Check ./flight_logs/ for your flight data analysis!")
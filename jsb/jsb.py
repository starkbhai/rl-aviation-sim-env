#!/usr/bin/env python3

import jsbsim
import numpy as np
import time
import math
import socket
import json
import threading

class JSBSImClient:
    """
    JSBSim Flight Environment for Reinforcement Learning
    Provides clean interface for RL training with aircraft simulation
    """
    
    def __init__(self, aircraft_model='c172p', dt=0.1 , obs_port=5555):
        """
        Initialize JSBSim Flight Environment
        
        Args:
            aircraft_model (str): Aircraft model to load
            dt (float): Simulation timestep in seconds
        """
        self.aircraft_model = aircraft_model
        self.dt = dt
        self.sim = None
        self.initialized = False
        self.episode_step = 0
        self.max_episode_steps = 1000

        # TODO- Port communication settings
        self.obs_port = obs_port
        self.obs_socket = None
        self.obs_publishing = False
        self.obs_thread = None
        
        # Initial conditions (will be used for reset)
        self.initial_conditions = {
            'lat': 37.62131,      # San Francisco Bay Area
            'lon': -122.37896,
            'altitude_ft': 5000.0,
            'speed_kts': 100.0,
            'heading_deg': 90.0,   # East
            'pitch_deg': 0.0,      # Level
            'roll_deg': 0.0        # Wings level
        }
        
        # Action and observation space definitions
        self.action_size = 4  # [elevator, aileron, rudder, throttle]
        self.observation_size = 12  # Key flight parameters
        
        print("🛩️  JSBSim RL Environment Created")
    
    # Todo : - Add Port Functions

    def start_obs_publishing(self):
        """Start publishing observations to port"""
        try:
            # Create UDP socket for sending observations
            self.obs_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.obs_publishing = True
            
            # Start background thread for publishing
            self.obs_thread = threading.Thread(target=self._obs_publishing_loop, daemon=True)
            self.obs_thread.start()
            
            print(f"✅ Started observation publishing on port {self.obs_port}")
            
        except Exception as e:
            print(f"❌ Failed to start observation publishing: {e}")
            self.obs_publishing = False
    
    def stop_obs_publishing(self):
        """Stop publishing observations"""
        self.obs_publishing = False
        if self.obs_socket:
            self.obs_socket.close()
            self.obs_socket = None
        print("🛑 Stopped observation publishing")
    
    def _obs_publishing_loop(self):
        """Background loop to continuously publish observations"""
        while self.obs_publishing and self.initialized:
            try:
                # Get current observations
                obs = self.getObs()
                
                if obs is not None and len(obs) >= 12:
                    # Create observation data packet
                    obs_data = {
                        'timestamp': time.time(),
                        'latitude': float(obs[0]),
                        'longitude': float(obs[1]),
                        'altitude_ft': float(obs[2]),
                        'roll': float(obs[3]),
                        'pitch': float(obs[4]),
                        'heading': float(obs[5]),
                        'airspeed_kts': float(obs[6]),
                        'vertical_speed_fps': float(obs[7]),
                        'throttle_pos': float(obs[8]),
                        'elevator_pos': float(obs[9]),
                        'aileron_pos': float(obs[10]),
                        'rudder_pos': float(obs[11]),
                        'episode_step': self.episode_step
                    }
                    
                    # Convert to JSON and send via UDP
                    json_data = json.dumps(obs_data).encode('utf-8')
                    self.obs_socket.sendto(json_data, ('localhost', self.obs_port))
                
                # Publish at 20Hz
                time.sleep(0.05)
                
            except Exception as e:
                if self.obs_publishing:  # Only print if we're supposed to be publishing
                    print(f"[WARNING] Observation publishing error: {e}")
                time.sleep(0.1)
    
    def initializeJsb(self):
        """
        Initialize JSBSim simulation
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            print("🚀 Initializing JSBSim...")
            
            # Create JSBSim instance
            self.sim = jsbsim.FGFDMExec(None)
            
            # Try to load aircraft model
            if not self.sim.load_model(self.aircraft_model):
                print(f"❌ Failed to load aircraft model: {self.aircraft_model}")
                return False
            
            print(f"✅ Successfully loaded aircraft: {self.aircraft_model}")
            
            # Set simulation timestep
            self.sim.set_dt(self.dt)
            
            # Set initial conditions
            self._set_initial_conditions()
            
            # Initialize simulation
            if not self.sim.run_ic():
                print("❌ Failed to initialize JSBSim initial conditions")
                return False
            
            # Start engine
            self._start_engine()
            
            # Run a few steps to stabilize
            for _ in range(10):
                self.sim.run()
            
            self.initialized = True
            self.episode_step = 0
            
            # Start publishing observations to port
            self.start_obs_publishing()


            print("✅ JSBSim initialized successfully!")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize JSBSim: {e}")
            self.initialized = False
            return False
    
    def _set_initial_conditions(self):
        """Set initial flight conditions"""
        ic = self.initial_conditions
        
        # Position
        self.sim.set_property_value('ic/lat-gc-deg', ic['lat'])
        self.sim.set_property_value('ic/long-gc-deg', ic['lon'])
        self.sim.set_property_value('ic/h-sl-ft', ic['altitude_ft'])
        
        # Velocity
        self.sim.set_property_value('ic/vc-kts', ic['speed_kts'])
        
        # Attitude
        self.sim.set_property_value('ic/psi-true-deg', ic['heading_deg'])
        self.sim.set_property_value('ic/theta-deg', ic['pitch_deg'])
        self.sim.set_property_value('ic/phi-deg', ic['roll_deg'])
        
        # Aircraft configuration
        self.sim.set_property_value('gear/gear-cmd-norm', 0.0)  # Gear up
        self.sim.set_property_value('fcs/flap-cmd-norm', 0.0)   # Flaps up
    
    def _start_engine(self):
        """Start aircraft engine"""
        try:
            # Start engine
            self.sim.set_property_value('propulsion/engine/set-running', 1)
            self.sim.set_property_value('propulsion/starter_cmd', 1)
            self.sim.set_property_value('fcs/mixture-cmd-norm', 1.0)
            self.sim.set_property_value('propulsion/magneto_cmd', 3)
            self.sim.set_property_value('fcs/throttle-cmd-norm', 0.7)
            
            print("🔧 Engine started")
            
        except Exception as e:
            print(f"⚠️  Engine start warning: {e}")
    
    def resetJsb(self):
        """
        Reset JSBSim to initial conditions
        
        Returns:
            np.ndarray: Initial observation
        """
        if not self.initialized:
            print("❌ JSBSim not initialized. Call initializeJsb() first.")
            return None
        
        try:
            print("🔄 Resetting JSBSim to initial position...")
            
            # Reset to initial conditions
            self._set_initial_conditions()
            
            # Re-initialize
            if not self.sim.run_ic():
                print("❌ Failed to reset initial conditions")
                return None
            
            # Restart engine
            self._start_engine()
            
            # Run a few steps to stabilize
            for _ in range(5):
                self.sim.run()
            
            # Reset episode counter
            self.episode_step = 0
            
            print("✅ JSBSim reset successfully!")
            
            # Return initial observation
            return self.getObs()
            
        except Exception as e:
            print(f"❌ Failed to reset JSBSim: {e}")
            return None
    
    def sendAction(self, action):
        """
        Send control actions to aircraft
        
        Args:
            action (list/np.ndarray): [elevator, aileron, rudder, throttle]
                - elevator: -1.0 to 1.0 (pitch control)
                - aileron:  -1.0 to 1.0 (roll control)
                - rudder:   -1.0 to 1.0 (yaw control)  
                - throttle:  0.0 to 1.0 (engine power)
        
        Returns:
            bool: True if action applied successfully
        """
        if not self.initialized:
            print("❌ JSBSim not initialized")
            return False
        
        try:
            # Ensure action is numpy array
            action = np.array(action)
            
            # Print detailed action values before clamping
            print("\n[DEBUG] Raw Action Input - Recived:")
            print(f"  Elevator (action[0]): {action[0]}")
            print(f"  Aileron  (action[1]): {action[1]}")
            print(f"  Rudder   (action[2]): {action[2]}")
            print(f"  Throttle (action[3]): {action[3]}")

            # Clamp actions to valid ranges
            elevator = np.clip(action[0], -1.0, 1.0)
            aileron = np.clip(action[1], -1.0, 1.0)
            rudder = np.clip(action[2], -1.0, 1.0)
            throttle = np.clip(action[3], 0.0, 1.0)
            
            # Apply control actions
            self.sim.set_property_value('fcs/elevator-cmd-norm', float(elevator))
            self.sim.set_property_value('fcs/aileron-cmd-norm', float(aileron))
            self.sim.set_property_value('fcs/rudder-cmd-norm', float(rudder))
            self.sim.set_property_value('fcs/throttle-cmd-norm', float(throttle))
            
            return True
            
        except Exception as e:
            print(f"❌ Failed to send action: {e}")
            return False
    
    def step(self):
        """
        Step the simulation forward by one timestep
        
        Returns:
            bool: True if step successful
        """
        if not self.initialized:
            return False
        
        try:
            # Step simulation
            success = self.sim.run()
            self.episode_step += 1
            return success
            
        except Exception as e:
            print(f"❌ Simulation step failed: {e}")
            return False
    
    def getObs(self):
        """
        Get current aircraft observations
        
        Returns:
            np.ndarray: Observation vector with shape (observation_size,)
                [lat, lon, alt_ft, roll, pitch, heading, 
                 airspeed, vertical_speed, throttle_pos, 
                 elevator_pos, aileron_pos, rudder_pos]
        """
        if not self.initialized:
            print("❌ JSBSim not initialized")
            return np.zeros(self.observation_size)
        
        try:
            # Get basic flight data
            obs = np.array([
                # Position (3)
                self.sim.get_property_value("position/lat-gc-deg"),
                self.sim.get_property_value("position/long-gc-deg"),
                self.sim.get_property_value("position/h-sl-ft"),
                
                # Attitude (3)
                self.sim.get_property_value("attitude/phi-deg"),      # roll
                self.sim.get_property_value("attitude/theta-deg"),    # pitch
                self.sim.get_property_value("attitude/psi-deg"),      # heading
                
                # Motion (2)
                self.sim.get_property_value("velocities/vc-kts"),     # airspeed
                self.sim.get_property_value("velocities/h-dot-fps"),  # vertical speed
                
                # Control positions (4)
                self.sim.get_property_value("fcs/throttle-pos-norm"),
                self.sim.get_property_value("fcs/elevator-pos-norm"),
                self.sim.get_property_value("fcs/aileron-pos-norm"),
                self.sim.get_property_value("fcs/rudder-pos-norm")
            ])
            
            # Handle any NaN values
            obs = np.nan_to_num(obs, nan=0.0)
            
            return obs
            
        except Exception as e:
            print(f"❌ Failed to get observations: {e}")
            return np.zeros(self.observation_size)
    
    def getDetailedObs(self):
        """
        Get comprehensive aircraft observations for analysis
        
        Returns:
            dict: Detailed flight data
        """
        if not self.initialized:
            return {}
        
        try:
            return {
                # Position & Navigation
                'position': {
                    'latitude': self.sim.get_property_value("position/lat-gc-deg"),
                    'longitude': self.sim.get_property_value("position/long-gc-deg"),
                    'altitude_ft': self.sim.get_property_value("position/h-sl-ft"),
                    'altitude_agl_ft': self.sim.get_property_value("position/h-agl-ft")
                },
                
                # Attitude
                'attitude': {
                    'roll_deg': self.sim.get_property_value("attitude/phi-deg"),
                    'pitch_deg': self.sim.get_property_value("attitude/theta-deg"),
                    'heading_deg': self.sim.get_property_value("attitude/psi-deg")
                },
                
                # Velocities
                'velocities': {
                    'airspeed_kts': self.sim.get_property_value("velocities/vc-kts"),
                    'groundspeed_kts': self.sim.get_property_value("velocities/vg-kts"),
                    'vertical_speed_fpm': self.sim.get_property_value("velocities/h-dot-fps") * 60
                },
                
                # Engine
                'engine': {
                    'throttle_pos': self.sim.get_property_value("fcs/throttle-pos-norm"),
                    'rpm': self.sim.get_property_value("propulsion/engine/engine-rpm"),
                    'thrust_lbs': self.sim.get_property_value("propulsion/engine/thrust-lbs")
                },
                
                # Control Surfaces
                'controls': {
                    'elevator_cmd': self.sim.get_property_value("fcs/elevator-cmd-norm"),
                    'elevator_pos': self.sim.get_property_value("fcs/elevator-pos-norm"),
                    'aileron_cmd': self.sim.get_property_value("fcs/aileron-cmd-norm"),
                    'aileron_pos': self.sim.get_property_value("fcs/aileron-pos-norm"),
                    'rudder_cmd': self.sim.get_property_value("fcs/rudder-cmd-norm"),
                    'rudder_pos': self.sim.get_property_value("fcs/rudder-pos-norm")
                },
                
                # Episode info
                'episode': {
                    'step': self.episode_step,
                    'sim_time': self.sim.get_property_value("simulation/sim-time-sec")
                }
            }
            
        except Exception as e:
            print(f"❌ Failed to get detailed observations: {e}")
            return {}
    
    def isDone(self):
        """
        Check if episode should terminate
        
        Returns:
            bool: True if episode is done
        """
        if not self.initialized:
            return True
        
        try:
            # Get current state
            altitude = self.sim.get_property_value("position/h-sl-ft")
            roll = abs(self.sim.get_property_value("attitude/phi-deg"))
            pitch = abs(self.sim.get_property_value("attitude/theta-deg"))
            airspeed = self.sim.get_property_value("velocities/vc-kts")
            
            # Termination conditions
            conditions = [
                self.episode_step >= self.max_episode_steps,  # Max steps
                altitude < 100,     # Too low
                altitude > 15000,   # Too high
                roll > 90,          # Inverted/extreme roll
                pitch > 45,         # Extreme pitch
                airspeed < 40,      # Stall speed
                airspeed > 250      # Too fast
            ]
            
            return any(conditions)
            
        except Exception as e:
            print(f"❌ Error checking done condition: {e}")
            return True
    
    def getReward(self, target_altitude=5000, target_speed=100, target_heading=90):
        """
        Calculate reward for current state (for RL training)
        
        Args:
            target_altitude (float): Target altitude in feet
            target_speed (float): Target airspeed in knots
            target_heading (float): Target heading in degrees
            
        Returns:
            float: Reward value
        """
        if not self.initialized:
            return -100.0
        
        try:
            # Get current state
            altitude = self.sim.get_property_value("position/h-sl-ft")
            airspeed = self.sim.get_property_value("velocities/vc-kts")
            heading = self.sim.get_property_value("attitude/psi-deg")
            roll = abs(self.sim.get_property_value("attitude/phi-deg"))
            pitch = abs(self.sim.get_property_value("attitude/theta-deg"))
            
            # Calculate individual rewards
            alt_error = abs(altitude - target_altitude)
            speed_error = abs(airspeed - target_speed)
            heading_error = min(abs(heading - target_heading), 360 - abs(heading - target_heading))
            
            # Reward components
            altitude_reward = max(0, 100 - alt_error / 10)  # Penalty for altitude deviation
            speed_reward = max(0, 50 - speed_error)         # Penalty for speed deviation
            heading_reward = max(0, 50 - heading_error)     # Penalty for heading deviation
            stability_reward = max(0, 50 - roll - pitch)   # Reward for stable flight
            
            # Penalty for extreme attitudes
            crash_penalty = 0
            if altitude < 100 or roll > 90 or pitch > 45 or airspeed < 40:
                crash_penalty = -500
            
            total_reward = altitude_reward + speed_reward + heading_reward + stability_reward + crash_penalty
            
            return total_reward
            
        except Exception as e:
            print(f"❌ Error calculating reward: {e}")
            return -100.0
    
    def close(self):
        """Clean up JSBSim simulation"""
        # Stop observation publishing
        self.stop_obs_publishing()

        if self.sim:
            self.sim = None
        self.initialized = False
        print("🛑 JSBSim environment closed")


# Example usage and testing functions
def test_environment():
    """Test the JSBSim RL environment"""
    print("🧪 Testing JSBSim RL Environment...")
    
    # Create environment
    env = JSBSImClient()
    
    # Initialize
    if not env.initializeJsb():
        print("❌ Failed to initialize environment")
        return
    
    print("✅ Environment initialized successfully")
    
    # Test reset
    initial_obs = env.resetJsb()
    print(f"📊 Initial observation shape: {initial_obs.shape}")
    print(f"📊 Initial observation: {initial_obs}")
    
    # Test actions and observations
    print("\n🎮 Testing actions and observations...")
    
    for step in range(100):
        # Random action
        action = np.random.uniform([-0.1, -0.1, -0.1, 0.6], [0.1, 0.1, 0.1, 0.8])
        
        # Send action
        env.sendAction(action)
        
        # Step simulation
        env.step()
        
        # Get observation
        obs = env.getObs()
        reward = env.getReward()
        done = env.isDone()
        
        if step % 20 == 0:
            print(f"Step {step}: Alt={obs[2]:.0f}ft, Speed={obs[6]:.1f}kts, Reward={reward:.1f}")
        
        if done:
            print(f"🏁 Episode finished at step {step}")
            break
    
    # Test detailed observations
    detailed = env.getDetailedObs()
    print(f"\n📊 Detailed observation keys: {list(detailed.keys())}")
    
    # Clean up
    env.close()
    print("✅ Test completed successfully!")


if __name__ == "__main__":
    test_environment()
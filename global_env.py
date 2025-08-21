import gymnasium as gym
import parameters
import space_definition as envSpaces
import numpy as np
import time
from gazebo.gz_transport_collision_detector_v2 import AircraftCollisionDetector
from gazebo.camera_sub_module import AircraftCameraSubscriber
import cv2
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose


class GlobalEnv(gym.Env):
    def __init__(self, client, max_episode_steps):
        super().__init__()  # Add super().__init__() call
        
        self.client = client
        envSpace = envSpaces.xplane_space()
        self.ControlParameters = parameters.getParameters()
        self.action_space = envSpace._action_space()
        self.observation_space = envSpace._observation_space()
        self.episode_steps = self.ControlParameters.episodeStep
        self.max_episode_steps = max_episode_steps
        self.parking_brake_released = False
        self.total_delays = 0.0
        self.delay_count = 0
        self.max_delay = 0.0
        self.packet_drops = 0
        self.timeout_incidents = 0
        self.episodes_completed = 0
        self.backprop_times = []

        # Instantiate the collision detector here
        self.collisionDetector = AircraftCollisionDetector()
        self.collisionDetector.set_ignore_ground(True)
        #is_ignoring_ground()

        # Instantiate the Camera here
        self.camera_sub = AircraftCameraSubscriber()


        # !Initialize gz-transport node for publishing
        self.gz_node = Node()
        self.publish_topic = "/gazebo/input_pose"
        
        # Create publisher using correct API
        self.publisher = self.gz_node.advertise(self.publish_topic, Pose)
        
    
    def publish_pose_data(self, data):
        """Publish X-Plane data to gz-transport topic"""
        
        try:
            # Create Pose message with X-Plane data
            pose_msg = Pose()
            
            # Store X-Plane data in the pose message
            # Using position fields for GPS coordinates
            pose_msg.position.x = data[0]     # Store latitude in x
            pose_msg.position.y = data[1]    # Store longitude in y  
            pose_msg.position.z = data[2]  # Store altitude in z
            
            # Store attitude in orientation (as Euler angles temporarily)
            pose_msg.orientation.x = data[3]      # Roll
            pose_msg.orientation.y = data[4]     # Pitch
            pose_msg.orientation.z = data[5]   # Heading/Yaw
            pose_msg.orientation.w = 1.0               # Marker to indicate this is Euler data
            
            # Skip timestamp for now - may not be required
            
            # Publish using correct API (returns boolean)
            success = self.publisher.publish(pose_msg)
            return success
            
        except Exception as e:
            print(f"[ERROR] Publishing failed: {e}")
            return False
    

    def close(self):  # Change from _close() to close()
        try:
            # ✅ PRINT COMPREHENSIVE STATISTICS BEFORE CLOSING
            if self.delay_count > 0:
                avg_delay = self.total_delays / self.delay_count
                avg_backprop = sum(self.backprop_times) / len(self.backprop_times) if self.backprop_times else 0
                packet_drop_rate = (self.packet_drops / self.delay_count) * 100 if self.delay_count > 0 else 0
                timeout_per_100_episodes = (self.timeout_incidents / max(1, self.episodes_completed)) * 100
                
                print(f"[FINAL STATS] Send-receive delay: {avg_delay*1000:.2f}ms avg, {self.max_delay*1000:.2f}ms max spike")
                print(f"[FINAL STATS] Packet drops: {self.packet_drops} ({packet_drop_rate:.2f}% drop rate)")
                print(f"[FINAL STATS] Timeout incidents: {self.timeout_incidents} ({timeout_per_100_episodes:.1f} per 100 episodes)")
                print(f"[FINAL STATS] Backprop time: {avg_backprop*1000:.2f}ms avg over {len(self.backprop_times)} measurements")
            self.client.close()
        except Exception as e:
            print(f"[ERROR] Failed to close client: {e}")

    def step(self, actions):
        start_step_time = time.time()
        self.ControlParameters.flag = False
        self.ControlParameters.episodeReward = 0
        self.episode_steps += 1

        max_retries = 3
        backprop_start_time = time.time()
        
        # Configuration flag - set this based on your preference
        drefArr = True  # True for DREF data, False for POSI data
       
        for attempt in range(max_retries):
            try:
                # Map 4 actions to X-Plane controls
                act = [
                    float(actions[0]),    # elevator (pitch)
                    float(actions[1]),    # aileron (roll) 
                    float(actions[2]),    # rudder (yaw)
                    float(actions[3]),    # throttle
                    0.0,                  # flaps (fixed at 0 - no flaps)
                    0.0,                  # wing sweep (fixed at 0 - no variable sweep)
                    0.0                   # landing gear (fixed at 0 - gear up)
                ]

                start_feedforward = time.time()
                send_start_time = time.time()
                self.client.sendCTRL(act)
                feedforward_time = time.time() - start_feedforward
                
                print(f"[INFO] Feedforward time: {feedforward_time:.4f} seconds")

                # Get unified observation directly
                finalObservationArr = self.client.getObs(use_dref=drefArr)

                #pub_success = self.publish_pose_data(finalObservationArr)
                #if pub_success:
                        #print("Successfully Publish data to GZ")

                # Print observation with descriptive names
                obs_names = [
                    "Altitude (m)",
                    "Pitch (deg)", 
                    "Roll (deg)",
                    "Heading (deg)",
                    "Latitude",
                    "Longitude", 
                    "Airspeed (kts)"
                ]

                print("=" * 60)
                print("📊 FLIGHT OBSERVATION DATA:")
                print("=" * 60)
                for i, (name, value) in enumerate(zip(obs_names, finalObservationArr)):
                    if i < 4:  # Flight attitude data
                        print(f"  {name:15}: {value:8.2f}")
                    elif i < 6:  # Position data
                        print(f"  {name:15}: {value:12.6f}")
                    else:  # Airspeed
                        print(f"  {name:15}: {value:8.2f}")
                print("=" * 60)

                # Alternative compact format:
                #print(f"🛩️  ALT:{finalObservationArr[0]:6.1f}m | PITCH:{finalObservationArr[1]:5.1f}° | ROLL:{finalObservationArr[2]:5.1f}° | HDG:{finalObservationArr[3]:5.1f}° | SPD:{finalObservationArr[6]:5.1f}kts")

                # Alternative single line with emojis:
                #print(f"✈️  📏{finalObservationArr[0]:.1f}m 📐{finalObservationArr[1]:.1f}° 🎯{finalObservationArr[2]:.1f}° 🧭{finalObservationArr[3]:.1f}° 🚀{finalObservationArr[6]:.1f}kts 📍({finalObservationArr[4]:.4f},{finalObservationArr[5]:.4f})")

                # Validate final observation array length
                expected_length = 7  # 4 common + 2 (lat,long) + 1 true_speed
                if len(finalObservationArr) == expected_length:
                    self.ControlParameters.state7 = finalObservationArr
                else:
                    print(f"[WARNING] Final observation length mismatch: expected {expected_length}, got {len(finalObservationArr)}")
                    if hasattr(self.ControlParameters, 'state7') and self.ControlParameters.state7:
                        finalObservationArr = self.ControlParameters.state7
                    else:
                        finalObservationArr = [0.0] * expected_length
                        self.ControlParameters.state7 = finalObservationArr

                # Print observation info
                print(f"[INFO] Alt={finalObservationArr[0]:.1f}, Pitch={finalObservationArr[1]:.1f}, "
                    f"Roll={finalObservationArr[2]:.1f}, Airspeed={finalObservationArr[6]:.1f}")

                # End delay measurement
                receive_end_time = time.time()
                send_receive_delay = receive_end_time - send_start_time
                self.total_delays += send_receive_delay
                self.delay_count += 1
                
                if send_receive_delay > self.max_delay:
                    self.max_delay = send_receive_delay
                avg_delay = self.total_delays / self.delay_count

                # Use the selected common values for reward calculation
                altitude = finalObservationArr[0] 
                pitch = finalObservationArr[1] 
                roll = finalObservationArr[2]
                true_speed = finalObservationArr[6]
                # Smart reward system for ground vs airborne
                reward = 0.0
                is_airborne = altitude > 200 and true_speed > 30
                
                if not is_airborne:
                    # Ground/takeoff rewards
                    reward += 10.0 * act[3]  # Reward throttle for takeoff
                    if true_speed > 0:
                        reward += min(true_speed / 10.0, 3.0)  # Reward speed buildup
                    reward -= abs(roll) * 0.2  # Keep wings level
                else:
                    # Airborne rewards
                    if altitude < 700:
                        reward += 5.0 * act[3]
                        reward += 1.0 if 5 <= pitch <= 20 else -1.0
                        reward += 1.0 if abs(roll) < 15 else -1.0
                    else:
                        reward += 2.0 if 8 <= pitch <= 15 else -1.0
                        reward += 2.0 if abs(roll) < 10 else -1.0
                        reward += 2.0 if 80 <= true_speed <= 300 else -1.0
                
                #Todo Get Camera 
                frame = self.camera_sub.get_latest_image()
                # if frame is not None:
                #      cv2.imshow("Gloabl ENV. Camera Feed", frame)

                if frame is not None:
                    try:
                        # # Create window if it doesn't exist
                        # cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)
                        # cv2.resizeWindow(self.camera_window_name, 640, 480)
                        
                        # Display the frame
                        cv2.imshow("Gloabl ENV. Camera Feed", frame)
                        
                        # CRITICAL: Add waitKey to allow OpenCV to process events
                        # Use a very small delay (1ms) to not block training
                        key = cv2.waitKey(1) & 0xFF
                        
                        # Uncomment this line if you want to see frame details
                        # print(f"📸 Camera Frame (shape: {frame.shape}, dtype: {frame.dtype})")
                        
                    except Exception as cam_error:
                        print(f"[WARNING] Camera display error: {cam_error}")
                     
                     #print(f"🧾 Frame Data (shape: {frame.shape}, dtype: {frame.dtype}):")
                     #print(frame if frame.size < 500 else frame[:5, :5])  # Truncate if too large

                # TODO Check Collide Status : --
                collision_status = self.collisionDetector.is_colliding()
                if collision_status:
                     self.ControlParameters.isCollision = True
                
                print("Collision : 💥 : ----- > ",self.collisionDetector.is_colliding()) 
               
                
                try:
                    # crash_status = self.client.getDREFs(self.ControlParameters.crash)[0][0]
                    crash_status = self.client.isCrash()
                    if crash_status == 1:
                        self.ControlParameters.flag = True
                        print("[INFO] Crash detected 💥")
                        self.episodes_completed += 1
                        self.ControlParameters.totalReward -= 50
                        reset_obs, reset_info = self.reset()
                        return reset_obs, 0.0, True, False, reset_info  # ✅ 5 VALUES
                except:
                    pass  # Ignore crash check errors

                self.ControlParameters.episodeReward = reward
                self.ControlParameters.totalReward += reward

                print(f"[INFO] Step {self.episode_steps}, Reward: {reward:.2f}, Total: {self.ControlParameters.totalReward:.1f}")

                if self.episode_steps >= self.max_episode_steps:
                    self.ControlParameters.flag = True
                    print(f"[INFO] Max steps reached ({self.episode_steps})")

                if self.ControlParameters.flag:
                    self.episodes_completed += 1
                    reward = self.ControlParameters.totalReward
                    self.ControlParameters.totalReward = 0.0
                else:
                    reward = self.ControlParameters.episodeReward

                # Record backprop time
                backprop_end_time = time.time()
                backprop_time = backprop_end_time - backprop_start_time
                self.backprop_times.append(backprop_time)

                # ✅ RETURN 5 VALUES: observation, reward, terminated, truncated, info
                observation = np.array(self.ControlParameters.state7, dtype=np.float32)
                terminated = bool(self.ControlParameters.flag or self.ControlParameters.isCollision)
                truncated = False  # We don't use truncation
                info = {}
                
                return observation, float(reward), terminated, truncated, info

            except Exception as e:
                print(f"[EXCEPTION] Step failed (attempt {attempt + 1}/{max_retries}): {e}")
                if attempt == max_retries - 1:
                    print("[ERROR] Max retries reached, returning default state")
                    observation = np.array(self.ControlParameters.state7, dtype=np.float32)
                    terminated = bool(self.ControlParameters.flag)
                    truncated = False
                    info = {}
                    # ✅ RETURN 5 VALUES
                    return observation, float(self.ControlParameters.episodeReward), terminated, truncated, info
                time.sleep(1.0)

    def _get_info(self):
        return {'control Parameters': self.ControlParameters, 'actions': self.action_space}

    def render(self, mode='human', close=False):
        pass

    def reset(self, seed=None, options=None):  # Fixed signature and return format
        # Handle seed parameter
        if seed is not None:
            np.random.seed(seed)
        
        self.ControlParameters.stateAircraftPosition = []
        self.ControlParameters.stateVariableValue = []
        self.ControlParameters.episodeReward = 0.
        self.ControlParameters.totalReward = 0.
        print(f"[RESET] Entered reset() --- > flag is {self.ControlParameters.flag} 🔴")
        self.ControlParameters.flag = False
        self.episode_steps = 0
        self.ControlParameters.episodeStep = 0
        self.ControlParameters.state14 = np.zeros((14,))
        self.ControlParameters.state7 = np.zeros((7,))
        self.parking_brake_released = False
        
        # ToDo if collision flag true then Reset
        if self.ControlParameters.isCollision:
            self.ControlParameters.isCollision = False
            self.collisionDetector.reset_collision_state()
            print("Reset -- Collision State")

        if not self.parking_brake_released:
            try:
                #self.client.sendDREF("sim/flightmodel/controls/parkbrake", 0.0)
                self.client.parkingBrakeRelease()
                self.parking_brake_released = True
            except Exception as e:
                print(f"[ERROR] Failed to release parking brake: {e}")

        # ✅ Set initial aircraft position to (0,0,0)
        initial_pos = [47.46362653786365, -122.31786739901888, 99.44899999070913, 2.168961524963379, 3.0465428829193115, 180.4329071044922, 1.0]
        self.client.resetPOSI(initial_pos)

        # try:
        #     initial_pos = [47.46362653786365, -122.31786739901888, 99.44899999070913, 2.168961524963379, 3.0465428829193115, 180.4329071044922, 1.0]
        #     self.client.sendPOSI(initial_pos)

        #     self.client.setDREF("sim/flightmodel/position/local_vx", 0.0)
        #     self.client.setDREF("sim/flightmodel/position/local_vy", 0.0)
        #     self.client.setDREF("sim/flightmodel/position/local_vz", 0.0)
        #     self.client.setDREF("sim/flightmodel/position/P", 0.0)  # roll rate
        #     self.client.setDREF("sim/flightmodel/position/Q", 0.0)  # pitch rate
        #     self.client.setDREF("sim/flightmodel/position/R", 0.0)  # yaw rate

        #     self.client.sendDATA([[25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            
        #     print(f"[INFO] Aircraft respawned at initial position: {initial_pos}")
        # except Exception as e:
        #     print(f"[ERROR] Failed to set initial position: {e}")

        # try:
        #     self.client.sendDATA([[25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        #     print("[INFO] Throttle reset to 0% for all engines")
        # except Exception as e:
        #     print(f"[ERROR] Failed to reset throttle: {e}")

        # try:
        #     self.client.sendDREF("sim/cockpit2/switches/custom_slider_on", 0.0)
        # except Exception as e:
        #     print(f"[ERROR] Failed to reset reset_status: {e}")

        # start_time = time.time()
        # timeout = 10.0    #Timeout occurs when connection is broken, connection should be back within 10 seconds
        # while time.time() - start_time < timeout:
        #     try:
        #         reset_complete = self.client.getDREF("sim/cockpit2/switches/custom_slider_on")[0]
        #         if reset_complete == 1.0:
        #             print("[INFO] Reset completed in X-Plane")
        #             break
        #     except Exception as e:
        #         print(f"[WARNING] Error checking reset status: {e}")
        #     time.sleep(0.5)
        # else:
        #     # ✅ COUNT RESET TIMEOUT AS TIMEOUT INCIDENT
        #     self.timeout_incidents += 1
        #     print("[WARNING] Reset timeout: X-Plane did not signal completion")

        # Convert observation to numpy array with correct dtype
        observation = np.array(self.ControlParameters.state7, dtype=np.float32)
        
        # Create info dictionary
        info = {
            "episode_steps": self.episode_steps,
            "reset_reason": "manual_reset" if options is None else options.get("reason", "manual_reset"),
            "timeout_incidents": self.timeout_incidents
        }
        
        # Return tuple (observation, info) as required by gymnasium
        return observation, info
from xplane_client import XPlaneClient
import gymnasium as gym
import parameters
import space_definition as envSpaces
import time

# IMPORTANT: This version is for a real X-Plane connection.
# The mock classes have been removed.

clientXp = XPlaneClient()
m_e_s = 100

class checkXplaneVal(gym.Env):
    def __init__(self, client=clientXp, max_episode_steps = m_e_s):
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
        
        # DREFs for position retrieval
        self.drefs = [
            "sim/flightmodel/position/elevation",         # 0 - Altitude MSL (meters)
            "sim/flightmodel/position/theta",             # 1 - Pitch (degrees)
            "sim/flightmodel/position/phi",               # 2 - Roll (degrees)
            "sim/flightmodel/position/psi",               # 3 - Heading (degrees)
            "sim/flightmodel/position/true_airspeed"      # 4 - True Airspeed
            "sim/cockpit2/engine/actuators/throttle_ratio_all", # 5 - Current throttle
            "sim/flightmodel/position/Q",                 # 6 - Pitch rate (deg/sec)
            "sim/flightmodel/position/P",                 # 7 - Roll rate (deg/sec)
            
            "sim/flightmodel/position/R",                 # 8 - Yaw rate (deg/sec)
            "sim/flightmodel/position/vh_ind_fpm",        # 9 - Vertical speed (fpm)
            "sim/flightmodel/position/indicated_airspeed",# 10 - indicate-Airspeed (knots)
            
            # "sim/flightmodel/controls/wing_sweep_ratio"   # 10 - For stability info
        ]

        self.drefs2 = ["sim/cockpit2/controls/yoke_pitch_ratio","sim/cockpit2/controls/yoke_roll_ratio",
		"sim/cockpit2/controls/yoke_heading_ratio","sim/flightmodel/position/alpha",
		"sim/cockpit2/controls/wingsweep_ratio","sim/cockpit2/controls/flap_ratio",
		"sim/flightmodel/position/groundspeed"]

    def get_dref_position(self):
        """
        Retrieves aircraft position and orientation data from X-Plane using getDREFs.
        Returns a dictionary with the values.
        """
        try:
            dref_values = self.client.getDREFs(self.drefs)
            print(f"The value of dref_values is: ---------------------------- > {dref_values}")
            keys = [
                "altitude_msl (Mean Sea Level)",
                "pitch",
                "roll",
                "heading",
                "true_airspeed",
                "throttle_ratio",
                "pitch_rate",
                "roll_rate",
                "yaw_rate",
                "vertical_speed",
                "indicated_airspeed",
            ]

            # Clean values (if empty/None, set to 0)
            safe_values = [val[0] if val and len(val) > 0 and val[0] is not None else 0 for val in dref_values]

            # Return dictionary
            return dict(zip(keys, safe_values))
        except Exception as e:
            print(f"[ERROR] An error occurred while fetching DREF data: {e}")
            return None

    def get_posi_data(self):
        """
        Retrieves aircraft position data from X-Plane using getPOSI.
        Returns a dictionary with the values.
        """
        try:
            posi_data = self.client.getPOSI()
            if not posi_data or len(posi_data) < 6:
                print("[ERROR] Failed to retrieve POSI data.")
                return None

            posi_dict = {
                "latitude": posi_data[0],
                "longitude": posi_data[1],
                "altitude_msl": posi_data[2],
                "pitch": posi_data[3],
                "roll": posi_data[4],
                "heading": posi_data[5],
            }
            return posi_dict
        except Exception as e:
            print(f"[ERROR] An error occurred while fetching POSI data: {e}")
            return None

# --- Main execution block to demonstrate the functions ---
# This part is outside the class, at the top level of the file.
if __name__ == "__main__":
    # Instantiate the environment
    env = checkXplaneVal()

    print("Starting loop to get X-Plane data...")

    # Loop 50 times
    for i in range(50):
        print(f"\n--- Iteration {i+1}/50 ---")
        
        # Call the get_dref_position function
        dref_values = env.get_dref_position()
        if dref_values:
            print("DREF Data:")
            for key, value in dref_values.items():
                print(f"  {key}: {value}")
        
        # Call the get_posi_data function
        posi_values = env.get_posi_data()
        if posi_values:
            print("POSI Data:")
            for key, value in posi_values.items():
                print(f"  {key}: {value}")
        
        # Sleep for a second to avoid overloading the connection
        time.sleep(1)

    print("\nLoop completed.")

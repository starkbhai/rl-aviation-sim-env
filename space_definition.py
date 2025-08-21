import numpy as np
from gymnasium import spaces

class xplane_space:
    def _action_space(self):
        # return spaces.Box(
        #     low=np.array([-1, -1, -1, -1, 0, 0, -0.5], dtype=np.float32),
        #     high=np.array([1, 1, 1, 1, 1, 1, 1.5], dtype=np.float32),
        #     dtype=np.float32
        # )
        """
        4-Action Space for Essential Flight Controls:
        Action[0]: Elevator (Pitch Control) - Range [-1, 1]
        Action[1]: Aileron (Roll Control)   - Range [-1, 1] 
        Action[2]: Rudder (Yaw Control)     - Range [-1, 1]
        Action[3]: Throttle (Engine Power)  - Range [-0.25, 1] (allows reverse thrust)
        """
        return spaces.Box(
            low=np.array([-1, -1, -1, 0], dtype=np.float32),
            high=np.array([1, 1, 1, 1], dtype=np.float32),
            dtype=np.float32
        )


    def _observation_space(self):
        # 14-dimensional flat state vector: [lat, long, alt, pitch, roll, heading, + 8 others]
        """
        7-element observation space:
        [altitude, pitch, roll, heading, latitude, longitude, true_speed]
        """
        return spaces.Box(low=-1e10, high=1e10, shape=(7,), dtype=np.float32)



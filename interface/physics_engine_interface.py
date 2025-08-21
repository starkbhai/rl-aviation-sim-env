#!/usr/bin/env python3

import numpy as np
import time
from abc import ABC, abstractmethod
from enum import Enum


class PhysicsEngine(Enum):
    """Enum for selecting physics engine"""
    JSBSIM = "jsbsim"
    XPLANE = "xplane"


class FlightInterface(ABC):
    """
    Abstract base class for flight simulation interface
    Defines common methods that both JSBSim and X-Plane must implement
    """
    
    @abstractmethod
    def sendCTRL(self, actions):
        """Send control actions to aircraft"""
        pass
    
    @abstractmethod
    def getObs(self, use_dref=True):
        """Get unified observation data"""
        pass
    
    @abstractmethod
    def getPOSI(self):
        """Get position data"""
        pass
    
    @abstractmethod
    def sendPOSI(self, position):
        """Send position data"""
        pass

    @abstractmethod
    def resetPOSI(self, position):
        """Reset position data"""
        pass
    
    @abstractmethod
    def parkingBrakeRelease(self):
        """Release parking brake"""
        pass
    
    @abstractmethod
    def isCrash(self):
        """Check if aircraft has crashed"""
        pass
    
    @abstractmethod
    def close(self):
        """Close connection"""
        pass


class JSBSimInterface(FlightInterface):
    """
    JSBSim implementation of flight interface
    Wraps JSBSim to match X-Plane API
    """
    
    def __init__(self):
        """Initialize JSBSim interface"""
        try:
            from jsb.jsb import JSBSImClient  # Import your JSBSim environment
            self.jsb_client = JSBSImClient()
            self.initialized = self.jsb_client.initializeJsb()
            self.last_observation = None
            self.crash_state = False
            
            if self.initialized:
                print("✅ JSBSim interface initialized successfully")
                # Reset to get initial observation
                self.last_observation = self.jsb_client.resetJsb()
            else:
                print("❌ Failed to initialize JSBSim")
                
        except Exception as e:
            print(f"❌ JSBSim interface initialization failed: {e}")
            self.initialized = False
    
    def sendCTRL(self, actions):
        """
        Send control actions to JSBSim aircraft
        
        Args:
            actions (list): [elevator, aileron, rudder, throttle, flaps, wing_sweep, gear]
                          JSBSim only uses first 4 values
        """
        if not self.initialized:
            return False
        
        try:
            # Extract primary controls (JSBSim uses 4 controls)
            jsb_action = [
                float(actions[0]),  # elevator
                float(actions[1]),  # aileron  
                float(actions[2]),  # rudder
                float(actions[3])   # throttle
            ]
            
            # Send action to JSBSim (no automatic stepping)
            success = self.jsb_client.sendAction(jsb_action)
            return success
            
        except Exception as e:
            print(f"❌ JSBSim sendCTRL failed: {e}")
            return False
    
    def getObs(self, use_dref=True):
        """
        Get unified observation data from JSBSim
        
        Args:
            use_dref (bool): Ignored for JSBSim (always same data source)
            
        Returns:
            list: [altitude, pitch, roll, heading, latitude, longitude, airspeed]
        """
        if not self.initialized:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            # Get observation from JSBSim
            obs = self.jsb_client.getObs()
            
            if obs is None or len(obs) < 7:
                print("❌ JSBSim getObs returned insufficient data")
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # JSBSim observation:
            # [lat, lon, alt_ft, roll, pitch, heading, airspeed, v_speed, throttle_pos, elevator_pos, aileron_pos, rudder_pos]
            # Convert to standard format: [altitude, pitch, roll, heading, latitude, longitude, airspeed]
            unified_obs = [
                float(obs[2]),    # altitude
                float(obs[4]),    # pitch
                float(obs[3]),    # roll
                float(obs[5]),    # heading
                float(obs[0]),    # latitude
                float(obs[1]),    # longitude
                float(obs[6])     # airspeed
            ]
            
            return unified_obs
            
        except Exception as e:
            print(f"❌ JSBSim getObs failed: {e}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    def getPOSI(self):
        """
        Get position data in X-Plane format
        
        Returns:
            list: [latitude, longitude, altitude, pitch, roll, heading, gear]
        """
        if not self.initialized:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        
        try:
            # Get observation from JSBSim
            obs = self.jsb_client.getObs()
            
            if obs is None or len(obs) < 7:
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            
            # JSBSim observation: [lat, lon, alt_ft, roll, pitch, heading, airspeed, v_speed, ...]
            # Convert to X-Plane POSI format
            posi_data = [
                float(obs[0]),    # latitude
                float(obs[1]),    # longitude  
                float(obs[2]),    # altitude (feet)
                float(obs[4]),    # pitch
                float(obs[3]),    # roll
                float(obs[5]),    # heading
                1.0               # gear (always up for JSBSim)
            ]
            
            return posi_data
            
        except Exception as e:
            print(f"❌ JSBSim getPOSI failed: {e}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    
    def sendPOSI(self, position):
        """
        Send position data (reset aircraft position)
        
        Args:
            position (list): [lat, lon, alt, pitch, roll, heading, gear]
        """
        if not self.initialized:
            return False
        
        try:
            # Reset JSBSim to initial position
            self.jsb_client.resetJsb()
            print("[INFO] JSBSim: Aircraft reset to initial position")
            return True
            
        except Exception as e:
            print(f"❌ JSBSim sendPOSI failed: {e}")
            return False
        
    def resetPOSI(self, position):
        """
        Send position data (reset aircraft position)
        
        Args:
            position (list): [lat, lon, alt, pitch, roll, heading, gear]
        """
        if not self.initialized:
            return False
        
        try:
            # Reset JSBSim to initial position
            self.jsb_client.resetJsb()
            print("[INFO] JSBSim: Aircraft reset to initial position")
            return True
            
        except Exception as e:
            print(f"❌ JSBSim sendPOSI failed: {e}")
            return False    
    
    def parkingBrakeRelease(self):
        """Release parking brake"""
        # JSBSim doesn't need parking brake release
        print("[INFO] JSBSim: Parking brake release (not needed)")
        return True
    
    def isCrash(self):
        """
        Check if aircraft has crashed
        
        Returns:
            bool: True if crashed
        """
        if not self.initialized:
            return True
        
        try:
            # Use JSBSim's isDone() method which checks for crash conditions
            return self.jsb_client.isDone()
            
        except Exception as e:
            print(f"❌ JSBSim isCrash failed: {e}")
            return False
    
    def close(self):
        """Close JSBSim interface"""
        try:
            if hasattr(self, 'jsb_client') and self.jsb_client:
                self.jsb_client.close()
            print("✅ JSBSim interface closed")
        except Exception as e:
            print(f"❌ JSBSim close failed: {e}")


class XPlaneInterface(FlightInterface):
    """
    X-Plane implementation of flight interface
    Direct wrapper around existing X-Plane client
    """
    
    def __init__(self):
        """Initialize X-Plane interface"""
        try:
            from xplane_client import XPlaneClient  # Import your X-Plane client
            self.xplane_client = XPlaneClient()
            self.initialized = True
            print("✅ X-Plane interface initialized successfully")
            
        except Exception as e:
            print(f"❌ X-Plane interface initialization failed: {e}")
            self.initialized = False
    
    def sendCTRL(self, actions):
        """Send control actions to X-Plane aircraft"""
        if not self.initialized:
            return False
        
        try:
            self.xplane_client.sendCTRL(actions)
            return True
        except Exception as e:
            print(f"❌ X-Plane sendCTRL failed: {e}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    def getPOSI(self):
        """Get position data from X-Plane"""
        if not self.initialized:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        
        try:
            return list(self.xplane_client.getPOSI())
        except Exception as e:
            print(f"❌ X-Plane getPOSI failed: {e}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    
    def sendDREF(self, dref, value):
        """Send data reference value to X-Plane"""
        if not self.initialized:
            return False
        
        try:
            self.xplane_client.sendDREF(dref, value)
            return True
        except Exception as e:
            print(f"❌ X-Plane sendDREF failed: {e}")
            return False
    
    def sendPOSI(self, position):
        """Send position data to X-Plane"""
        if not self.initialized:
            return False
        
        try:
            self.xplane_client.sendPOSI(position)
            return True
        except Exception as e:
            print(f"❌ X-Plane sendPOSI failed: {e}")
            return False
    
    def resetPOSI(self, position):
        
        try:
            initial_pos = [47.46362653786365, -122.31786739901888, 99.44899999070913, 2.168961524963379, 3.0465428829193115, 180.4329071044922, 1.0]
            self.xplane_client.sendPOSI(initial_pos)

            self.xplane_client.setDREF("sim/flightmodel/position/local_vx", 0.0)
            self.xplane_client.setDREF("sim/flightmodel/position/local_vy", 0.0)
            self.xplane_client.setDREF("sim/flightmodel/position/local_vz", 0.0)
            self.xplane_client.setDREF("sim/flightmodel/position/P", 0.0)  # roll rate
            self.xplane_client.setDREF("sim/flightmodel/position/Q", 0.0)  # pitch rate
            self.xplane_client.setDREF("sim/flightmodel/position/R", 0.0)  # yaw rate

            self.xplane_client.sendDATA([[25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            return True
        except Exception as e:
            print(f"❌ X-Plane senFailed to set initial position: {e}")
        
        try:
            self.client.sendDREF("sim/cockpit2/switches/custom_slider_on", 0.0)
        except Exception as e:
            print(f"[ERROR] Failed to reset reset_status: {e}")
        
        start_time = time.time()
        timeout = 10.0    #Timeout occurs when connection is broken, connection should be back within 10 seconds
        while time.time() - start_time < timeout:
            try:
                reset_complete = self.client.getDREF("sim/cockpit2/switches/custom_slider_on")[0]
                if reset_complete == 1.0:
                    print("[INFO] Reset completed in X-Plane")
                    break
            except Exception as e:
                print(f"[WARNING] Error checking reset status: {e}")
            time.sleep(0.5)
        else:
            # ✅ COUNT RESET TIMEOUT AS TIMEOUT INCIDENT
            self.timeout_incidents += 1
            print("[WARNING] Reset timeout: X-Plane did not signal completion")
     
    def getObs(self, use_dref=True):
        """
        Get unified observation data from X-Plane
        
        Args:
            use_dref (bool): If True, use DREF data; if False, use POSI data
            
        Returns:
            list: [altitude, pitch, roll, heading, latitude, longitude, airspeed]
        """
        if not self.initialized:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        try:
            # Define DREF list for positions
            drefForPositions = [
                "sim/flightmodel/position/elevation",           # 0 - Altitude MSL (meters)
                "sim/flightmodel/position/theta",               # 1 - Pitch (degrees)
                "sim/flightmodel/position/phi",                 # 2 - Roll (degrees)
                "sim/flightmodel/position/psi",                 # 3 - Heading (degrees)
                "sim/flightmodel/position/longitude",           # 4 - Longitude
                "sim/flightmodel/position/latitude",            # 5 - Latitude
                "sim/flightmodel/position/indicated_airspeed"   # 6 - Indicated Airspeed
            ]
            
            # Get both data sources with null checks
            stateAircraftPositionWithDref = self.xplane_client.getDREFs(drefForPositions)
            stateAircraftPositionWithGetPos = self.xplane_client.getPOSI()
            
            # Convert POSI to list if it's not None
            if stateAircraftPositionWithGetPos is not None:
                stateAircraftPositionWithGetPos = list(stateAircraftPositionWithGetPos)
            
            # Validate DREF data with null checks
            if (stateAircraftPositionWithDref is None or 
                not isinstance(stateAircraftPositionWithDref, (list, tuple)) or 
                len(stateAircraftPositionWithDref) < 7):
                print("❌ getDREFs returned None or insufficient data")
                # If DREF fails, try to use POSI data if available
                if (stateAircraftPositionWithGetPos is not None and 
                    isinstance(stateAircraftPositionWithGetPos, (list, tuple)) and 
                    len(stateAircraftPositionWithGetPos) >= 6):
                    print("[INFO] Falling back to POSI data only")
                    return [
                        float(stateAircraftPositionWithGetPos[2]),  # altitude
                        float(stateAircraftPositionWithGetPos[3]),  # pitch
                        float(stateAircraftPositionWithGetPos[4]),  # roll
                        float(stateAircraftPositionWithGetPos[5]),  # heading
                        float(stateAircraftPositionWithGetPos[1]),  # latitude
                        float(stateAircraftPositionWithGetPos[0]),  # longitude
                        0.0  # airspeed (unavailable from POSI)
                    ]
                else:
                    print("❌ Both DREF and POSI data unavailable")
                    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Validate POSI data with null checks
            if (stateAircraftPositionWithGetPos is None or 
                not isinstance(stateAircraftPositionWithGetPos, (list, tuple)) or 
                len(stateAircraftPositionWithGetPos) < 6):
                print("❌ getPOSI returned None or incomplete data")
                if not use_dref:
                    print("[INFO] POSI requested but unavailable, falling back to DREF")
                    use_dref = True  # Force use of DREF since POSI is unavailable

            # Get airspeed from DREF with additional null checks
            airspeed = 0.0
            try:
                if (len(stateAircraftPositionWithDref) > 6 and 
                    stateAircraftPositionWithDref[6] is not None and
                    isinstance(stateAircraftPositionWithDref[6], (list, tuple)) and
                    len(stateAircraftPositionWithDref[6]) > 0):
                    
                    airspeed_raw = stateAircraftPositionWithDref[6][0]
                    if airspeed_raw is not None and airspeed_raw != "":
                        airspeed = max(0.0, float(airspeed_raw))
                    else:
                        airspeed = 0.0
                else:
                    airspeed = 0.0
                    
            except (IndexError, TypeError, ValueError) as e:
                airspeed = 0.0
                print(f"[INFO] Airspeed unavailable: {e}")

            # Choose data source based on use_dref flag and data availability
            if use_dref and stateAircraftPositionWithDref is not None:
                try:
                    # Use DREF data for flight parameters with null checks
                    flight_data = []
                    for i in range(4):  # altitude, pitch, roll, heading
                        if (i < len(stateAircraftPositionWithDref) and 
                            stateAircraftPositionWithDref[i] is not None and
                            isinstance(stateAircraftPositionWithDref[i], (list, tuple)) and
                            len(stateAircraftPositionWithDref[i]) > 0):
                            flight_data.append(float(stateAircraftPositionWithDref[i][0]))
                        else:
                            flight_data.append(0.0)
                    
                    # Get latitude and longitude with null checks
                    if (len(stateAircraftPositionWithDref) > 5 and
                        stateAircraftPositionWithDref[5] is not None and
                        isinstance(stateAircraftPositionWithDref[5], (list, tuple)) and
                        len(stateAircraftPositionWithDref[5]) > 0):
                        latitude = float(stateAircraftPositionWithDref[5][0])
                    else:
                        latitude = 0.0
                        
                    if (len(stateAircraftPositionWithDref) > 4 and
                        stateAircraftPositionWithDref[4] is not None and
                        isinstance(stateAircraftPositionWithDref[4], (list, tuple)) and
                        len(stateAircraftPositionWithDref[4]) > 0):
                        longitude = float(stateAircraftPositionWithDref[4][0])
                    else:
                        longitude = 0.0
                    
                    print("[INFO] Using DREF-based observation")
                    
                except (IndexError, TypeError, ValueError) as e:
                    print(f"[WARNING] DREF data parsing failed: {e}, falling back to POSI")
                    use_dref = False
            
            if not use_dref and stateAircraftPositionWithGetPos is not None:
                try:
                    # Use POSI data for flight parameters (but airspeed still from DREF)
                    flight_data = [
                        float(stateAircraftPositionWithGetPos[2]),   # altitude (index 2 in POSI)
                        float(stateAircraftPositionWithGetPos[3]),   # pitch (index 3 in POSI)
                        float(stateAircraftPositionWithGetPos[4]),   # roll (index 4 in POSI)
                        float(stateAircraftPositionWithGetPos[5])    # heading (index 5 in POSI)
                    ]
                    latitude = float(stateAircraftPositionWithGetPos[1])    # latitude from POSI
                    longitude = float(stateAircraftPositionWithGetPos[0])   # longitude from POSI
                    print("[INFO] Using POSI-based observation (airspeed from DREF)")
                    
                except (IndexError, TypeError, ValueError) as e:
                    print(f"[WARNING] POSI data parsing failed: {e}, using default values")
                    flight_data = [0.0, 0.0, 0.0, 0.0]
                    latitude = 0.0
                    longitude = 0.0
            
            # If we get here and don't have flight_data, set defaults
            if 'flight_data' not in locals():
                print("[WARNING] No valid data source available, using defaults")
                flight_data = [0.0, 0.0, 0.0, 0.0]
                latitude = 0.0
                longitude = 0.0
            
            # Convert to standard format: [altitude, pitch, roll, heading, latitude, longitude, airspeed]
            unified_obs = [
                float(flight_data[0]),  # altitude
                float(flight_data[1]),  # pitch
                float(flight_data[2]),  # roll
                float(flight_data[3]),  # heading
                float(latitude),        # latitude
                float(longitude),       # longitude
                float(airspeed)         # airspeed (always from DREF)
            ]

            return unified_obs
            
        except Exception as e:
            print(f"❌ X-Plane getObs (OBSERVAT) failed: {e}")
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
    def parkingBrakeRelease(self):
        """Release parking brake in X-Plane"""
        return self.sendDREF("sim/flightmodel/controls/parkbrake", 0.0)
    
    def isCrash(self):
        """Check if aircraft has crashed in X-Plane"""
        if not self.initialized:
            return True
        
        try:
            # You'll need to define crash detection logic for your X-Plane setup
            crash_drefs = ["sim/flightmodel2/misc/has_crashed"]  # Example DREF
            crash_data = self.xplane_client.getDREFs(crash_drefs)
            return crash_data[0][0] == 1.0 if crash_data and crash_data[0] else False
        except Exception as e:
            print(f"❌ X-Plane isCrash failed: {e}")
            return False
    
    def close(self):
        """Close X-Plane interface"""
        try:
            if hasattr(self, 'xplane_client') and self.xplane_client:
                self.xplane_client.close()
            print("✅ X-Plane interface closed")
        except Exception as e:
            print(f"❌ X-Plane close failed: {e}")


class UnifiedFlightInterface:
    """
    Unified interface that can switch between JSBSim and X-Plane
    """
    
    def __init__(self, physics_engine=PhysicsEngine.XPLANE):
        """
        Initialize unified flight interface
        
        Args:
            physics_engine (PhysicsEngine): Which physics engine to use
        """
        self.physics_engine = physics_engine
        self.interface = None
        
        print(f"🚀 Initializing {physics_engine.value.upper()} interface...")
        
        if physics_engine == PhysicsEngine.JSBSIM:
            self.interface = JSBSimInterface()
        elif physics_engine == PhysicsEngine.XPLANE:
            self.interface = XPlaneInterface()
        else:
            raise ValueError(f"Unsupported physics engine: {physics_engine}")
        
        if not self.interface.initialized:
            raise RuntimeError(f"Failed to initialize {physics_engine.value} interface")
        
        print(f"✅ {physics_engine.value.upper()} interface ready!")
    
    def sendCTRL(self, actions):
        """Send control actions to aircraft"""
        return self.interface.sendCTRL(actions)
    
    def getPOSI(self):
        """Get position data"""
        return self.interface.getPOSI()
    
    def getObs(self, use_dref=True):
        """Get unified observation data"""
        return self.interface.getObs(use_dref)
    
    def sendPOSI(self, position):
        """Send position data"""
        return self.interface.sendPOSI(position)
    
    def resetPOSI(self, position):
        """Reset Position Data"""
        return self.interface.resetPOSI(position)
    
    def parkingBrakeRelease(self):
        """Release parking brake"""
        return self.interface.parkingBrakeRelease()
    
    def isCrash(self):
        """Check if aircraft has crashed"""
        return self.interface.isCrash()
    
    def close(self):
        """Close interface"""
        if self.interface:
            self.interface.close()
        print(f"✅ {self.physics_engine.value.upper()} interface closed")
    
    def get_engine_type(self):
        """Get current physics engine type"""
        return self.physics_engine
    
    def is_jsbsim(self):
        """Check if using JSBSim"""
        return self.physics_engine == PhysicsEngine.JSBSIM
    
    def is_xplane(self):
        """Check if using X-Plane"""
        return self.physics_engine == PhysicsEngine.XPLANE


# Factory function for easy creation
def create_flight_interface(engine_type="xplane"):
    """
    Factory function to create flight interface
    
    Args:
        engine_type (str): "jsbsim" or "xplane"
        
    Returns:
        UnifiedFlightInterface: Configured interface
    """
    if engine_type.lower() == "jsbsim":
        return UnifiedFlightInterface(PhysicsEngine.JSBSIM)
    elif engine_type.lower() == "xplane":
        return UnifiedFlightInterface(PhysicsEngine.XPLANE)
    else:
        raise ValueError(f"Invalid engine type: {engine_type}. Use 'jsbsim' or 'xplane'")


# Example usage and testing
def test_interface(engine_type="xplane"):
    """Test the unified interface"""
    print(f"🧪 Testing {engine_type.upper()} interface...")
    
    try:
        # Create interface
        client = create_flight_interface(engine_type)
        
        print(f"✅ Interface created successfully")
        print(f"📊 Engine type: {client.get_engine_type().value}")
        
        # Test basic operations
        print("\n🎮 Testing basic operations...")
        
        # Test position
        position = client.getPOSI()
        print(f"📍 Position: {position}")
        
        # Test controls
        test_action = [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # Neutral with 50% throttle
        success = client.sendCTRL(test_action)
        print(f"🎮 Control send: {'✅' if success else '❌'}")
        
        # Test DREFs
        test_drefs = ["sim/flightmodel/position/theta", "sim/flightmodel/position/phi"]
        dref_data = client.getDREFs(test_drefs)
        print(f"📊 DREF data: {dref_data}")
        
        # Test crash detection
        crash_status = client.isCrash()
        print(f"💥 Crash status: {crash_status}")
        
        # Test parking brake
        brake_success = client.parkingBrakeRelease()
        print(f"🚗 Brake release: {'✅' if brake_success else '❌'}")
        
        print(f"\n✅ {engine_type.upper()} interface test completed successfully!")
        
        # Clean up
        client.close()
        
    except Exception as e:
        print(f"❌ Interface test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    print("🛩️  Unified Flight Interface Testing")
    print("=" * 50)
    
    # Test both interfaces
    test_interface("jsbsim")
    print("\n" + "-" * 30 + "\n")
    test_interface("xplane")
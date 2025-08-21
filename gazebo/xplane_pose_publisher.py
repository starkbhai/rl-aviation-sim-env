#!/usr/bin/env python3

import math
import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from xplane_client import XPlaneClient

# Import gz-transport and gz-msgs
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

class FinalGZTransportXPlane:
    def __init__(self, reference_lat=None, reference_lon=None, reference_alt=None):
        """
        Final working X-Plane to Gazebo bridge using gz-transport
        """
        self.client = None
        self.connected = False
        
        # Reference point (origin in Gazebo world)
        self.ref_lat = reference_lat
        self.ref_lon = reference_lon
        self.ref_alt = reference_alt
        self.ref_heading = None
        
        # Earth radius in meters
        self.EARTH_RADIUS = 6378137.0
        
        # Connection retry settings
        self.max_retries = 3
        self.consecutive_failures = 0
        self.max_consecutive_failures = 10
        
        # Initialize gz-transport node
        self.gz_node = Node()
        self.pose_service = "/world/default/set_pose"
        
        print("🚀 Final gz-transport X-Plane Bridge")
        self.connect_to_xplane()
    
    def connect_to_xplane(self):
        """Establish connection to X-Plane"""
        try:
            print("🔌 Connecting to X-Plane...")
            self.client = XPlaneClient()
            
            # Test connection
            test_data = self.client.getDREF("sim/time/total_running_time_sec")
            if test_data and len(test_data) > 0:
                self.connected = True
                self.consecutive_failures = 0
                print("✅ Connected to X-Plane successfully!")
                return True
            else:
                print("❌ X-Plane connection test failed")
                return False
                
        except Exception as e:
            print(f"❌ Failed to connect to X-Plane: {e}")
            self.connected = False
            return False
    
    def reconnect_if_needed(self):
        """Try to reconnect if connection is lost"""
        if not self.connected:
            print("🔄 Attempting to reconnect to X-Plane...")
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
            return self.connect_to_xplane()
        return True
    
    def set_reference_point(self, lat, lon, alt, heading):
        """Set the reference point and initial heading for coordinate conversion"""
        self.ref_lat = lat
        self.ref_lon = lon
        self.ref_alt = alt
        self.ref_heading = heading
        print(f"📍 Reference point set: ({lat:.6f}, {lon:.6f}, {alt:.2f}m, {heading:.1f}°)")
    
    def gps_to_local_with_heading(self, lat, lon, alt, heading):
        """Convert GPS coordinates to local XYZ coordinates with heading correction"""
        if self.ref_lat is None or self.ref_lon is None or self.ref_alt is None:
            return 0, 0, 0
        
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)
        
        # Calculate differences in meters
        dlat = lat_rad - ref_lat_rad
        dlon = lon_rad - ref_lon_rad
        
        # Convert to meters in Earth coordinate system
        north_m = dlat * self.EARTH_RADIUS
        east_m = dlon * self.EARTH_RADIUS * math.cos(ref_lat_rad)
        up_m = alt - self.ref_alt
        
        # If we have a reference heading, rotate coordinates so initial heading becomes forward
        if self.ref_heading is not None:
            # Calculate angle offset
            angle_offset = math.radians(self.ref_heading)
            
            # Rotate coordinates: initial heading direction becomes +X (forward)
            # Fix: Flip Y coordinate to correct left/right direction
            gazebo_x = north_m * math.cos(angle_offset) + east_m * math.sin(angle_offset)
            gazebo_y = north_m * math.sin(angle_offset) - east_m * math.cos(angle_offset)  # Flipped Y
            gazebo_z = up_m
        else:
            # Default: North=+X, East=+Y (with Y flipped to fix left/right)
            gazebo_x = north_m
            gazebo_y = -east_m   # Flip Y coordinate
            gazebo_z = up_m
        
        return gazebo_x, gazebo_y, gazebo_z
    
    def euler_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        """Convert Euler angles to quaternion with coordinate system correction and heading offset"""
        # Apply heading offset to align X-Plane heading with Gazebo forward direction
        if self.ref_heading is not None:
            # Subtract the initial heading so the aircraft points forward in Gazebo
            corrected_yaw = yaw_deg - self.ref_heading
        else:
            corrected_yaw = yaw_deg
        
        # Convert X-Plane angles to Gazebo convention (flip directions)
        gazebo_roll = roll_deg         # Keep roll same direction
        gazebo_pitch = -pitch_deg      # Flip pitch direction  
        gazebo_yaw = -corrected_yaw    # Flip yaw direction
        
        # Convert degrees to radians
        roll = math.radians(gazebo_roll)
        pitch = math.radians(gazebo_pitch)
        yaw = math.radians(gazebo_yaw)
        
        # Convert to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def get_xplane_data(self):
        """Get aircraft data from X-Plane with retry logic"""
        if not self.connected:
            if not self.reconnect_if_needed():
                return None
        
        datarefs = [
            "sim/flightmodel/position/elevation",    # altitude (feet)
            "sim/flightmodel/position/longitude",    # longitude
            "sim/flightmodel/position/latitude",     # latitude
            "sim/flightmodel/position/theta",        # pitch (degrees)
            "sim/flightmodel/position/phi",          # roll (degrees)
            "sim/flightmodel/position/psi"           # heading (degrees)
        ]
        
        for attempt in range(self.max_retries):
            try:
                values = self.client.getDREFs(datarefs)
                
                if not values or len(values) < 6:
                    raise ValueError("Incomplete data received")
                
                for val_list in values:
                    if not val_list or len(val_list) == 0:
                        raise ValueError("Empty dataref value")
                
                # Reset failure counter on success
                self.consecutive_failures = 0
                
                return {
                    'altitude_ft': values[0][0],
                    'longitude': values[1][0],
                    'latitude': values[2][0], 
                    'pitch': values[3][0],
                    'roll': values[4][0],
                    'heading': values[5][0]
                }
                
            except Exception as e:
                self.consecutive_failures += 1
                if attempt < self.max_retries - 1:
                    time.sleep(0.1)  # Brief delay before retry
                else:
                    # Mark as disconnected after all retries fail
                    self.connected = False
                    
                    # Only print warnings occasionally to avoid spam
                    if self.consecutive_failures % 20 == 0:
                        print(f"[WARNING] X-Plane data failed: {e}")
                        
                        if self.consecutive_failures >= self.max_consecutive_failures:
                            print("⚠️ Many consecutive X-Plane failures!")
                            print("   Check: X-Plane running, aircraft loaded, plugin active")
                        
        return None
    
    def send_pose_gz_transport(self, x, y, z, roll, pitch, yaw, model_name="aircraft"):
        """
        Send pose to Gazebo using working gz-transport method
        """
        try:
            # Convert to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
            
            # Create Pose message
            pose_msg = Pose()
            pose_msg.name = model_name
            
            # Set position
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z
            
            # Set orientation
            pose_msg.orientation.x = qx
            pose_msg.orientation.y = qy
            pose_msg.orientation.z = qz
            pose_msg.orientation.w = qw
            
            # Send request using working API signature
            success = self.gz_node.request(
                self.pose_service,  # service name
                pose_msg,          # request message
                Pose,              # request type
                Boolean,           # response type
                100                # timeout in ms (short for high frequency)
            )
            
            return success
            
        except Exception as e:
            # Only print errors occasionally to avoid spam
            if hasattr(self, '_error_count'):
                self._error_count += 1
            else:
                self._error_count = 1
                
            if self._error_count % 100 == 0:
                print(f"[WARNING] gz-transport error (#{self._error_count}): {e}")
            
            return False
    
    def run_bridge(self, update_rate=20, model_name="aircraft"):
        """
        Main bridge loop using gz-transport
        """
        print(f"🚀 Starting gz-transport bridge at {update_rate} Hz...")
        print("Press Ctrl+C to stop")
        
        if not self.connected:
            print("❌ Not connected to X-Plane. Exiting...")
            return
        
        loop_count = 0
        dt = 1.0 / update_rate
        successful_updates = 0
        gazebo_successes = 0
        
        try:
            while True:
                loop_count += 1
                start_time = time.time()
                
                # Get data from X-Plane
                data = self.get_xplane_data()
                
                if data is not None:
                    successful_updates += 1
                    
                    # Set reference point from first valid reading
                    if self.ref_lat is None:
                        # Convert altitude from feet to meters
                        alt_m = data['altitude_ft'] * 0.3048
                        self.set_reference_point(
                            data['latitude'], 
                            data['longitude'], 
                            alt_m,
                            data['heading']  # Store initial heading
                        )
                    
                    # Convert GPS to local coordinates with heading correction
                    alt_m = data['altitude_ft'] * 0.3048  # feet to meters
                    x, y, z = self.gps_to_local_with_heading(
                        data['latitude'],
                        data['longitude'], 
                        alt_m,
                        data['heading']
                    )
                    
                    # Send to Gazebo using gz-transport
                    gazebo_success = self.send_pose_gz_transport(
                        x, y, z,
                        data['roll'], 
                        data['pitch'],
                        data['heading'],
                        model_name
                    )
                    if gazebo_success:
                        gazebo_successes += 1
                    
                    # Print status every 50 loops
                    if loop_count % 50 == 0:
                        xplane_rate = (successful_updates / loop_count) * 100
                        gazebo_rate = (gazebo_successes / loop_count) * 100
                        print(f"\n--- Loop {loop_count} ---")
                        print(f"✈️  X-Plane: {xplane_rate:.1f}% | 🚀 gz-transport: {gazebo_rate:.1f}%")
                        print(f"📍 Position: X={x:.2f}, Y={y:.2f}, Z={z:.2f} (meters)")
                        print(f"🎭 Attitude: R={data['roll']:.1f}°, P={data['pitch']:.1f}°, Y={data['heading']:.1f}°")
                        
                        # Show movement hint if aircraft isn't moving
                        if abs(x) < 0.1 and abs(y) < 0.1 and abs(z) < 0.1:
                            print("💡 Aircraft appears stationary - try moving it in X-Plane!")
                
                # Maintain update rate
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Shutting down gz-transport bridge...")
            
            if loop_count > 0:
                xplane_rate = (successful_updates / loop_count) * 100
                gazebo_rate = (gazebo_successes / loop_count) * 100
                print(f"📊 Final Statistics:")
                print(f"   ✈️  X-Plane reads: {successful_updates}/{loop_count} ({xplane_rate:.1f}%)")
                print(f"   🚀 gz-transport: {gazebo_successes}/{loop_count} ({gazebo_rate:.1f}%)")
                
        finally:
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
            print("✅ gz-transport bridge closed.")

# Test function
def test_manual_poses():
    """Test manual poses to verify everything works"""
    print("🧪 Testing manual poses...")
    
    bridge = FinalGZTransportXPlane()
    
    test_positions = [
        (0, 0, 2, 0, 0, 0, "Reset to origin"),
        (5, 0, 2, 0, 0, 0, "Move forward"),
        (5, 5, 2, 0, 0, 45, "Move right with turn"),
        (0, 0, 5, 0, 0, 0, "Move up"),
        (0, 0, 2, 0, 0, 0, "Back to start"),
    ]
    
    for x, y, z, roll, pitch, yaw, description in test_positions:
        print(f"\n🎯 {description}: ({x}, {y}, {z}) yaw={yaw}°")
        
        success = bridge.send_pose_gz_transport(x, y, z, roll, pitch, yaw)
        
        print(f"   Result: {'✅ Success' if success else '❌ Failed'}")
        print(f"   👀 Check Gazebo - aircraft should be at ({x}, {y}, {z})")
        
        time.sleep(2)

if __name__ == "__main__":
    # Option 1: Run the bridge
    bridge = FinalGZTransportXPlane()
    bridge.run_bridge(update_rate=20, model_name="aircraft")
    
    # Option 2: Test manual poses (uncomment to use)
    # test_manual_poses()
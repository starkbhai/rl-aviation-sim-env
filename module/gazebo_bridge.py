#!/usr/bin/env python3

import math
import time

# Import gz-transport and gz-msgs
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

class GazeboBridge:
    def __init__(self, reference_lat=None, reference_lon=None, reference_alt=None):
        """
        Gazebo bridge - subscribes to pose data and publishes to Gazebo
        """
        # Reference point (origin in Gazebo world)
        self.ref_lat = reference_lat
        self.ref_lon = reference_lon
        self.ref_alt = reference_alt
        self.ref_heading = None
        
        # Earth radius in meters
        self.EARTH_RADIUS = 6378137.0
        
        # Initialize gz-transport node
        self.gz_node = Node()
        self.subscribe_topic = "/gazebo/input_pose"
        self.pose_service = "/world/default/set_pose"
        
        # Statistics
        self.loop_count = 0
        self.successful_conversions = 0
        self.gazebo_successes = 0
        
        print("🚀 Gazebo Bridge Starting")
        print(f"🎧 Subscribing to topic: {self.subscribe_topic}")
        
        # Create subscriber using correct API
        if self.gz_node.subscribe(Pose, self.subscribe_topic, self.pose_callback):
            print("✅ Subscriber created successfully")
        else:
            print("❌ Failed to create subscriber")
            
        print("Press Ctrl+C to stop")
    
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
    
    def pose_callback(self, msg: Pose):
        """Callback function for incoming pose messages"""
        try:
            self.loop_count += 1
            
            # Extract data from the message
            # Data is stored as: lat=x, lon=y, alt_ft=z, roll=qx, pitch=qy, heading=qz
            data = {
                'latitude': msg.position.x,
                'longitude': msg.position.y,
                'altitude_ft': msg.position.z,
                'roll': msg.orientation.x,
                'pitch': msg.orientation.y,
                'heading': msg.orientation.z
            }
            
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
            
            self.successful_conversions += 1
            
            # Send to Gazebo using gz-transport
            gazebo_success = self.send_pose_gz_transport(
                x, y, z,
                data['roll'], 
                data['pitch'],
                data['heading'],
                "aircraft"  # model name
            )
            if gazebo_success:
                self.gazebo_successes += 1
            
            # Print status every 50 messages
            if self.loop_count % 50 == 0:
                conversion_rate = (self.successful_conversions / self.loop_count) * 100
                gazebo_rate = (self.gazebo_successes / self.loop_count) * 100
                print(f"\n--- Message {self.loop_count} ---")
                print(f"🎧 Received: 100% | 🔄 Conversion: {conversion_rate:.1f}% | 🚀 Gazebo: {gazebo_rate:.1f}%")
                print(f"📍 Position: X={x:.2f}, Y={y:.2f}, Z={z:.2f} (meters)")
                print(f"🎭 Attitude: R={data['roll']:.1f}°, P={data['pitch']:.1f}°, Y={data['heading']:.1f}°")
                
                # Show movement hint if aircraft isn't moving
                if abs(x) < 0.1 and abs(y) < 0.1 and abs(z) < 0.1:
                    print("💡 Aircraft appears stationary - try moving it in the simulator!")
                    
        except Exception as e:
            print(f"[ERROR] Callback processing failed: {e}")
    
    def run_bridge(self):
        """
        Main bridge loop - keeps the subscriber alive
        """
        print("🚀 Gazebo bridge is running...")
        print("Waiting for pose messages...")
        
        try:
            # Keep the program alive to receive messages
            while True:
                time.sleep(0.001)  # Small sleep to prevent busy waiting
                
        except KeyboardInterrupt:
            print(f"\n🛑 Shutting down Gazebo bridge...")
            
            if self.loop_count > 0:
                conversion_rate = (self.successful_conversions / self.loop_count) * 100
                gazebo_rate = (self.gazebo_successes / self.loop_count) * 100
                print(f"📊 Final Statistics:")
                print(f"   🎧 Messages received: {self.loop_count}")
                print(f"   🔄 Successful conversions: {self.successful_conversions}/{self.loop_count} ({conversion_rate:.1f}%)")
                print(f"   🚀 Gazebo updates: {self.gazebo_successes}/{self.loop_count} ({gazebo_rate:.1f}%)")
                
        finally:
            print("✅ Gazebo bridge closed.")

# Test function for manual poses
def test_manual_poses():
    """Test manual poses to verify Gazebo connection works"""
    print("🧪 Testing manual poses...")
    
    bridge = GazeboBridge()
    
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
    # Option 1: Run the bridge (default)
    bridge = GazeboBridge()
    bridge.run_bridge()
    
    # Option 2: Test manual poses (uncomment to use)
    # test_manual_poses()
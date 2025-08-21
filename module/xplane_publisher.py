#!/usr/bin/env python3

import time
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from xplane_client import XPlaneClient

# Import gz-transport and gz-msgs
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose

class XPlanePublisher:
    def __init__(self):
        """
        X-Plane data publisher - reads from X-Plane and publishes to gz-transport topic
        """
        self.client = None
        self.connected = False
        
        # Connection retry settings
        self.max_retries = 3
        self.consecutive_failures = 0
        self.max_consecutive_failures = 10
        
        # Initialize gz-transport node for publishing
        self.gz_node = Node()
        self.publish_topic = "/gazebo/input_pose"
        
        # Create publisher using correct API
        self.publisher = self.gz_node.advertise(self.publish_topic, Pose)
        
        print("🛩️ X-Plane Publisher Starting")
        print(f"📡 Publishing on topic: {self.publish_topic}")
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
    
    def publish_pose_data(self, data):
        """Publish X-Plane data to gz-transport topic"""
        try:
            # Create Pose message with X-Plane data
            pose_msg = Pose()
            
            # Store X-Plane data in the pose message
            # Using position fields for GPS coordinates
            pose_msg.position.x = data['latitude']     # Store latitude in x
            pose_msg.position.y = data['longitude']    # Store longitude in y  
            pose_msg.position.z = data['altitude_ft']  # Store altitude in z
            
            # Store attitude in orientation (as Euler angles temporarily)
            pose_msg.orientation.x = data['roll']      # Roll
            pose_msg.orientation.y = data['pitch']     # Pitch
            pose_msg.orientation.z = data['heading']   # Heading/Yaw
            pose_msg.orientation.w = 1.0               # Marker to indicate this is Euler data
            
            # Skip timestamp for now - may not be required
            
            # Publish using correct API (returns boolean)
            success = self.publisher.publish(pose_msg)
            return success
            
        except Exception as e:
            print(f"[ERROR] Publishing failed: {e}")
            return False
    
    def run_publisher(self, update_rate=20):
        """
        Main publisher loop
        """
        print(f"🚀 Starting X-Plane publisher at {update_rate} Hz...")
        print("Press Ctrl+C to stop")
        
        if not self.connected:
            print("❌ Not connected to X-Plane. Exiting...")
            return
        
        loop_count = 0
        dt = 1.0 / update_rate
        successful_reads = 0
        successful_publishes = 0
        
        try:
            while True:
                loop_count += 1
                start_time = time.time()
                
                # Get data from X-Plane
                data = self.get_xplane_data()
                
                if data is not None:
                    successful_reads += 1
                    
                    # Publish to gz-transport topic
                    pub_success = self.publish_pose_data(data)
                    if pub_success:
                        successful_publishes += 1
                    
                    # Print status every 100 loops
                    if loop_count % 100 == 0:
                        read_rate = (successful_reads / loop_count) * 100
                        pub_rate = (successful_publishes / loop_count) * 100
                        print(f"\n--- Loop {loop_count} ---")
                        print(f"✈️  X-Plane reads: {read_rate:.1f}% | 📡 Publishing: {pub_rate:.1f}%")
                        print(f"📍 GPS: Lat={data['latitude']:.6f}, Lon={data['longitude']:.6f}, Alt={data['altitude_ft']:.1f}ft")
                        print(f"🎭 Attitude: R={data['roll']:.1f}°, P={data['pitch']:.1f}°, Y={data['heading']:.1f}°")
                
                # Maintain update rate
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Shutting down X-Plane publisher...")
            
            if loop_count > 0:
                read_rate = (successful_reads / loop_count) * 100
                pub_rate = (successful_publishes / loop_count) * 100
                print(f"📊 Final Statistics:")
                print(f"   ✈️  X-Plane reads: {successful_reads}/{loop_count} ({read_rate:.1f}%)")
                print(f"   📡 Publishing: {successful_publishes}/{loop_count} ({pub_rate:.1f}%)")
                
        finally:
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
            print("✅ X-Plane publisher closed.")

if __name__ == "__main__":
    publisher = XPlanePublisher()
    publisher.run_publisher(update_rate=20)
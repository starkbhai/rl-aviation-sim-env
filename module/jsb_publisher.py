#!/usr/bin/env python3

import time
import sys
import os
import socket
import json
import threading
from queue import Queue, Empty

# Import gz-transport and gz-msgs
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose

class JSBSimPublisher:
    def __init__(self, listen_port=5555):
        """
        JSBSim data publisher - listens on port for observation data from JSBSim
        
        Args:
            listen_port (int): Port to listen for JSBSim observation data
        """
        self.listen_port = listen_port
        self.listen_socket = None
        self.listening = False
        self.listen_thread = None
        
        # Data queue for thread-safe communication
        self.data_queue = Queue(maxsize=10)
        self.last_data = None
        self.last_data_time = 0
        
        # Connection statistics
        self.packets_received = 0
        self.packets_processed = 0
        self.connection_active = False
        
        # Initialize gz-transport node for publishing
        self.gz_node = Node()
        self.publish_topic = "/gazebo/input_pose"
        
        # Create publisher using correct API
        self.publisher = self.gz_node.advertise(self.publish_topic, Pose)
        
        print("🛩️ JSBSim Publisher Starting")
        print(f"👂 Listening on port: {self.listen_port}")
        print(f"📡 Publishing on topic: {self.publish_topic}")
        
        # Start listening for data
        self.start_listening()
    
    def start_listening(self):
        """Start listening for JSBSim observation data on UDP port"""
        try:
            # Create UDP socket for receiving data
            self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.listen_socket.bind(('localhost', self.listen_port))
            self.listen_socket.settimeout(1.0)  # 1 second timeout
            
            self.listening = True
            
            # Start background thread for listening
            self.listen_thread = threading.Thread(target=self._listening_loop, daemon=True)
            self.listen_thread.start()
            
            print(f"✅ Started listening on port {self.listen_port}")
            
        except Exception as e:
            print(f"❌ Failed to start listening: {e}")
            self.listening = False
    
    def stop_listening(self):
        """Stop listening for data"""
        self.listening = False
        if self.listen_socket:
            self.listen_socket.close()
            self.listen_socket = None
        print("🛑 Stopped listening")
    
    def _listening_loop(self):
        """Background loop to continuously listen for observation data"""
        print("👂 Listening for JSBSim observation data...")
        consecutive_timeouts = 0
        
        while self.listening:
            try:
                # Receive data from JSBSim
                data, addr = self.listen_socket.recvfrom(4096)  # Buffer size
                
                # Parse JSON data
                obs_data = json.loads(data.decode('utf-8'))
                
                # Update statistics
                self.packets_received += 1
                consecutive_timeouts = 0
                
                if not self.connection_active:
                    print("✅ Connected to JSBSim data stream!")
                    self.connection_active = True
                
                # Add to queue (non-blocking, drop old data if queue is full)
                try:
                    self.data_queue.put_nowait(obs_data)
                except:
                    # Queue is full, remove old data and add new
                    try:
                        self.data_queue.get_nowait()
                        self.data_queue.put_nowait(obs_data)
                    except:
                        pass
                
            except socket.timeout:
                consecutive_timeouts += 1
                
                # Print status occasionally during timeouts
                if consecutive_timeouts == 5:  # After 5 seconds
                    print("⚠️ No data received from JSBSim for 5 seconds")
                    print("   Make sure JSBSim is running and publishing observations")
                    self.connection_active = False
                elif consecutive_timeouts % 30 == 0:  # Every 30 seconds
                    print(f"⚠️ Still waiting for JSBSim data... ({consecutive_timeouts}s)")
                
            except json.JSONDecodeError as e:
                print(f"[WARNING] Invalid JSON data received: {e}")
                
            except Exception as e:
                if self.listening:  # Only print if we're supposed to be listening
                    print(f"[WARNING] Listening error: {e}")
                time.sleep(0.1)
    
    def get_latest_data(self):
        """Get the latest observation data from the queue"""
        latest_data = None
        
        # Get all available data (keep only the latest)
        while True:
            try:
                latest_data = self.data_queue.get_nowait()
                self.packets_processed += 1
            except Empty:
                break
        
        if latest_data:
            self.last_data = latest_data
            self.last_data_time = time.time()
        
        return latest_data
    
    def is_data_fresh(self, max_age=1.0):
        """Check if we have fresh data within max_age seconds"""
        if self.last_data_time == 0:
            return False
        return (time.time() - self.last_data_time) < max_age
    
    def publish_pose_data(self, data):
        """Publish JSBSim data to gz-transport topic"""
        try:
            # Create Pose message with JSBSim data
            pose_msg = Pose()
            
            # Store JSBSim data in the pose message
            pose_msg.position.x = data['latitude']     # Store latitude in x
            pose_msg.position.y = data['longitude']    # Store longitude in y  
            pose_msg.position.z = data['altitude_ft']  # Store altitude in z
            
            # Store attitude in orientation (as Euler angles temporarily)
            pose_msg.orientation.x = data['roll']      # Roll
            pose_msg.orientation.y = data['pitch']     # Pitch
            pose_msg.orientation.z = data['heading']   # Heading/Yaw
            pose_msg.orientation.w = 1.0               # Marker to indicate this is Euler data
            
            # Publish using correct API (returns boolean)
            success = self.publisher.publish(pose_msg)
            return success
            
        except Exception as e:
            print(f"[ERROR] Publishing failed: {e}")
            return False
    
    def show_connection_instructions(self):
        """Show instructions for connecting to JSBSim"""
        print("\n" + "="*60)
        print("❌ No JSBSim data stream detected!")
        print("="*60)
        print("💡 To use this publisher:")
        print()
        print("1. 🚀 Start your RL training script with JSBSim:")
        print("   python train_new_rl_model.py")
        print()
        print("2. 📡 Make sure JSBSim is publishing observations to port")
        print(f"   (Publisher listening on port {self.listen_port})")
        print()
        print("3. 🔄 JSBSim should automatically start sending data")
        print("   when it initializes")
        print()
        print("⚠️  Make sure JSBSim is configured to publish to the right port!")
        print("="*60)
    
    def run_publisher(self, update_rate=20):
        """
        Main publisher loop - listens for data and publishes to Gazebo
        """
        print(f"🚀 Starting JSBSim publisher at {update_rate} Hz...")
        print("Press Ctrl+C to stop")
        
        if not self.listening:
            print("❌ Not listening for data. Cannot start publisher!")
            return
        
        loop_count = 0
        dt = 1.0 / update_rate
        successful_publishes = 0
        data_available_count = 0
        last_status_time = 0
        
        # Show initial instructions
        self.show_connection_instructions()
        
        try:
            while True:
                loop_count += 1
                start_time = time.time()
                
                # Get latest observation data from queue
                data = self.get_latest_data()
                
                if data is not None:
                    data_available_count += 1
                    
                    # Publish to gz-transport topic
                    pub_success = self.publish_pose_data(data)
                    if pub_success:
                        successful_publishes += 1
                    
                    # Print status every 100 loops
                    if loop_count % 100 == 0:
                        data_rate = (data_available_count / loop_count) * 100
                        pub_rate = (successful_publishes / loop_count) * 100
                        packet_loss = max(0, self.packets_received - self.packets_processed)
                        
                        print(f"\n--- Loop {loop_count} ---")
                        print(f"📡 Data availability: {data_rate:.1f}% | Publishing: {pub_rate:.1f}%")
                        print(f"📦 Packets: Received={self.packets_received}, Processed={self.packets_processed}, Loss={packet_loss}")
                        print(f"📍 GPS: Lat={data['latitude']:.6f}, Lon={data['longitude']:.6f}, Alt={data['altitude_ft']:.1f}ft")
                        print(f"🎭 Attitude: R={data['roll']:.1f}°, P={data['pitch']:.1f}°, Y={data['heading']:.1f}°")
                        print(f"🚀 Speed: {data['airspeed_kts']:.1f}kts | ⬆️ V/S: {data.get('vertical_speed_fps', 0)*60:.0f}fpm")
                        print(f"🎮 Controls: T={data.get('throttle_pos', 0):.2f}, E={data.get('elevator_pos', 0):.2f}")
                        print(f"⏱️ Data age: {time.time() - data['timestamp']:.3f}s")
                
                else:
                    # No new data available
                    current_time = time.time()
                    
                    # Check if we have any recent data
                    if self.is_data_fresh(max_age=2.0):
                        # We have recent data, just no new updates this loop
                        pass
                    else:
                        # No recent data at all
                        if (current_time - last_status_time) > 5.0:  # Print every 5 seconds
                            if self.connection_active:
                                print(f"⚠️ No fresh data from JSBSim (Loop {loop_count})")
                            else:
                                print(f"⚠️ Waiting for JSBSim connection... (Loop {loop_count})")
                            last_status_time = current_time
                
                # Maintain update rate
                elapsed = time.time() - start_time
                sleep_time = max(0, dt - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Shutting down JSBSim publisher...")
            
            if loop_count > 0:
                data_rate = (data_available_count / loop_count) * 100
                pub_rate = (successful_publishes / loop_count) * 100
                packet_loss = max(0, self.packets_received - self.packets_processed)
                
                print(f"📊 Final Statistics:")
                print(f"   📡 Data availability: {data_available_count}/{loop_count} ({data_rate:.1f}%)")
                print(f"   📡 Publishing success: {successful_publishes}/{loop_count} ({pub_rate:.1f}%)")
                print(f"   📦 Total packets received: {self.packets_received}")
                print(f"   📦 Total packets processed: {self.packets_processed}")
                print(f"   📦 Packet loss: {packet_loss}")
                
        finally:
            self.stop_listening()
            print("✅ JSBSim publisher closed.")

def main():
    """Main function"""
    print("🛩️ JSBSim Publisher (Port-based Communication)")
    print("=" * 60)
    
    # Create publisher with custom port if needed
    import argparse
    parser = argparse.ArgumentParser(description='JSBSim Publisher')
    parser.add_argument('--port', type=int, default=5555, help='Port to listen for JSBSim data')
    args = parser.parse_args()
    
    # Create publisher
    publisher = JSBSimPublisher(listen_port=args.port)
    
    # Run publisher loop
    publisher.run_publisher(update_rate=20)

if __name__ == "__main__":
    main()
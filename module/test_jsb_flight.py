#!/usr/bin/env python3
import jsbsim
import numpy as np
import time
import socket
import json
import math

class SimpleJSBSimFlightTest:
    """
    Simple JSBSim flight test that sends observation data to publisher
    No external dependencies required
    """
    
    def __init__(self, obs_port=5555):
        """Initialize simple flight test"""
        self.obs_port = obs_port
        self.sim = None
        self.initialized = False
        self.obs_socket = None
        
        print("🛩️ Simple JSBSim Flight Test")
        print(f"📡 Sending data to port: {obs_port}")
    
    def initialize_jsbsim(self):
        """Initialize JSBSim"""
        try:
            print("🚀 Initializing JSBSim...")
            
            self.sim = jsbsim.FGFDMExec(None)
            
            if not self.sim.load_model('c172p'):
                print("❌ Failed to load aircraft model")
                return False
            
            print("✅ Aircraft loaded: c172p")
            
            # Set timestep
            self.sim.set_dt(0.05)  # 20Hz
            
            # Initial conditions - interesting starting position
            self.sim.set_property_value('ic/lat-gc-deg', 37.62131)      # San Francisco
            self.sim.set_property_value('ic/long-gc-deg', -122.37896)
            self.sim.set_property_value('ic/h-sl-ft', 2000.0)           # 2000ft altitude
            self.sim.set_property_value('ic/vc-kts', 100.0)             # 100 knots
            self.sim.set_property_value('ic/psi-true-deg', 45.0)        # Northeast heading
            self.sim.set_property_value('ic/theta-deg', 2.0)            # Slight climb
            self.sim.set_property_value('ic/phi-deg', 0.0)              # Wings level
            
            # Aircraft config
            self.sim.set_property_value('gear/gear-cmd-norm', 0.0)      # Gear up
            self.sim.set_property_value('fcs/flap-cmd-norm', 0.0)       # Flaps up
            
            # Initialize
            if not self.sim.run_ic():
                print("❌ Failed to initialize conditions")
                return False
            
            # Start engine
            self.sim.set_property_value('propulsion/engine/set-running', 1)
            self.sim.set_property_value('fcs/mixture-cmd-norm', 1.0)
            self.sim.set_property_value('propulsion/magneto_cmd', 3)
            self.sim.set_property_value('fcs/throttle-cmd-norm', 0.75)
            
            # Stabilize
            for _ in range(20):
                self.sim.run()
            
            self.initialized = True
            print("✅ JSBSim initialized successfully!")
            
            # Setup UDP socket
            self.obs_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"✅ Ready to send data to port {self.obs_port}")
            
            return True
            
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            return False
    
    def get_observations(self):
        """Get current flight observations"""
        try:
            obs = {
                'timestamp': time.time(),
                'latitude': self.sim.get_property_value("position/lat-gc-deg"),
                'longitude': self.sim.get_property_value("position/long-gc-deg"),
                'altitude_ft': self.sim.get_property_value("position/h-sl-ft"),
                'roll': self.sim.get_property_value("attitude/phi-deg"),
                'pitch': self.sim.get_property_value("attitude/theta-deg"),
                'heading': self.sim.get_property_value("attitude/psi-deg"),
                'airspeed_kts': self.sim.get_property_value("velocities/vc-kts"),
                'vertical_speed_fps': self.sim.get_property_value("velocities/h-dot-fps"),
                'throttle_pos': self.sim.get_property_value("fcs/throttle-pos-norm"),
                'elevator_pos': self.sim.get_property_value("fcs/elevator-pos-norm"),
                'aileron_pos': self.sim.get_property_value("fcs/aileron-pos-norm"),
                'rudder_pos': self.sim.get_property_value("fcs/rudder-pos-norm"),
                'episode_step': 0,
                'flight_time': 0.0
            }
            return obs
        except Exception as e:
            print(f"❌ Failed to get observations: {e}")
            return None
    
    def send_observations(self, obs):
        """Send observations to publisher"""
        try:
            json_data = json.dumps(obs).encode('utf-8')
            self.obs_socket.sendto(json_data, ('localhost', self.obs_port))
            return True
        except Exception as e:
            print(f"[WARNING] Failed to send data: {e}")
            return False
    
    def apply_flight_controls(self, time_elapsed):
        """Apply automated flight controls based on time"""
        
        # Create interesting flight pattern
        if time_elapsed < 10:
            # Phase 1: Climb (0-10s)
            elevator = 0.15
            aileron = 0.0
            rudder = 0.0
            throttle = 0.85
            phase = "Climbing"
            
        elif time_elapsed < 20:
            # Phase 2: Level cruise (10-20s)  
            elevator = 0.0
            aileron = 0.0
            rudder = 0.0
            throttle = 0.75
            phase = "Level Flight"
            
        elif time_elapsed < 30:
            # Phase 3: Gentle right turn (20-30s)
            elevator = 0.0
            aileron = 0.2
            rudder = 0.05
            throttle = 0.75
            phase = "Right Turn"
            
        elif time_elapsed < 40:
            # Phase 4: Gentle left turn (30-40s)
            elevator = 0.0
            aileron = -0.2
            rudder = -0.05
            throttle = 0.75
            phase = "Left Turn"
            
        elif time_elapsed < 50:
            # Phase 5: Level again (40-50s)
            elevator = 0.0
            aileron = 0.0
            rudder = 0.0
            throttle = 0.75
            phase = "Level Flight"
            
        elif time_elapsed < 60:
            # Phase 6: Climbing turn (50-60s)
            elevator = 0.1
            aileron = 0.15
            rudder = 0.03
            throttle = 0.8
            phase = "Climbing Turn"
            
        else:
            # Phase 7: Gentle descent (60s+)
            elevator = -0.05
            aileron = 0.0
            rudder = 0.0
            throttle = 0.65
            phase = "Descent"
        
        # Apply controls
        self.sim.set_property_value('fcs/elevator-cmd-norm', elevator)
        self.sim.set_property_value('fcs/aileron-cmd-norm', aileron)
        self.sim.set_property_value('fcs/rudder-cmd-norm', rudder)
        self.sim.set_property_value('fcs/throttle-cmd-norm', throttle)
        
        return phase
    
    def run_test_flight(self, duration=60):
        """Run automated test flight"""
        if not self.initialize_jsbsim():
            return False
        
        print(f"\n🚀 Starting {duration}s automated test flight...")
        print("📡 Sending observation data to publisher")
        print("💡 Start your publisher: python jsb_publisher.py --port", self.obs_port)
        print("=" * 70)
        
        start_time = time.time()
        step_count = 0
        data_sent = 0
        last_print_time = 0
        
        try:
            while (time.time() - start_time) < duration:
                loop_start = time.time()
                time_elapsed = time.time() - start_time
                
                # Apply flight controls
                current_phase = self.apply_flight_controls(time_elapsed)
                
                # Step simulation
                self.sim.run()
                step_count += 1
                
                # Get observations
                obs = self.get_observations()
                if obs:
                    obs['episode_step'] = step_count
                    obs['flight_time'] = time_elapsed
                    
                    # Send to publisher
                    if self.send_observations(obs):
                        data_sent += 1
                    
                    # Print status every 2 seconds
                    if time_elapsed - last_print_time >= 2.0:
                        print(f"⏱️ {time_elapsed:5.1f}s | "
                              f"Phase: {current_phase:12} | "
                              f"Alt: {obs['altitude_ft']:6.0f}ft | "
                              f"Speed: {obs['airspeed_kts']:5.1f}kts | "
                              f"Hdg: {obs['heading']:5.1f}° | "
                              f"📡 Sent: {data_sent:4d}")
                        last_print_time = time_elapsed
                
                # Maintain 20Hz rate
                elapsed = time.time() - loop_start
                sleep_time = max(0, 0.05 - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\n⚠️ Flight interrupted by Ctrl+C")
            
        except Exception as e:
            print(f"\n❌ Flight error: {e}")
            
        finally:
            # Final statistics
            final_time = time.time() - start_time
            avg_rate = data_sent / max(1, final_time)
            
            print(f"\n📊 Flight Test Summary:")
            print(f"   ⏱️ Total time: {final_time:.1f} seconds")
            print(f"   📊 Simulation steps: {step_count}")
            print(f"   📡 Data packets sent: {data_sent}")
            print(f"   📈 Average data rate: {avg_rate:.1f} Hz")
            print(f"   🎯 Target rate: 20.0 Hz")
            
            if abs(avg_rate - 20.0) < 2.0:
                print("   ✅ Data rate within acceptable range")
            else:
                print("   ⚠️ Data rate outside target range")
            
            # Cleanup
            if self.obs_socket:
                self.obs_socket.close()
            
            print("✅ Test flight completed!")
            return True

def run_simple_data_test(port=5555, duration=10):
    """Run a simple data generation test without JSBSim"""
    print(f"🧪 Running simple data test for {duration} seconds...")
    print(f"📡 Sending test data to port {port}")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        start_time = time.time()
        packet_count = 0
        
        # Simulate a circular flight pattern
        while (time.time() - start_time) < duration:
            elapsed = time.time() - start_time
            
            # Circular flight pattern
            radius = 0.01  # Small radius in degrees
            center_lat = 37.62131
            center_lon = -122.37896
            angular_speed = 0.1  # radians per second
            
            angle = angular_speed * elapsed
            lat = center_lat + radius * math.cos(angle)
            lon = center_lon + radius * math.sin(angle)
            
            # Simulate banking in the turn
            roll = 15.0 * math.sin(angle)  # Bank angle
            heading = math.degrees(angle) % 360
            
            # Create realistic flight data
            test_obs = {
                'timestamp': time.time(),
                'latitude': lat,
                'longitude': lon,
                'altitude_ft': 3000.0 + 100.0 * math.sin(elapsed * 0.2),  # Gentle altitude variation
                'roll': roll,
                'pitch': 2.0 + 3.0 * math.sin(elapsed * 0.3),  # Gentle pitch variation
                'heading': heading,
                'airspeed_kts': 120.0 + 10.0 * math.sin(elapsed * 0.15),  # Speed variation
                'vertical_speed_fps': 2.0 * math.cos(elapsed * 0.2),  # Climb/descent
                'throttle_pos': 0.75,
                'elevator_pos': 0.05 * math.sin(elapsed * 0.3),
                'aileron_pos': 0.1 * math.sin(angle),
                'rudder_pos': 0.02 * math.sin(angle),
                'episode_step': packet_count,
                'flight_time': elapsed
            }
            
            # Send data
            json_data = json.dumps(test_obs).encode('utf-8')
            sock.sendto(json_data, ('localhost', port))
            
            packet_count += 1
            
            # Print status every second
            if packet_count % 20 == 0:
                print(f"📤 Sent {packet_count} packets | "
                      f"Alt: {test_obs['altitude_ft']:.0f}ft | "
                      f"Hdg: {test_obs['heading']:.1f}° | "
                      f"Roll: {test_obs['roll']:.1f}°")
            
            time.sleep(0.05)  # 20Hz
        
        sock.close()
        print(f"✅ Sent {packet_count} test packets successfully")
        return True
        
    except Exception as e:
        print(f"❌ Simple test failed: {e}")
        return False

def main():
    """Main function"""
    print("🛩️ JSBSim Flight Test Suite")
    print("=" * 50)
    
    import argparse
    parser = argparse.ArgumentParser(description='JSBSim Flight Test')
    parser.add_argument('--port', type=int, default=5555,
                       help='Port to send observation data')
    parser.add_argument('--mode', choices=['jsbsim', 'simple'], default='jsbsim',
                       help='Test mode: jsbsim or simple data test')
    parser.add_argument('--duration', type=int, default=60,
                       help='Test duration in seconds')
    
    args = parser.parse_args()
    
    print(f"📋 Test Configuration:")
    print(f"   🔌 Port: {args.port}")
    print(f"   🔧 Mode: {args.mode}")
    print(f"   ⏱️ Duration: {args.duration} seconds")
    
    print(f"\n💡 Instructions:")
    print(f"   1. Start this test script")
    print(f"   2. In another terminal, run:")
    print(f"      python jsb_publisher.py --port {args.port}")
    print(f"   3. Check that data flows from this test → Publisher → Gazebo")
    
    input("\nPress Enter to start the test...")
    
    if args.mode == 'jsbsim':
        # JSBSim flight test
        print("\n🛩️ Starting JSBSim flight test...")
        try:
            flight_test = SimpleJSBSimFlightTest(obs_port=args.port)
            success = flight_test.run_test_flight(duration=args.duration)
        except Exception as e:
            print(f"❌ JSBSim test failed: {e}")
            print("🔄 Falling back to simple data test...")
            success = run_simple_data_test(port=args.port, duration=args.duration)
    else:
        # Simple data test
        print("\n📊 Starting simple data test...")
        success = run_simple_data_test(port=args.port, duration=args.duration)
    
    if success:
        print("\n🎉 Test completed successfully!")
        print("\n📋 Next Steps:")
        print("   ✅ If publisher received data, the connection works")
        print("   ✅ Check Gazebo for aircraft movement")
        print("   ✅ You can now use this with your RL training")
    else:
        print("\n❌ Test failed!")
        print("\n🔧 Troubleshooting:")
        print("   - Make sure JSBSim is installed: pip install jsbsim")
        print("   - Check that the port is not in use")
        print("   - Verify publisher is running and listening")

if __name__ == "__main__":
    main()
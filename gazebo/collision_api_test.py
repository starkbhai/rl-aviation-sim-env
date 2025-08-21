#!/usr/bin/env python3

import time
from gz_transport_collision_detector import AircraftCollisionDetector

def test_collision_apis():
    """Simple test of collision detector APIs"""
    
    print("🧪 Testing Collision Detector APIs")
    print("="*40)
    
    # Create detector
    detector = AircraftCollisionDetector()
    
    print("\n📡 Waiting for collision... (move aircraft to touch ground)")
    
    # Test loop
    for i in range(30):  # Run for 30 seconds
        
        # Test all the APIs
        is_colliding = detector.is_colliding()
        collision_entity = detector.get_collision_entity()
        collision_status = detector.get_collision_status()
        has_collision = detector.has_collision_occurred()
        
        current_time = time.strftime("%H:%M:%S")
        
        print(f"\n[{current_time}] Test {i+1}:")
        print(f"  is_colliding():         {is_colliding}")
        print(f"  get_collision_entity(): {collision_entity}")
        print(f"  get_collision_status(): {collision_status}")
        print(f"  has_collision_occurred(): {has_collision}")
        
        # Test reset after collision is detected
        if is_colliding and i % 5 == 0:  # Reset every 5 iterations when colliding
            print(f"  🔄 Calling reset_collision_state()...")
            detector.reset_collision_state()
            
            # Check status after reset
            print(f"  After reset - is_colliding(): {detector.is_colliding()}")
        
        time.sleep(1)
    
    print("\n✅ API test completed!")

if __name__ == "__main__":
    test_collision_apis()
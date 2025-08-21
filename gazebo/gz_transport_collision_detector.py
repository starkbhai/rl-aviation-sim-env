#!/usr/bin/env python3

import time
import threading
from typing import Tuple, Optional

import sys
sys.path.append("/usr/lib/python3/dist-packages") 
import gz.transport13

class AircraftCollisionDetector:
    def __init__(self):
        """
        Simple collision detector - callbacks work automatically!
        No background thread needed - Gazebo Transport handles it.
        """
        
        # Import Gazebo Transport
        try:
            import gz.transport13 as gz_transport
            from gz.msgs10 import contacts_pb2
            
            self.gz_transport = gz_transport
            self.contacts_pb2 = contacts_pb2
            
        except ImportError as e:
            print(f"❌ Import error: {e}")
            raise ImportError("Gazebo Transport not available")
        
        # Create node
        self.node = self.gz_transport.Node()
        
        # Collision state (thread-safe for callbacks)
        self._lock = threading.Lock()
        self._aircraft_colliding = False
        self._collision_entity = None
        self._last_update_time = None
        
        # Fuselage contact topic
        self.fuselage_topic = "/world/default/model/aircraft/link/fuselage/sensor/fuselage_contact/contact"
        
        # Subscribe - callbacks will work automatically!
        self._setup_subscriber()
        
        print("✅ Aircraft Collision Detector ready (callbacks active)")
        
    def _setup_subscriber(self):
        """Subscribe to fuselage contact sensor"""
        
        def fuselage_callback(msg):
            self._fuselage_contact_callback(msg)
        
        if self.node.subscribe(self.contacts_pb2.Contacts, self.fuselage_topic, fuselage_callback):
            print("✅ Subscribed to fuselage contact sensor")
        else:
            raise RuntimeError("Failed to subscribe to fuselage contact sensor")
    
    def _fuselage_contact_callback(self, msg):
        """Fuselage contact callback - runs automatically in background!"""
        
        with self._lock:
            self._last_update_time = time.time()
            
            if len(msg.contact) > 0:
                # Aircraft is colliding
                contact = msg.contact[0]
                
                # Find what we're colliding with
                if 'aircraft' in contact.collision1.name:
                    entity = contact.collision2.name
                else:
                    entity = contact.collision1.name
                
                # Clean up entity name
                entity = entity.replace('::collision', '')
                
                # Update state
                was_colliding = self._aircraft_colliding
                self._aircraft_colliding = True
                self._collision_entity = entity
                
                # Log state change
                if not was_colliding:
                    print(f"🚨 AIRCRAFT COLLISION with {entity}")
                
            else:
                # No collision
                was_colliding = self._aircraft_colliding
                self._aircraft_colliding = False
                self._collision_entity = None
                
                # Log state change
                if was_colliding:
                    print("✅ Aircraft collision cleared")
    
    def is_colliding(self) -> bool:
        """Check if aircraft is currently colliding"""
        with self._lock:
            return self._aircraft_colliding
    
    def get_collision_entity(self) -> Optional[str]:
        """Get the entity the aircraft is colliding with"""
        with self._lock:
            return self._collision_entity
    
    def get_collision_status(self) -> Tuple[bool, str]:
        """Get complete collision status"""
        with self._lock:
            if self._aircraft_colliding:
                return True, self._collision_entity or "Unknown entity"
            else:
                return False, "No collision"
    
    def get_last_update_time(self) -> Optional[float]:
        """Get timestamp of last sensor update"""
        with self._lock:
            return self._last_update_time
    
    def is_sensor_active(self, timeout: float = 5.0) -> bool:
        """Check if sensor is actively receiving data"""
        with self._lock:
            if self._last_update_time is None:
                return False
            return (time.time() - self._last_update_time) < timeout
    
    def reset_collision_state(self):
        """
        Manually reset collision state to False
        Perfect for gym environment reset() calls
        """
        with self._lock:
            was_colliding = self._aircraft_colliding
            self._aircraft_colliding = False
            self._collision_entity = None
            
            if was_colliding:
                print("🔄 Collision state manually reset")
    
    def has_collision_occurred(self) -> bool:
        """
        Alias for is_colliding() - more intuitive for gym environments
        Returns True if collision has occurred and hasn't been reset
        """
        return self.is_colliding()

# Usage examples
if __name__ == '__main__':
    import signal
    import sys
    
    def signal_handler(sig, frame):
        print("\n\nShutting down...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create detector - callbacks start working immediately!
    detector = AircraftCollisionDetector()
    
    print("\n" + "="*50)
    print("✈️  SIMPLE MODULE COLLISION DETECTOR")
    print("="*50)
    print("Callbacks running automatically in background!")
    print("Press Ctrl+C to stop...")
    print("="*50 + "\n")
    
    # Simple status loop
    iteration = 0
    while True:
        iteration += 1
        
        collision_detected, entity = detector.get_collision_status()
        current_time = time.strftime("%H:%M:%S")
        
        if collision_detected:
            print(f"[{current_time}] 🚨 COLLISION: {entity}")
        else:
            print(f"[{current_time}] ✅ No collision (iter: {iteration})")
        
        time.sleep(1.0)
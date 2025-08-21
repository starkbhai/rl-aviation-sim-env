#!/usr/bin/env python3

import time
import threading
from typing import Tuple, Optional

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
        
        # Collision filtering settings
        self._ignored_entities = set()
        self._ground_entities = {
            "ground::ground_link",
            "ground_plane",
            "ground",
            "plane"
        }
        self._ignore_ground = False
        
        # Smart collision tracking for prioritization
        self._current_collisions = set()  # Track all current collisions
        self._priority_entity = None      # Current priority entity
        
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
        """Fuselage contact callback with smart entity prioritization"""
        
        with self._lock:
            self._last_update_time = time.time()
            
            if len(msg.contact) > 0:
                # Extract all entities from this contact message
                entities_in_contact = set()
                
                for contact in msg.contact:
                    # Find what we're colliding with
                    if 'aircraft' in contact.collision1.name:
                        entity = contact.collision2.name
                    else:
                        entity = contact.collision1.name
                    
                    # Clean up entity name
                    entity = entity.replace('::collision', '')
                    
                    # Add to current contact set if not ignored
                    if not self._should_ignore_entity(entity):
                        entities_in_contact.add(entity)
                
                # Update our collision tracking
                self._current_collisions = entities_in_contact
                
                if entities_in_contact:
                    # We have valid (non-ignored) collisions
                    self._aircraft_colliding = True
                    
                    # Smart entity selection with prioritization
                    selected_entity = self._select_priority_entity(entities_in_contact)
                    
                    # Update if priority changed
                    if self._collision_entity != selected_entity:
                        old_entity = self._collision_entity
                        self._collision_entity = selected_entity
                        
                        if old_entity is None:
                            print(f"🚨 AIRCRAFT COLLISION with {selected_entity}")
                        else:
                            print(f"🔄 COLLISION PRIORITY: {old_entity} → {selected_entity}")
                else:
                    # No valid collisions (all filtered out)
                    if self._aircraft_colliding:
                        print("🔄 All collisions filtered - state cleared")
                    self._aircraft_colliding = False
                    self._collision_entity = None
                    self._current_collisions = set()
                    
            else:
                # No contacts in this message - but maintain sticky behavior
                # Don't clear state here since we only get messages when contact exists
                pass
    
    def _select_priority_entity(self, entities: set) -> str:
        """
        Select highest priority entity from current collisions
        Priority: Non-ground > Ground (when filtering is on)
        """
        if not entities:
            return None
        
        # If ground filtering is OFF, return any entity (first one)
        if not self._ignore_ground:
            return next(iter(entities))
        
        # Ground filtering is ON - prioritize non-ground entities
        non_ground_entities = {e for e in entities if not self._is_ground_entity(e)}
        
        if non_ground_entities:
            # Return highest priority non-ground entity
            return next(iter(non_ground_entities))
        else:
            # Only ground entities (shouldn't happen since they'd be filtered)
            return next(iter(entities))
    
    def _should_ignore_entity(self, entity: str) -> bool:
        """Check if entity should be ignored based on filter settings"""
        
        # Check if entity is in ignored list
        if entity in self._ignored_entities:
            return True
        
        # Check if ground collision should be ignored
        if self._ignore_ground and self._is_ground_entity(entity):
            return True
        
        return False
    
    def _is_ground_entity(self, entity: str) -> bool:
        """Check if entity is considered ground"""
        entity_lower = entity.lower()
        
        # Check against known ground entity names
        for ground_name in self._ground_entities:
            if ground_name.lower() in entity_lower:
                return True
        
        return False
    
    def set_ignore_ground(self, ignore: bool):
        """
        Enable/disable ground collision detection
        
        Args:
            ignore: If True, ground collisions will be ignored
        """
        with self._lock:
            self._ignore_ground = ignore
            
            # Re-evaluate current collision state with new filter settings
            if self._current_collisions:
                # Filter current collisions based on new settings
                valid_entities = {e for e in self._current_collisions if not self._should_ignore_entity(e)}
                
                if valid_entities:
                    # We have valid non-ignored entities
                    selected_entity = self._select_priority_entity(valid_entities)
                    
                    if self._collision_entity != selected_entity:
                        old_entity = self._collision_entity
                        self._aircraft_colliding = True
                        self._collision_entity = selected_entity
                        print(f"🔄 Filter change: {old_entity} → {selected_entity}")
                    
                else:
                    # All current collisions are now ignored
                    if self._aircraft_colliding:
                        print("🔄 All collisions now ignored - state cleared")
                    self._aircraft_colliding = False
                    self._collision_entity = None
        
        print(f"🌍 Ground collision detection: {'OFF' if ignore else 'ON'}")
    
    def add_ignored_entity(self, entity_name: str):
        """
        Add entity to ignore list
        
        Args:
            entity_name: Name of entity to ignore (e.g., "building::wall")
        """
        with self._lock:
            self._ignored_entities.add(entity_name)
            
            # If currently colliding with this entity, clear state
            if self._aircraft_colliding and self._collision_entity == entity_name:
                self._aircraft_colliding = False
                self._collision_entity = None
                print(f"🔄 {entity_name} collision ignored - state cleared")
        
        print(f"➕ Added '{entity_name}' to ignore list")
    
    def remove_ignored_entity(self, entity_name: str):
        """
        Remove entity from ignore list
        
        Args:
            entity_name: Name of entity to stop ignoring
        """
        with self._lock:
            self._ignored_entities.discard(entity_name)
        
        print(f"➖ Removed '{entity_name}' from ignore list")
    
    def clear_ignored_entities(self):
        """Clear all ignored entities"""
        with self._lock:
            self._ignored_entities.clear()
        
        print("🗑️ Cleared all ignored entities")
    
    def get_ignored_entities(self) -> set:
        """Get list of currently ignored entities"""
        with self._lock:
            return self._ignored_entities.copy()
    
    def is_ignoring_ground(self) -> bool:
        """Check if ground collisions are being ignored"""
        with self._lock:
            return self._ignore_ground
    
    def add_ground_entity(self, entity_name: str):
        """
        Add entity name to be considered as 'ground'
        
        Args:
            entity_name: Entity name to treat as ground
        """
        with self._lock:
            self._ground_entities.add(entity_name)
        
        print(f"🌍 Added '{entity_name}' to ground entities")
    
    def get_current_collisions(self) -> set:
        """Get all entities currently in collision (for debugging)"""
        with self._lock:
            return self._current_collisions.copy()
    
    def get_collision_summary(self) -> dict:
        """Get detailed collision info including filter status"""
        with self._lock:
            return {
                'collision_detected': self._aircraft_colliding,
                'collision_entity': self._collision_entity,
                'current_collisions': list(self._current_collisions),
                'ignore_ground': self._ignore_ground,
                'ignored_entities': list(self._ignored_entities),
                'ground_entities': list(self._ground_entities)
            }
    
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
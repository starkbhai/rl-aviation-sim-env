#!/usr/bin/env python3

import cv2
import numpy as np
import threading
import time
from typing import Optional, Tuple
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

class AircraftCameraSubscriber:
    """
    Module-friendly camera subscriber that runs in background
    Continuously receives images and provides latest frame on demand
    """
    
    def __init__(self, topic: str = "/aircraft/front_camera", auto_start: bool = True):
        """
        Initialize camera subscriber
        
        Args:
            topic: Camera topic to subscribe to
            auto_start: If True, starts background monitoring automatically
        """
        
        self.topic = topic
        
        # Create Gazebo Transport node
        try:
            self.node = Node()
        except Exception as e:
            print(f"❌ Failed to create Gazebo Transport node: {e}")
            raise
        
        # Thread-safe image storage
        self._lock = threading.Lock()
        self._latest_image_msg = None
        self._latest_image_array = None
        self._last_update_time = None
        self._frame_count = 0
        
        # Subscribe to camera topic
        self._setup_subscriber()
        
        print(f"✅ Aircraft Camera Subscriber initialized on topic: {topic}")
        
    def _setup_subscriber(self):
        """Setup camera topic subscription"""
        
        def image_callback(msg: Image):
            self._image_callback(msg)
        
        if self.node.subscribe(Image, self.topic, image_callback):
            print(f"✅ Subscribed to camera topic: {self.topic}")
        else:
            raise RuntimeError(f"Failed to subscribe to camera topic: {self.topic}")
    
    def _image_callback(self, msg: Image):
        """
        Internal callback - runs automatically in background thread
        Processes and stores the latest image
        """
        
        # Parse image message to numpy array
        numpy_image = self._parse_image_message(msg)
        
        if numpy_image is not None:
            with self._lock:
                self._latest_image_msg = msg
                self._latest_image_array = numpy_image
                self._last_update_time = time.time()
                self._frame_count += 1
                
                # Optional: Print frame rate occasionally
                if self._frame_count % 30 == 0:  # Every 30 frames
                    print(f"📸 Received frame {self._frame_count} ({msg.width}x{msg.height})")
    
    def _parse_image_message(self, msg: Image) -> Optional[np.ndarray]:
        """
        Parse Gazebo image message to numpy array
        (Same logic as original camera_subscriber.py)
        """
        pixel_format = msg.pixel_format_type

        # Fix for data/header mismatch
        expected_3_channel_size = msg.height * msg.width * 3
        if pixel_format == 3 and len(msg.data) == expected_3_channel_size:
            pixel_format = 15  # Override to BGR_INT8

        # Format mapping
        format_map = {
            10: (np.uint8, 3),   # RGB_INT8
            11: (np.uint8, 4),   # RGBA_INT8
            15: (np.uint8, 3),   # BGR_INT8
            12: (np.uint8, 4),   # BGRA_INT8
            2:  (np.uint8, 1),   # L_INT8 (Grayscale)
            3:  (np.uint16, 1),  # L_INT16 (Grayscale)
            20: (np.float32, 1)  # R_FLOAT32 (Depth)
        }

        if pixel_format not in format_map:
            print(f"❌ Unsupported pixel format: {pixel_format}")
            return None

        dtype, channels = format_map[pixel_format]

        try:
            image_array_1d = np.frombuffer(msg.data, dtype=dtype)
            
            if channels > 1:
                image = image_array_1d.reshape(msg.height, msg.width, channels)
            else:
                image = image_array_1d.reshape(msg.height, msg.width)
            
            # Convert to BGR for OpenCV compatibility
            image = self._convert_to_bgr(image, pixel_format, msg)
            
            return image
            
        except ValueError as e:
            print(f"❌ Error reshaping image: {e}")
            return None
    
    def _convert_to_bgr(self, image: np.ndarray, pixel_format: int, msg: Image) -> np.ndarray:
        """Convert image to BGR format for OpenCV compatibility"""
        
        # Check if format was overridden
        expected_3_channel_size = msg.height * msg.width * 3
        is_overridden = (pixel_format == 3 and len(msg.data) == expected_3_channel_size)
        
        if is_overridden:
            return image  # Already BGR
        
        if pixel_format == 10:  # RGB_INT8 -> BGR
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif pixel_format == 11:  # RGBA_INT8 -> BGRA
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
        elif pixel_format in [2, 3, 20]:  # Grayscale or Depth
            if image.dtype == np.float32:  # Normalize depth
                image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        
        return image  # Return as-is for other formats
    
    def get_latest_image(self) -> Optional[np.ndarray]:
        """
        Get the latest received image as numpy array (BGR format)
        
        Returns:
            np.ndarray: Latest image in BGR format, or None if no image received
        """
        with self._lock:
            if self._latest_image_array is not None:
                return self._latest_image_array.copy()  # Return copy for thread safety
            return None
    
    def get_image_info(self) -> Optional[Tuple[int, int, int]]:
        """
        Get image dimensions and channel info
        
        Returns:
            Tuple[int, int, int]: (height, width, channels) or None if no image
        """
        with self._lock:
            if self._latest_image_array is not None:
                if len(self._latest_image_array.shape) == 3:
                    h, w, c = self._latest_image_array.shape
                    return h, w, c
                else:
                    h, w = self._latest_image_array.shape
                    return h, w, 1
            return None
    
    def get_frame_count(self) -> int:
        """Get total number of frames received"""
        with self._lock:
            return self._frame_count
    
    def get_last_update_time(self) -> Optional[float]:
        """Get timestamp of last frame"""
        with self._lock:
            return self._last_update_time
    
    def is_receiving_frames(self, timeout: float = 2.0) -> bool:
        """
        Check if camera is actively receiving frames
        
        Args:
            timeout: Seconds without frames before considering inactive
            
        Returns:
            bool: True if recently received frames, False otherwise
        """
        with self._lock:
            if self._last_update_time is None:
                return False
            return (time.time() - self._last_update_time) < timeout
    
    def get_frame_rate(self, window_seconds: float = 5.0) -> float:
        """
        Estimate current frame rate
        
        Args:
            window_seconds: Time window to calculate rate over
            
        Returns:
            float: Estimated frames per second
        """
        with self._lock:
            if self._last_update_time is None or self._frame_count < 2:
                return 0.0
            
            # Simple estimation based on total frames and elapsed time
            # (In practice, you'd want a more sophisticated rolling average)
            elapsed = time.time() - (self._last_update_time - window_seconds)
            if elapsed > 0:
                return min(self._frame_count / elapsed, self._frame_count / window_seconds)
            return 0.0

# Standalone execution mode for testing
def main():
    """Standalone mode - displays camera feed in window"""
    
    import signal
    import sys
    
    def signal_handler(sig, frame):
        print("\n\nShutting down...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Create camera subscriber
        camera = AircraftCameraSubscriber()
        
        print("\n" + "="*50)
        print("📸 AIRCRAFT CAMERA VIEWER")
        print("="*50)
        print("Press 'q' to quit, 's' to save frame")
        print("="*50 + "\n")
        
        # Display window
        window_name = "Aircraft Camera Feed"
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        
        frame_save_count = 0
        
        while True:
            # Get latest image
            image = camera.get_latest_image()
            
            if image is not None:
                # Add frame info overlay
                info = camera.get_image_info()
                frame_count = camera.get_frame_count()
                is_active = camera.is_receiving_frames()
                
                if info:
                    h, w, c = info
                    status = "🟢 ACTIVE" if is_active else "🔴 STALE"
                    text = f"Frame: {frame_count} | {w}x{h}x{c} | {status}"
                    cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow(window_name, image)
            else:
                # Show waiting message
                waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(waiting_img, "Waiting for camera frames...", (100, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.imshow(window_name, waiting_img)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s') and image is not None:
                # Save current frame
                filename = f"aircraft_frame_{frame_save_count:04d}.jpg"
                cv2.imwrite(filename, image)
                print(f"💾 Saved frame: {filename}")
                frame_save_count += 1
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        cv2.destroyAllWindows()
        print("✅ Camera viewer closed")

if __name__ == "__main__":
    main()
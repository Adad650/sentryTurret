"""
Motion Detection Module
Detects motion in video feed and maps coordinates to percentage values.
"""

import cv2
import time
import sys
from typing import Tuple, Optional

class MotionDetector:
    def __init__(self, camera_index: int = 0, threshold: int = 25, min_area: int = 5000):
        """
        Initialize motion detector
        
        Args:
            camera_index: Camera device index
            threshold: Motion detection threshold (0-255)
            min_area: Minimum contour area to consider as motion
        """
        self.camera_index = camera_index
        self.threshold = threshold
        self.min_area = min_area
        self.cap = None
        self.old_frame = None
        self.frame_width = 0
        self.frame_height = 0
        
    def initialize_camera(self) -> bool:
        """Initialize camera and get frame dimensions"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            # Try to set optimal camera properties
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Read first frame to get dimensions
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to read from camera")
                return False
                
            self.frame_height, self.frame_width = frame.shape[:2]
            print(f"Camera initialized: {self.frame_width}x{self.frame_height}")
            
            # Convert to grayscale for motion detection
            self.old_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return True
            
        except Exception as e:
            print(f"Camera initialization failed: {e}")
            return False
    
    def map_motion(self, x: int, y: int) -> Tuple[float, float]:
        """
        Map motion coordinates to percentage values
        
        Args:
            x: X coordinate of motion center
            y: Y coordinate of motion center
            
        Returns:
            Tuple of (x_percentage, y_percentage)
        """
        # Map coordinates to percentage (-100 to 100)
        x_percent = (-x / self.frame_width) * 100
        y_percent = (-y / self.frame_height) * 100
        
        # Clamp to valid range
        x_percent = max(-100, min(100, x_percent))
        y_percent = max(-100, min(100, y_percent))
        
        return x_percent, y_percent
    
    def detect_motion(self) -> Optional[Tuple[int, int]]:
        """
        Detect motion in current frame
        
        Returns:
            Tuple of (center_x, center_y) if motion detected, None otherwise
        """
        if self.cap is None or self.old_frame is None:
            return None
            
        ret, frame = self.cap.read()
        if not ret:
            return None
            
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Calculate frame difference
        diff = cv2.absdiff(self.old_frame, gray)
        
        # Apply threshold
        _, thresh = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Update old frame
        self.old_frame = gray
        
        # Find largest contour above minimum area
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > self.min_area:
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x = x + w // 2
                center_y = y + h // 2
                return center_x, center_y
                
        return None
    
    def process_frame(self, show_display: bool = True) -> Optional[Tuple[float, float]]:
        """
        Process current frame and detect motion
        
        Args:
            show_display: Whether to show the video display
            
        Returns:
            Tuple of (x_percent, y_percent) if motion detected, None otherwise
        """
        if self.cap is None:
            return None
            
        ret, frame = self.cap.read()
        if not ret:
            return None
            
        # Detect motion
        motion_center = self.detect_motion()
        
        if motion_center:
            center_x, center_y = motion_center
            
            # Map to percentage
            x_percent, y_percent = self.map_motion(center_x, center_y)
            
            # Draw motion rectangle
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            diff = cv2.absdiff(self.old_frame, gray)
            _, thresh = cv2.threshold(diff, self.threshold, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > self.min_area:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Print motion coordinates
            print(f"Motion detected: x = {x_percent:.1f}%, y = {y_percent:.1f}%")
            
            if show_display:
                # Resize frame for display (maintain aspect ratio)
                display_width = 1920
                display_height = int(display_width * self.frame_height / self.frame_width)
                resized_frame = cv2.resize(frame, (display_width, display_height))
                
                # Mirror horizontally
                mirrored = cv2.flip(resized_frame, 1)
                cv2.imshow("Motion Detection", mirrored)
            
            return x_percent, y_percent
            
        return None
    
    def run(self, show_display: bool = True):
        """Main loop for motion detection"""
        if not self.initialize_camera():
            print("Failed to initialize camera")
            return
            
        print("Motion detection started. Press 'q' to quit.")
        
        try:
            while True:
                result = self.process_frame(show_display)
                
                # Handle key presses
                if show_display:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Error during execution: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    """Main function to run motion detection"""
    detector = MotionDetector(
        camera_index=0,
        threshold=25,
        min_area=5000
    )
    detector.run(show_display=True)

if __name__ == "__main__":
    main()
    
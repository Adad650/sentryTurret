# motion_servo_track.py - Raspberry Pi 3 Version
import cv2
import time
import RPi.GPIO as GPIO
from collections import deque
import sys
import os

def cleanup_and_exit(cap=None, pwm_pan=None, pwm_tilt=None):
    """Clean up resources and exit gracefully"""
    if pwm_pan:
        pwm_pan.stop()
    if pwm_tilt:
        pwm_tilt.stop()
    GPIO.cleanup()
    if cap:
        cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)

def test_camera_access():
    """Test different camera indices and settings for Pi"""
    print("Testing camera access on Raspberry Pi...")
    
    # Check for video devices
    if os.path.exists("/dev/video0"):
        print("✓ /dev/video0 found")
    else:
        print("✗ /dev/video0 not found")
    
    # Test different camera indices
    for camera_index in [0, 1]:
        print(f"Trying camera index {camera_index}...")
        cap = cv2.VideoCapture(camera_index)
        
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                height, width = frame.shape[:2]
                print(f"✓ Camera {camera_index} found: {width}x{height}")
                cap.release()
                return camera_index
            else:
                print(f"✗ Camera {camera_index} opened but can't read frames")
                cap.release()
        else:
            print(f"✗ Camera {camera_index} not accessible")
    
    # Try with V4L2 backend specifically for Pi
    print("Trying V4L2 backend...")
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            height, width = frame.shape[:2]
            print(f"✓ Camera found with V4L2: {width}x{height}")
            cap.release()
            return 0
        cap.release()
    
    return None

# ========= Servo setup (BCM pins) =========
try:
    GPIO.setmode(GPIO.BCM)  # Using BCM numbering
    PAN_PIN  = 21   # left/right  servo (GPIO21)
    TILT_PIN = 22   # up/down     servo (GPIO22)

    GPIO.setup(PAN_PIN,  GPIO.OUT)
    GPIO.setup(TILT_PIN, GPIO.OUT)

    pwm_pan  = GPIO.PWM(PAN_PIN,  50)   # 50 Hz
    pwm_tilt = GPIO.PWM(TILT_PIN, 50)

    def set_angle(pwm, deg):
        deg = max(0.0, min(180.0, float(deg)))
        duty_cycle = (deg / 18.0) + 2.0
        pwm.ChangeDutyCycle(duty_cycle)

    # start centered
    pwm_pan.start(0)
    pwm_tilt.start(0)
    time.sleep(0.1)
    pan_deg  = 90.0
    tilt_deg = 90.0
    set_angle(pwm_pan,  pan_deg)
    set_angle(pwm_tilt, tilt_deg)
    print("Servos initialized and centered")

except Exception as e:
    print(f"GPIO setup failed: {e}")
    print("Make sure you're running with sudo: sudo python src/test.py")
    cleanup_and_exit()

# ========= Camera setup for Pi =========
print("Initializing camera on Raspberry Pi...")

# Test camera access first
camera_index = test_camera_access()
if camera_index is None:
    print("\n❌ No camera detected on Raspberry Pi!")
    print("\nTroubleshooting steps for Pi:")
    print("1. Check camera connection (USB or Pi Camera)")
    print("2. Run: ls /dev/video*")
    print("3. For Pi Camera: sudo raspi-config -> Interface Options -> Camera")
    print("4. For USB camera: lsusb")
    print("5. Check permissions: sudo usermod -a -G video $USER")
    print("6. Reboot: sudo reboot")
    print("7. Try running with sudo: sudo python src/test.py")
    cleanup_and_exit(cap=None, pwm_pan=pwm_pan, pwm_tilt=pwm_tilt)

try:
    print(f"Opening camera {camera_index} on Pi...")
    
    # Try V4L2 backend first (better for Pi)
    cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
    if not cap.isOpened():
        # Fallback to default backend
        cap = cv2.VideoCapture(camera_index)
    
    # Set Pi-optimized camera settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # Try to read a test frame
    ok, frame = cap.read()
    if not ok:
        print("❌ Camera opened but can't read frames")
        print("Trying alternative settings...")
        
        # Try different resolutions for Pi
        resolutions = [(320, 240), (640, 480), (1280, 720)]
        for width, height in resolutions:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ok, frame = cap.read()
            if ok:
                print(f"✓ Camera working at {width}x{height}")
                break
        
        if not ok:
            print("❌ Camera still not working with any resolution")
            cleanup_and_exit(cap, pwm_pan, pwm_tilt)
    
    H, W = frame.shape[:2]
    print(f"✓ Camera initialized on Pi: {W}x{H}")

except Exception as e:
    print(f"❌ Camera setup failed: {e}")
    cleanup_and_exit(cap=None, pwm_pan=pwm_pan, pwm_tilt=pwm_tilt)

# ========= Motion detection setup =========
# Simple frame differencing for motion detection
previous_gray = None
MIN_CONTOUR_AREA = 2000  # Lower threshold for Pi
MOTION_THRESHOLD = 25    # Threshold for motion detection

# ========= Servo control parameters =========
PAN_SPEED = 1.5    # Degrees per frame (slower for Pi)
TILT_SPEED = 1.5   # Degrees per frame
DEADBAND = 40      # Pixels from center to ignore (larger for Pi)

print("Motion tracking started on Raspberry Pi. Press 'q' to quit, 'c' to center servos.")

try:
    while True:
        ok, frame = cap.read()
        if not ok: 
            print("Failed to read frame")
            break

        # Convert to grayscale and blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # Motion detection using frame differencing
        if previous_gray is not None:
            # Calculate frame difference
            frame_delta = cv2.absdiff(previous_gray, gray)
            _, thresh = cv2.threshold(frame_delta, MOTION_THRESHOLD, 255, cv2.THRESH_BINARY)
            thresh = cv2.dilate(thresh, None, iterations=2)
            
            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            motion_detected = False
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                    motion_detected = True
                    
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Draw rectangle around motion
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (0, 255, 255), -1)
                    
                    # Calculate distance from center
                    center_error_x = center_x - (W // 2)
                    center_error_y = center_y - (H // 2)
                    
                    # Move servos based on motion position
                    if abs(center_error_x) > DEADBAND:
                        if center_error_x > 0:  # Motion is right of center
                            pan_deg -= PAN_SPEED
                        else:  # Motion is left of center
                            pan_deg += PAN_SPEED
                        
                        # Clamp pan angle
                        pan_deg = max(0.0, min(180.0, pan_deg))
                        set_angle(pwm_pan, pan_deg)
                        print(f"Pan servo moved to {pan_deg:.1f}°")
                    
                    if abs(center_error_y) > DEADBAND:
                        if center_error_y > 0:  # Motion is below center
                            tilt_deg += TILT_SPEED
                        else:  # Motion is above center
                            tilt_deg -= TILT_SPEED
                        
                        # Clamp tilt angle
                        tilt_deg = max(45.0, min(135.0, tilt_deg))
                        set_angle(pwm_tilt, tilt_deg)
                        print(f"Tilt servo moved to {tilt_deg:.1f}°")

        # Update previous frame
        previous_gray = gray

        # Draw center crosshair
        cv2.line(frame, (W//2, 0), (W//2, H), (255, 0, 0), 1)
        cv2.line(frame, (0, H//2), (W, H//2), (255, 0, 0), 1)
        cv2.circle(frame, (W//2, H//2), 10, (255, 0, 0), 2)

        # Display frame (smaller for Pi performance)
        display_frame = cv2.resize(frame, (640, 480))
        mirrored_frame = cv2.flip(display_frame, 1)
        cv2.imshow("Motion Tracking (q=quit, c=center)", mirrored_frame)
        
        # Handle key presses
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): 
            break
        if k == ord('c'):
            pan_deg, tilt_deg = 90.0, 90.0
            set_angle(pwm_pan, pan_deg)
            set_angle(pwm_tilt, tilt_deg)
            print("Servos centered")

except KeyboardInterrupt:
    print("\nInterrupted by user")
except Exception as e:
    print(f"Error during execution: {e}")
finally:
    # cleanup
    cleanup_and_exit(cap, pwm_pan, pwm_tilt)


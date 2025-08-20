# motion_servo_track.py
import cv2
import time
import RPi.GPIO as GPIO
from collections import deque
import sys

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

# ========= Servo setup (BOARD pins) =========
try:
    GPIO.setmode(GPIO.BOARD)
    PAN_PIN  = 11   # left/right  servo
    TILT_PIN = 13   # up/down     servo  (optional; disable if you only want X)

    GPIO.setup(PAN_PIN,  GPIO.OUT)
    GPIO.setup(TILT_PIN, GPIO.OUT)

    pwm_pan  = GPIO.PWM(PAN_PIN,  50)   # 50 Hz
    pwm_tilt = GPIO.PWM(TILT_PIN, 50)

    def set_angle(pwm, deg):
        deg = max(0.0, min(180.0, float(deg)))
        pwm.ChangeDutyCycle(2.5 + deg/18.0)

    # start centered
    pwm_pan.start(0)
    pwm_tilt.start(0)
    time.sleep(0.1)
    pan_deg  = 90.0
    tilt_deg = 90.0
    set_angle(pwm_pan,  pan_deg)
    set_angle(pwm_tilt, tilt_deg)

except Exception as e:
    print(f"GPIO setup failed: {e}")
    cleanup_and_exit()

# ========= Camera (fast settings) =========
try:
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    ok, frame = cap.read()
    if not ok:
        print("No camera detected.")
        cleanup_and_exit(cap, pwm_pan, pwm_tilt)
    
    H, W = frame.shape[:2]
    print(f"Camera initialized: {W}x{H}")

except Exception as e:
    print(f"Camera setup failed: {e}")
    cleanup_and_exit(cap=None, pwm_pan=pwm_pan, pwm_tilt=pwm_tilt)

# ========= Motion model & filters =========
fg = cv2.createBackgroundSubtractorMOG2(history=400, varThreshold=32, detectShadows=False)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
AREA_MIN   = 3500     # ignore small blobs
STABLE_N   = 3        # need N consecutive frames
centers_x  = deque(maxlen=STABLE_N)
centers_y  = deque(maxlen=STABLE_N)

# ========= Control (map px -> degrees using FOV) =========
CAM_H_FOV = 60.0   # <-- set to your cam's real FOV
CAM_V_FOV = 35.0

PAN_INVERT  =  1.0  # flip to -1.0 if moving opposite
TILT_INVERT = -1.0  # typically up is negative image Y

Kp_pan, Kd_pan   = 0.55, 0.18
Kp_tilt, Kd_tilt = 0.55, 0.18
DEADBAND_DEG = 1.2
STEP_MAX_DEG = 4.0
MIN_STEP_DEG = 0.2
prev_err_pan = 0.0
prev_err_tilt= 0.0
last_update  = 0.0

def px_to_deg(err_px, size_px, fov_deg):
    # err_px: pixels from center (left/up negative, right/down positive)
    return (err_px / (size_px/2.0)) * (fov_deg/2.0)

print("Motion tracking started. Press 'q' to quit, 'c' to center servos.")

try:
    while True:
        ok, frame = cap.read()
        if not ok: 
            print("Failed to read frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)

        mask = fg.apply(gray, learningRate=0.001)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        have_blob = False
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            if cv2.contourArea(c) > AREA_MIN:
                x,y,w,h = cv2.boundingRect(c)
                cx = x + w//2
                cy = y + h//2
                centers_x.append(cx)
                centers_y.append(cy)
                have_blob = (len(centers_x) == STABLE_N)
                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            else:
                centers_x.clear()
                centers_y.clear()
        else:
            centers_x.clear()
            centers_y.clear()

        # Only move when we've seen consistent motion
        if have_blob:
            cx = int(sum(centers_x)/len(centers_x))
            cy = int(sum(centers_y)/len(centers_y))

            # pixel errors from image center
            err_x_px = (cx - W/2.0)
            err_y_px = (cy - H/2.0)

            # convert to degrees using camera FOV
            err_pan_deg  = px_to_deg(err_x_px, W, CAM_H_FOV) * PAN_INVERT
            err_tilt_deg = px_to_deg(err_y_px, H, CAM_V_FOV) * TILT_INVERT

            # deadband
            do_pan  = abs(err_pan_deg)  > DEADBAND_DEG
            do_tilt = abs(err_tilt_deg) > DEADBAND_DEG

            # PD control + rate limit
            now = time.time()
            if now - last_update >= 0.02:  # ~50 Hz servo updates
                if do_pan:
                    d_pan   = Kp_pan*err_pan_deg + Kd_pan*(err_pan_deg - prev_err_pan)
                    d_pan   = max(-STEP_MAX_DEG, min(STEP_MAX_DEG, d_pan))
                    if abs(d_pan) >= MIN_STEP_DEG:
                        pan_deg = max(0.0, min(180.0, pan_deg + d_pan))
                        set_angle(pwm_pan, pan_deg)
                    prev_err_pan = err_pan_deg
                else:
                    prev_err_pan = 0.0

                if do_tilt:
                    d_tilt  = Kp_tilt*err_tilt_deg + Kd_tilt*(err_tilt_deg - prev_err_tilt)
                    d_tilt  = max(-STEP_MAX_DEG, min(STEP_MAX_DEG, d_tilt))
                    if abs(d_tilt) >= MIN_STEP_DEG:
                        tilt_deg = max(0.0, min(180.0, tilt_deg + d_tilt))
                        set_angle(pwm_tilt, tilt_deg)
                    prev_err_tilt = err_tilt_deg
                else:
                    prev_err_tilt = 0.0

                last_update = now

            # visuals
            cv2.circle(frame, (cx,cy), 4, (0,255,255), -1)

        # guides
        cv2.line(frame, (W//2, 0), (W//2, H), (255,0,0), 1)
        cv2.line(frame, (0, H//2), (W, H//2), (255,0,0), 1)

        cv2.imshow("Motion Track (q=quit, c=center)", frame)
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

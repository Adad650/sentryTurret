# motionServoTrackOneServo_GPIO.py
# Raspberry Pi + MG90 (pan-only left/right) motion tracking using RPi.GPIO + OpenCV.
# Naming style: likeThis (camelCase). No pigpio/daemon required.

import cv2
import time
import sys
import os
import RPi.GPIO as GPIO

# ==================== Config ====================
panGpio = 21                    # BCM pin for pan servo signal (GPIO21 / pin 40 on 40-pin header)
frameW, frameH = 640, 480
fps = 60

# Motion detection thresholds (tuned for Pi)
minContourArea = 2000
motionThreshold = 25
deadbandPx = 40                 # ignore near-center jitter

# Pan dynamics and limits
panSpeedDeg = 1.5               # degrees per update
panMinDeg = 20.0                # keep off hard end-stops
panMaxDeg = 160.0
panHomeDeg = 90.0
invertPan = False               # set True if motion direction feels reversed

# MG90 calibration (typical) for 50 Hz PWM duty cycle
# 1ms..2ms pulse → 5%..10% duty at 50 Hz (period = 20ms)
servoMinDuty = 5.0              # ≈ 0°
servoMaxDuty = 10.0             # ≈ 180°
servoHomeDuty = 7.5             # ≈ 90°
pwmHz = 50

# Smoothing for error → smoother motion (0..1)
smoothing = 0.4

# Camera backends to try on Pi
cameraBackends = [cv2.CAP_V4L2, 0]
# =================================================

def angleToDuty(deg: float) -> float:
    """Map angle [0..180] to duty cycle [%] for 50 Hz PWM."""
    d = max(0.0, min(180.0, float(deg)))
    print(servoMinDuty + (servoMaxDuty - servoMinDuty) * (d / 180.0))
    return servoMinDuty + (servoMaxDuty - servoMinDuty) * (d / 180.0)

def safeSetAngle(pwm, deg: float) -> float:
    """Clamp angle to safe limits and set duty on the PWM channel."""
    d = max(panMinDeg, min(panMaxDeg, float(deg)))
    pwm.ChangeDutyCycle(angleToDuty(d))
    return d

def testCameraIndex():
    """Try /dev/video* indices and return a working index or None."""
    if os.path.exists("/dev/video0"):
        print("✓ /dev/video0 present")
    else:
        print("✗ /dev/video0 not found")
    for idx in (0, 1):
        for backend in cameraBackends:
            cap = cv2.VideoCapture(idx, backend) if backend else cv2.VideoCapture(idx)
            if not cap.isOpened():
                cap.release()
                continue
            ok, frame = cap.read()
            cap.release()
            if ok:
                return idx
    return None

def doCleanup(cap, pwm):
    try:
        if cap:
            cap.release()
    except Exception:
        pass
    try:
        if pwm:
            pwm.ChangeDutyCycle(0)
            pwm.stop()
    except Exception:
        pass
    try:
        GPIO.cleanup()
    except Exception:
        pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass

def main():
    print("=== Raspberry Pi MG90 Pan Tracker (one servo) — RPi.GPIO ===")

    # --- GPIO init ---
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(panGpio, GPIO.OUT)
    pwm = GPIO.PWM(panGpio, pwmHz)
    pwm.start(0)
    time.sleep(0.1)

    currentPanDeg = panHomeDeg
    currentPanDeg = safeSetAngle(pwm, currentPanDeg)
    time.sleep(0.2)

    # --- Camera init ---
    camIndex = testCameraIndex()
    if camIndex is None:
        print("No camera found.\nCheck: ls /dev/video*, cabling, permissions (add user to 'video'), raspi-config.")
        doCleanup(None, pwm)
        sys.exit(1)

    print(f"Opening camera index {camIndex} …")
    cap = None
    for backend in cameraBackends:
        cap = cv2.VideoCapture(camIndex, backend) if backend else cv2.VideoCapture(camIndex)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frameW)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frameH)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, testFrame = cap.read()
        if ok:
            break
        cap.release()
        cap = None

    if cap is None:
        print("❌ Camera could not be opened with any backend.")
        doCleanup(None, pwm)
        sys.exit(1)

    ok, frame = cap.read()
    if not ok:
        print("❌ Camera opened but cannot read frames.")
        doCleanup(cap, pwm)
        sys.exit(1)

    H, W = frame.shape[:2]
    print(f"✓ Camera up: {W}x{H} @ ~{fps} FPS")

    previousGray = None
    filteredErrX = 0.0

    print("Motion tracking: 'q' quit, 'c' center pan.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Frame read failed")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if previousGray is not None:
                delta = cv2.absdiff(previousGray, gray)
                _, thresh = cv2.threshold(delta, motionThreshold, 255, cv2.THRESH_BINARY)
                thresh = cv2.dilate(thresh, None, iterations=2)

                contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) > minContourArea:
                        x, y, w, h = cv2.boundingRect(c)
                        cx = x + w // 2
                        cy = y + h // 2

                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

                        errX = cx - (W // 2)

                        # Deadband to avoid jitter
                        if abs(errX) > deadbandPx:
                            # Low-pass filter
                            filteredErrX = (smoothing * filteredErrX) + ((1.0 - smoothing) * errX)

                            # Direction (invert if needed)
                            if (filteredErrX > 0) ^ invertPan:
                                currentPanDeg -= panSpeedDeg
                            else:
                                currentPanDeg += panSpeedDeg

                            currentPanDeg = safeSetAngle(pwm, currentPanDeg)
                            # print(f"Pan: {currentPanDeg:.1f}°")

            previousGray = gray

            # Crosshair
            cv2.line(frame, (W // 2, 0), (W // 2, H), (255, 0, 0), 1)
            cv2.line(frame, (0, H // 2), (W, H // 2), (255, 0, 0), 1)
            cv2.circle(frame, (W // 2, H // 2), 10, (255, 0, 0), 2)

            display = cv2.resize(frame, (640, 480))
            display = cv2.flip(display, 1)
            cv2.imshow("Pan Motion Tracking (q=quit, c=center)", display)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                break
            if k == ord('c'):
                currentPanDeg = safeSetAngle(pwm, panHomeDeg)
                print("Centered pan")

    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        doCleanup(cap, pwm)

if __name__ == "__main__":
    main()

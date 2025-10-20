#!/usr/bin/env python3
# servo_usb_flask_55deg.py
# Implements your angle-finding + display code, adds USB cam robustness + servo (0..55°) + startup safety.

import os
import sys
import time
import threading
import cv2
import RPi.GPIO as GPIO
from flask import Flask, jsonify, Response

# ------------------- SETTINGS -------------------
# Servo
SERVO_PIN = 17
PWM_HZ = 50
MIN_US, MAX_US = 500, 2500   # typical analog servo pulse range
SERVO_MIN_DEG = 0.0          # hard clamp range
SERVO_MAX_DEG = 55.0
DEADBAND_DEG = 0.4           # ignore tiny changes

# Camera
FOV_DEG = 55.0               # your camera horizontal FOV
CAM_INDEXES = [0, 1, 2, 3]
CAP_PROFILES = [
    (1280, 720, 30),
    (1280, 720, 15),
    (640, 480, 30),
    (640, 480, 15),
]

# Behavior
STARTUP_HOLD_SEC = 10.0
MIN_CONTOUR_AREA = 5000
SETTLE_SEC = 0.03            # short pause after PWM update

# ------------------- GLOBAL STATE -------------------
app = Flask(__name__)
currentAngle = 0.0         # what we're commanding to the servo (0..55)
targetFound = False
lastFrame = None
angleLock = threading.Lock()
frameLock = threading.Lock()

# ------------------- UTILITIES -------------------
def has_display() -> bool:
    return bool(os.environ.get("DISPLAY"))

def angle_to_duty(angle_deg: float) -> float:
    a = max(0.0, min(180.0, float(angle_deg)))
    pulse_us = MIN_US + (a / 180.0) * (MAX_US - MIN_US)
    return (pulse_us / 20000.0) * 100.0  # 20 ms period for 50Hz

def servo_go(pwm, angle_deg: float) -> None:
    pwm.ChangeDutyCycle(angle_to_duty(angle_deg))
    time.sleep(SETTLE_SEC)

def open_usb_camera():
    """Try multiple indices + profiles until one works. Returns (cap, width, height, fps)."""
    for idx in CAM_INDEXES:
        for (w, h, f) in CAP_PROFILES:
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS,          f)
            ok, frame = cap.read()
            if ok and frame is not None:
                aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                af = int(cap.get(cv2.CAP_PROP_FPS))
                print(f"[INFO] Using USB camera index {idx} at {aw}x{ah}@{af}")
                return cap, aw, ah, max(af, 5)
            cap.release()
    raise RuntimeError("No working USB camera on /dev/video0..3. Try another port or lower profiles.")

# ------------------- FLASK API THREAD (YOUR CODE) -------------------
def apiThread():
    @app.get("/api/angle")
    def apiAngle():
        with angleLock:
            return jsonify({"angle": currentAngle, "seen": targetFound})

    @app.get("/stream")
    def stream():
        def gen():
            while True:
                with frameLock:
                    frame = None if lastFrame is None else lastFrame.copy()
                if frame is None:
                    time.sleep(0.01)
                    continue
                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    continue
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + jpg.tobytes()
                    + b"\r\n"
                )
        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    # Bind to all interfaces so you can view from another device
    app.run(host="0.0.0.0", port=17464, debug=False, threaded=True)

# Start the API server
threading.Thread(target=apiThread, daemon=True).start()

# ------------------- SERVO INIT -------------------
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, PWM_HZ)
pwm.start(0)

try:
    # Safety: start at 0° and hold for 10s
    print(f"[INFO] Moving servo to 0° and holding for {int(STARTUP_HOLD_SEC)}s…")
    servo_go(pwm, 0.0)
    with angleLock:
        currentAngle = 0.0
        targetFound = False
    time.sleep(STARTUP_HOLD_SEC)

    # ------------------- CAMERA INIT -------------------
    cap, width, height, fps = open_usb_camera()

    # Prime first gray frame
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Camera failed to provide an initial frame.")
    oldGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Headless-safe: only show a window if a display exists
    show_window = has_display()
    if not show_window:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        print("[MODE] Headless detected: use /stream in your browser for preview.")

    print("[RUNNING] Motion tracking + servo (0..55°). Press 'q' in window to quit (if shown).")

    # ------------------- MAIN LOOP (YOUR ANGLE/DISPLAY LOGIC) -------------------
    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(oldGray, gray)
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        oldGray = gray

        angle = 90.0   # default display value (won't be written to servo unless seen)
        seen = False

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                # map 0..width -> 0..55 (your FOV)
                angle = (cx / float(width)) * FOV_DEG
                seen = True

        # ----- Update shared angle + move servo (clamped 0..55 with deadband) -----
        with angleLock:
            if seen:
                desired = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, angle))
                if abs(desired - currentAngle) >= DEADBAND_DEG:
                    currentAngle = desired
                    servo_go(pwm, currentAngle)
            targetFound = seen
            displayAngle = currentAngle  # what we overlay

        # Share frame for /stream and (maybe) local window
        with frameLock:
            lastFrame = frame.copy()

        cv2.putText(
            frame,
            f"Angle: {displayAngle:.1f}°  Seen: {targetFound}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 220, 0),
            2,
        )

        if show_window:
            cv2.imshow("Motion (0..55°)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

finally:
    print("[CLEANUP] Releasing resources…")
    try:
        pwm.ChangeDutyCycle(0)
        time.sleep(0.3)
        pwm.stop()
    except Exception:
        pass
    GPIO.cleanup()
    try:
        cap.release()
        if has_display():
            cv2.destroyAllWindows()
    except Exception:
        pass
    print("[DONE] Clean exit.")

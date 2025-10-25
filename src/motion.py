import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import RPi.GPIO as GPIO
from flask import Flask, jsonify, Response

SERVO_PIN = 17
PWM_HZ = 50
MIN_US, MAX_US = 500, 2500
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 55.0
DEADBAND_DEG = 0.4
MIN_CONTOUR_AREA = 5000
SETTLE_SEC = 0.5
STARTUP_HOLD_SEC = 10.0
DEFAULT_CAMERA_FOV_DEG = 55.0
CAMERA_FOV_ENV = "CAMERA_FOV_DEG"

app = Flask(__name__)
currentAngle = SERVO_MIN_DEG
targetFound = False
lastFrame = None
angleLock = threading.Lock()
frameLock = threading.Lock()
duty_release_time = 0.0
duty_active = False
cameraFovDeg = DEFAULT_CAMERA_FOV_DEG


def _parse_fov(value, source: str) -> Optional[float]:
    try:
        fov = float(value)
    except (TypeError, ValueError):
        print(f"[FOV] Ignoring invalid value from {source!r}: {value!r}")
        return None
    if not 0.0 < fov < 180.0:
        print(f"[FOV] Ignoring out-of-range FOV from {source!r}: {fov!r}")
        return None
    return fov


def _load_fov_from_file(path: Path) -> Optional[float]:
    try:
        if path.suffix.lower() == ".json":
            data = json.loads(path.read_text())
            for key in ("horizontal_fov_deg", "h_fov_deg", "fov_deg", "fov"):
                if key in data:
                    fov = _parse_fov(data[key], f"{path}:{key}")
                    if fov is not None:
                        print(f"[FOV] Using {fov:.2f} deg from {path}.")
                        return fov
        else:
            text = path.read_text().strip()
            if text:
                fov = _parse_fov(text, str(path))
                if fov is not None:
                    print(f"[FOV] Using {fov:.2f} deg from {path}.")
                    return fov
    except Exception as exc:
        print(f"[FOV] Failed reading {path}: {exc}")
    return None


def detect_camera_fov(cap, frame_width: int) -> float:
    env_val = os.getenv(CAMERA_FOV_ENV)
    if env_val:
        fov = _parse_fov(env_val, f"env:{CAMERA_FOV_ENV}")
        if fov is not None:
            print(f"[FOV] Using {fov:.2f} deg from environment variable {CAMERA_FOV_ENV}.")
            return fov

    base_dir = Path(__file__).resolve().parent
    for candidate in (
        base_dir / "camera_fov.json",
        base_dir / "camera_fov.txt",
        base_dir.parent / "camera_fov.json",
        base_dir.parent / "camera_fov.txt",
    ):
        if candidate.exists():
            fov = _load_fov_from_file(candidate)
            if fov is not None:
                return fov

    width = float(frame_width)
    if width > 0.0:
        fx_candidates = []
        for attr in (
            "CAP_PROP_OBSENSOR_INTRINSIC_FX",
            "CAP_PROP_INTELPERC_DEPTH_FOCAL_LENGTH_HORZ",
            "CAP_PROP_OPENNI_FOCAL_LENGTH",
        ):
            prop = getattr(cv2, attr, None)
            if prop is not None:
                fx_candidates.append(prop)

        for prop in fx_candidates:
            fx = cap.get(prop)
            if fx and fx > 0.0:
                fov = 2.0 * math.degrees(math.atan(width / (2.0 * fx)))
                if 1.0 <= fov < 179.0:
                    print(f"[FOV] Using {fov:.2f} deg from camera intrinsics (prop {prop}).")
                    return fov

    print(
        f"[FOV] Falling back to default {DEFAULT_CAMERA_FOV_DEG:.2f} deg. "
        f"Set ${CAMERA_FOV_ENV} or camera_fov.json to override."
    )
    return DEFAULT_CAMERA_FOV_DEG


def angle_to_duty(angle_deg: float) -> float:
    clamped = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, float(angle_deg)))
    span_deg = SERVO_MAX_DEG - SERVO_MIN_DEG
    if span_deg <= 0.0:
        pulse_us = MIN_US
    else:
        normalized = (clamped - SERVO_MIN_DEG) / span_deg
        servo_span_us = (span_deg / 180.0) * (MAX_US - MIN_US)
        pulse_us = MIN_US + normalized * servo_span_us
    return (pulse_us / 20000.0) * 100.0


def servo_go(pwm, angle_deg: float) -> None:
    duty = angle_to_duty(angle_deg)
    pwm.ChangeDutyCycle(duty)
    global duty_release_time, duty_active
    duty_release_time = time.monotonic() + SETTLE_SEC
    duty_active = True



def servo_update(pwm, angle, seen):
    global currentAngle, targetFound, duty_release_time, duty_active
    now = time.monotonic()

    with angleLock:
        if duty_active and now >= duty_release_time:
            pwm.ChangeDutyCycle(0.0)
            duty_active = False

        if seen and angle is not None:
            desired = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, angle))
            mirrored = SERVO_MIN_DEG + (SERVO_MAX_DEG - desired)
            if abs(mirrored - currentAngle) >= DEADBAND_DEG:
                currentAngle = mirrored
                servo_go(pwm, currentAngle)
        targetFound = bool(seen)
        display_angle = currentAngle

    return display_angle, targetFound


def init_servo():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, PWM_HZ)
    pwm.start(0)

    servo_go(pwm, SERVO_MIN_DEG)
    with angleLock:
        global currentAngle, targetFound
        currentAngle = SERVO_MIN_DEG
        targetFound = False
    time.sleep(STARTUP_HOLD_SEC)
    return pwm


def cleanup(pwm, cap):
    if pwm is not None:
        try:
            pwm.ChangeDutyCycle(0)
            time.sleep(0.3)
            pwm.stop()
        except Exception:
            pass
        try:
            GPIO.cleanup()
        except Exception:
            pass

    if cap is not None:
        try:
            cap.release()
        except Exception:
            pass

    try:
        cv2.destroyAllWindows()
    except Exception:
        pass

def apiThread():
    @app.get("/api/angle")
    def apiAngle():
        with angleLock:
            return jsonify({"angle": currentAngle, "seen": targetFound, "fov": cameraFovDeg})

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

    app.run(host="0.0.0.0", port=17464, debug=False, threaded=True)

threading.Thread(target=apiThread, daemon=True).start()

pwm = None
cap = None

try:
    pwm = init_servo()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 30)
    ret, frame = cap.read()
    if not ret or frame is None:
        raise RuntimeError("Camera failed to provide an initial frame.")
    oldGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detectedFov = detect_camera_fov(cap, frame.shape[1])
    with angleLock:
        cameraFovDeg = detectedFov
    print(f"[FOV] Active horizontal field of view: {cameraFovDeg:.2f} deg")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(oldGray, gray)
        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        oldGray = gray

        angle = None
        seen = False
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                angle = (cx / frame.shape[1]) * cameraFovDeg
                seen = True

        displayAngle, seenFlag = servo_update(pwm, angle, seen)

        with frameLock:
            lastFrame = frame.copy()

        cv2.putText(
            frame,
            f"Angle: {displayAngle:.1f} deg  Seen: {seenFlag}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 220, 0),
            2,
        )
        cv2.putText(
            frame,
            f"Camera FOV: {cameraFovDeg:.1f} deg",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 200, 200),
            1,
        )
        cv2.imshow("Motion", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
except KeyboardInterrupt:
    pass
finally:
    cleanup(pwm, cap)

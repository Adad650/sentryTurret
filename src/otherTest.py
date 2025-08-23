# visionAngleServer.py
# Shows a live OpenCV window AND hosts a Flask API that returns the calculated angle.
# Also provides an MJPEG stream at /stream.
# Angle rule: 90° is center. Horizontal error maps linearly via FOV=55°.

import cv2
import time
import math
import threading
from flask import Flask, jsonify, Response, send_file

# ====== Config ======
cameraIndex = 0          # 0 for default cam; use 1 if you have multiple
frameW, frameH = 640, 480
fpsTarget = 30
fovDeg = 55.0            # camera horizontal field-of-view
centerAngleDeg = 90.0    # 90° = camera center
minContourArea = 2000
motionThreshold = 25
deadbandPx = 20          # pixels around center ignored for tiny jitter

mjpegQuality = 80
snapshotPath = "latest.jpg"

# ====== Shared state ======
app = Flask(__name__)
cap = None
capLock = threading.Lock()

lastFrame = None
lastFrameLock = threading.Lock()

currentAngle = centerAngleDeg
angleLock = threading.Lock()

running = True

# ====== Core math ======
def clampAngle(deg: float) -> float:
    return max(0.0, min(180.0, float(deg)))

def xOffsetToAngle(xOffsetPx: float, imgW: int) -> float:
    """
    xOffsetPx: + to the right, - to the left, relative to image center.
    Map to degrees using FOV. Center -> 90°, right edge ~ +FOV/2, left edge ~ -FOV/2.
    """
    halfW = imgW / 2.0
    halfFov = fovDeg / 2.0
    # normalized offset: +1 at right edge, -1 at left edge
    norm = xOffsetPx / halfW
    return clampAngle(centerAngleDeg + norm * halfFov)

# ====== Camera + vision loop ======
def cameraLoop():
    global cap, lastFrame, currentAngle, running

    # open camera
    with capLock:
        cam = cv2.VideoCapture(cameraIndex, cv2.CAP_V4L2)
        if not cam.isOpened():
            cam = cv2.VideoCapture(cameraIndex)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, frameW)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, frameH)
        cam.set(cv2.CAP_PROP_FPS, fpsTarget)
        cap = cam

    prevGray = None
    frames = 0
    t0 = time.time()

    while running:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        angle = centerAngleDeg

        if prevGray is not None:
            delta = cv2.absdiff(prevGray, gray)
            _, thresh = cv2.threshold(delta, motionThreshold, 255, cv2.THRESH_BINARY)
            thresh = cv2.dilate(thresh, None, iterations=2)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > minContourArea:
                    x, y, w, h = cv2.boundingRect(c)
                    cx = x + w // 2
                    cy = y + h // 2
                    # draw box & center
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)

                    errX = cx - (frame.shape[1] // 2)
                    if abs(errX) > deadbandPx:
                        angle = xOffsetToAngle(errX, frame.shape[1])

        prevGray = gray

        # draw overlays
        H, W = frame.shape[:2]
        cv2.line(frame, (W//2, 0), (W//2, H), (255, 0, 0), 1)
        cv2.line(frame, (0, H//2), (W, H//2), (255, 0, 0), 1)
        cv2.putText(frame, f"Angle: {angle:.1f} deg", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 220, 50), 2)

        # update shared angle + last frame
        with angleLock:
            currentAngle = angle
        with lastFrameLock:
            lastFrame = frame.copy()

        # show window (close with 'q')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # terminal FPS once per second
        frames += 1
        t1 = time.time()
        if t1 - t0 >= 1.0:
            fps = frames / (t1 - t0)
            print(f"angle={angle:.1f}")
            t0 = t1
            frames = 0

    # cleanup
    with capLock:
        cap.release()
        cap = None
    print("[vision] stopped")

# ====== MJPEG streaming ======
def mjpegGenerator():
    while True:
        with lastFrameLock:
            frame = None if lastFrame is None else lastFrame.copy()
        if frame is None:
            time.sleep(0.01)
            continue
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, mjpegQuality])
        if not ok:
            continue
        chunk = jpg.tobytes()
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n"
               b"Content-Length: " + str(len(chunk)).encode() + b"\r\n\r\n" +
               chunk + b"\r\n")

@app.get("/stream")
def stream():
    return Response(mjpegGenerator(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.post("/snapshot")
def makeSnapshot():
    with lastFrameLock:
        if lastFrame is None:
            return jsonify({"ok": False, "error": "no frame yet"}), 503
        cv2.imwrite(snapshotPath, lastFrame)
    return jsonify({"ok": True, "path": "/snapshot.jpg"})

@app.get("/snapshot.jpg")
def getSnapshot():
    return send_file(snapshotPath, mimetype="image/jpeg", max_age=0)

# ====== API: get current angle (JSON) ======
@app.get("/api/angle")
def getAngle():
    with angleLock:
        a = float(currentAngle)
    return jsonify({"angle": a, "fov": fovDeg, "center": centerAngleDeg})

@app.get("/api/health")
def health():
    return jsonify({"ok": True})

def startCameraThread():
    th = threading.Thread(target=cameraLoop, daemon=True)
    th.start()
    return th

if __name__ == "__main__":
    # start vision thread
    th = startCameraThread()

    # IMPORTANT: requires a desktop/X session for the OpenCV window.
    # If you're on SSH without X forwarding, the window will fail;
    # the API and /stream will still work.

    try:
        app.run(host="0.0.0.0", port=8000, debug=False, threaded=True)
    finally:
        running = False
        time.sleep(0.2)
        cv2.destroyAllWindows()

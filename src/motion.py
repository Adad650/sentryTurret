import cv2, time, threading
from flask import Flask, jsonify, Response

app = Flask(__name__)
currentAngle = 90.0
targetFound = False
lastFrame = None
angleLock = threading.Lock()
frameLock = threading.Lock()

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

    app.run(host="0.0.0.0", port=17464, debug=False, threaded=True)

threading.Thread(target=apiThread, daemon=True).start()

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 30)
ret, frame = cap.read()
oldGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(oldGray, gray)
    _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    oldGray = gray

    angle = 90.0
    seen = False
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 5000:
            x, y, w, h = cv2.boundingRect(c)
            cx, cy = x + w // 2, y + h // 2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
            angle = (cx / frame.shape[1]) * 55  # map 0..width → 0..55
            seen = True

    with angleLock:
        if seen:
            currentAngle = angle
        targetFound = seen
        displayAngle = currentAngle

    with frameLock:
        lastFrame = frame.copy()

    cv2.putText(
        frame,
        f"Angle: {displayAngle:.1f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 220, 0),
        2,
    )
    cv2.imshow("Motion", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
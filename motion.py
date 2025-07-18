import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 60)

ret, old = cap.read()
old = cv2.cvtColor(old, cv2.COLOR_BGR2GRAY)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(old, gray)
    _, thresh = cv2.threshold(diff, 25, 255, 0)
    contours, _ = cv2.findContours(thresh, 1, 2)
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 500:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    # Resize the frame to 1080p resolution
    resized_frame = cv2.resize(frame, (3840, 2160))  # Resize to 3840x2160 (4K)
    cv2.imshow("footage", resized_frame)
    
    old = gray
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
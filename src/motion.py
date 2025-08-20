from time import sleep
import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 60)

ret, old = cap.read()
old = cv2.cvtColor(old, cv2.COLOR_BGR2GRAY)

def mapMotion(x, y):
    mappedX = (x/1920) * 100
    mappedY = (y/1080) * 100
    print (f"x = {round(mappedX)}%")
    print (f"y = {round(mappedY)}%\n")
    sleep(0.01)
    return 0

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(old, gray)
    _, thresh = cv2.threshold(diff, 25, 255, 0)
    contours, _ = cv2.findContours(thresh, 1, 2)
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 5000:
            x, y, w, h = cv2.boundingRect(c)
            centerX = x + w // 2
            centerY = y + h // 2
            #print(f"Motion center: ({center_x}, {center_y})")
            mapMotion(centerX + 1, centerY + 1)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Resize to 4K, then mirror horizontally

    resized_frame = cv2.resize(frame, (1920, 1080))
    mirrored = cv2.flip(resized_frame, 1)  # 1=horiz, 0=vert, -1=both
    cv2.imshow("footage", mirrored)
        



    old = gray
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
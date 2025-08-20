import cv2
import time
import RPi.GPIO as GPIO

PAN_SERVO_PIN = 21
TILT_SERVO_PIN = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(PAN_SERVO_PIN, GPIO.OUT)
GPIO.setup(TILT_SERVO_PIN, GPIO.OUT)

pwm_pan = GPIO.PWM(PAN_SERVO_PIN, 50)
pwm_tilt = GPIO.PWM(TILT_SERVO_PIN, 50)

pwm_pan.start(0)
pwm_tilt.start(0)

pan_angle = 90
tilt_angle = 90
print("Centering servos...")

duty_cycle_pan = (pan_angle / 18) + 2
pwm_pan.ChangeDutyCycle(duty_cycle_pan)
time.sleep(0.05)

duty_cycle_tilt = (tilt_angle / 18) + 2
pwm_tilt.ChangeDutyCycle(duty_cycle_tilt)
time.sleep(0.05)

cap = cv2.VideoCapture(0)
time.sleep(0.2)
if not cap.isOpened():
    print("Error: Could not open video stream.")
    GPIO.cleanup()
    exit()

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera resolution: {frame_width}x{frame_height}")

ret, previous_frame = cap.read()
if not ret:
    print("Error: Could not read initial frame from camera.")
    cap.release()
    GPIO.cleanup()
    exit()
    
previous_gray = cv2.cvtColor(previous_frame, cv2.COLOR_BGR2GRAY)
previous_gray = cv2.GaussianBlur(previous_gray, (21, 21), 0)

MIN_CONTOUR_AREA = 4000
DISPLAY_RESOLUTION = (1280, 720)
MOTION_THRESHOLD = 0.15
SERVO_INCREMENT = 1

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Stream ended.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        
        frame_delta = cv2.absdiff(previous_gray, gray)
        _, thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                (x, y, w, h) = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                center_x = x + w // 2
                center_y = y + h // 2
                
                if center_x < frame_width * (0.5 - MOTION_THRESHOLD):
                    pan_angle += SERVO_INCREMENT
                elif center_x > frame_width * (0.5 + MOTION_THRESHOLD):
                    pan_angle -= SERVO_INCREMENT
                
                if center_y < frame_height * (0.5 - MOTION_THRESHOLD):
                    tilt_angle += SERVO_INCREMENT
                elif center_y > frame_height * (0.5 + MOTION_THRESHOLD):
                    tilt_angle -= SERVO_INCREMENT

                pan_angle = max(0, min(180, pan_angle))
                tilt_angle = max(45, min(135, tilt_angle))

                duty_cycle_pan = (pan_angle / 18) + 2
                pwm_pan.ChangeDutyCycle(duty_cycle_pan)
                time.sleep(0.05)

                duty_cycle_tilt = (tilt_angle / 18) + 2
                pwm_tilt.ChangeDutyCycle(duty_cycle_tilt)
                time.sleep(0.05)

        previous_gray = gray

        display_frame = cv2.resize(frame, DISPLAY_RESOLUTION)
        mirrored_frame = cv2.flip(display_frame, 1)
        cv2.imshow("Motion Tracking", mirrored_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    print("Exiting program and cleaning up GPIO.")
    pwm_pan.stop()
    pwm_tilt.stop()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
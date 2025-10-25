# Sentry Turret - Motion Detection & Servo Tracking

This project implements advanced motion detection and servo tracking systems using OpenCV and Raspberry Pi. It includes multiple implementations with different features and control methods.

## Features

### Motion Detection & Single Servo Control (`motion.py`)
- **Real-time motion detection** using frame differencing
- **Single-axis servo control** (pan only) with RPi.GPIO
- **Web API** for remote monitoring and control
- **MJPEG streaming** of the camera feed
- **RESTful endpoints** for angle data and snapshots
- **Thread-safe** design for concurrent operation
- **Configurable** motion detection and servo parameters

### Pan-Only Servo Tracking (`test.py`)
- **Single-axis servo control** (pan) with RPi.GPIO
- **Motion detection** with configurable thresholds
- **Smooth motion** with adjustable speed and deadband
- **Camera configuration** for optimal performance
- **Keyboard controls** for manual operation

### Vision Server with Flask API (`otherTest.py`)
- **Web-based API** for motion detection and angle calculation
- **MJPEG streaming** endpoint
- **RESTful endpoints** for angle data and snapshots
- **Configurable** field of view and motion parameters
- **Cross-platform** operation (not limited to Raspberry Pi)

## Requirements

### Hardware
- **For servo control (motion.py, test.py):**
  - Raspberry Pi (any model)
  - SG90 or similar servo motor(s)
  - Camera module or USB webcam
  - Breadboard and jumper wires
  - 5V power supply for servos

### Software
- Python 3.7+
- OpenCV 4.8+
- RPi.GPIO (for servo control)
- Flask (for web interfaces)

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request

## Acknowledgments
- OpenCV for computer vision capabilities
- Flask for web interface
- Raspberry Pi community for hardware support

## Usage

### Motion Detection Only
```bash
python src/motion.py
```
- Press `q` to quit
- Motion coordinates are printed as percentages
- Green rectangle shows detected motion area

### Servo Tracking System
```bash
python src/test.py
```
- Press `q` to quit
- Press `c` to center servos
- Servos automatically track detected motion
- Blue crosshairs show tracking center

## Configuration

### Motion Detection Parameters
```python
detector = MotionDetector(
    camera_index=0,      # Camera device number
    threshold=25,        # Motion sensitivity (0-255)
    min_area=5000        # Minimum motion area (pixels)
)
```

### Servo Tracking Parameters
```python
# Camera field of view (adjust for your camera)
CAM_H_FOV = 60.0   # Horizontal field of view
CAM_V_FOV = 35.0   # Vertical field of view

# Control parameters
Kp_pan, Kd_pan   = 0.55, 0.18    # Pan PID gains
Kp_tilt, Kd_tilt = 0.55, 0.18    # Tilt PID gains
DEADBAND_DEG = 1.2               # Deadband (degrees)
STEP_MAX_DEG = 4.0               # Max step size (degrees)
```

## How It Works

### Motion Detection
1. **Frame Capture:** Grabs frames from camera at 30 FPS
2. **Frame Differencing:** Compares consecutive frames to detect changes
3. **Thresholding:** Applies binary threshold to isolate motion
4. **Contour Detection:** Finds moving objects using contour analysis
5. **Coordinate Mapping:** Converts pixel coordinates to percentage values

### Servo Tracking
1. **Background Subtraction:** Uses MOG2 algorithm for robust motion detection
2. **Motion Filtering:** Requires 3 consecutive detections for stability
3. **Coordinate Conversion:** Maps pixel errors to servo angles using FOV
4. **PD Control:** Proportional-derivative control for smooth movement
5. **Rate Limiting:** Prevents servo damage with maximum step limits

## Troubleshooting

### Common Issues
- **"No camera detected"**: Check camera connection and permissions
- **Servo jitter**: Reduce `Kp` and `Kd` values or increase `DEADBAND_DEG`
- **Poor tracking**: Adjust `CAM_H_FOV` and `CAM_V_FOV` for your camera
- **GPIO errors**: Ensure you're running on Raspberry Pi with proper permissions

### Performance Tips
- Use USB 3.0 camera for higher frame rates
- Reduce resolution if experiencing lag
- Adjust `min_area` based on your use case
- Fine-tune PID parameters for your specific setup

## License
This project is licensed under the MIT License.

## Contributing
Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.
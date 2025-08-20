# OpenCV Motion Detection & Servo Tracking

This project implements advanced motion detection and servo tracking systems using OpenCV. It includes both a standalone motion detector and a servo-controlled tracking system for Raspberry Pi.

## Features

### Motion Detection (`motion.py`)
- **Real-time motion detection** using frame differencing
- **Object tracking** with bounding box visualization
- **Percentage mapping** of motion coordinates
- **Dynamic resolution detection** - adapts to any camera
- **Robust error handling** and graceful cleanup
- **Configurable parameters** for sensitivity and detection area
- **Mirrored display** for intuitive viewing

### Servo Tracking (`test.py`)
- **Dual-axis servo control** (pan/tilt) for motion tracking
- **PD control system** for smooth servo movement
- **Background subtraction** for improved motion detection
- **Stable motion filtering** to reduce jitter
- **Field of view calibration** for accurate tracking
- **Rate limiting** to prevent servo damage
- **Emergency stop** and centering controls

## Requirements

### Hardware (for servo tracking)
- Raspberry Pi (any model)
- 2x SG90 or similar servo motors
- Camera module or USB webcam
- Breadboard and jumper wires

### Software
- Python 3.7+
- OpenCV 4.8+
- RPi.GPIO (for servo control)

## Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Adad650/openCV.git
   cd openCV
   ```

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Hardware setup (for servo tracking):**
   - Connect pan servo to GPIO pin 11 (BOARD numbering)
   - Connect tilt servo to GPIO pin 13 (BOARD numbering)
   - Power servos with 5V supply
   - Connect camera to USB or CSI port

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
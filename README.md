# Motion Detection with OpenCV

This project implements a motion detection system using OpenCV. It captures video from the webcam, detects motion by comparing frames, and highlights the area of motion with a green rectangle.

## Features
- Real-time motion detection using a webcam.
- Highlights the largest moving object with a green rectangle.
- Resizes the video feed to 1080p resolution for better visibility.

## Requirements
- Python 3.x
- OpenCV library (`cv2`)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/Adad650/openCV.git
   ```
2. Install the required dependencies:
   ```bash
   pip install opencv-python
   ```

## Usage
1. Run the script:
   ```bash
   python motion.py
   ```
2. Press `q` to exit the program.

## How It Works
1. Captures video frames from the webcam.
2. Converts frames to grayscale for processing.
3. Computes the difference between consecutive frames to detect motion.
4. Identifies the largest moving object and draws a green rectangle around it.
5. Resizes the video feed to 1080p resolution for display.

## Notes
- Ensure your webcam is connected and accessible.
- Adjust the motion sensitivity by modifying the threshold and contour area in the code.

## License
This project is licensed under the MIT License.
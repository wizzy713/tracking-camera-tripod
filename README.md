# CamX - Automated Tripod Tracking App

CamX is an intelligent Android application designed to work with a motorized tripod. It uses computer vision to detect subjects and automatically controls the tripod's pan/tilt servos to keep the subject centered in the frame.

## 🚀 Key Features

- **Predictive Tracking**: Uses a **Kalman Filter** to estimate subject velocity and predict future position, significantly reducing tracking lag.
- **Single-Target Focus**: Intelligent logic that locks onto the most prominent subject for stable motor movement.
- **Full Camera Suite**: Supports photo capture, video recording with audio, flash control, and front/rear camera switching.
- **Low Latency**: High-performance image analysis running at 30+ FPS.
- **Hardware Integration**: Sends real-time pan angles via UDP to an ESP32-controlled tripod.

## 🛠️ How it Works

### 1. Detection (MediaPipe & ML Kit)
The app uses **Google ML Kit Object Detection**, which is built on **MediaPipe** technology. This provides high-accuracy, on-device detection of people and objects without requiring an internet connection.

### 2. Prediction (Kalman Filter)
To solve the problem of "chasing" the subject (lag), CamX implements a custom **1D Kalman Filter**.
- It maintains a state of the subject's **position** and **velocity**.
- Every camera frame updates the filter.
- The app predicts where the subject will be in the next **100ms** to account for mechanical motor lag and network latency.

### 3. Communication (UDP)
The calculated "Lead Angle" is converted to a servo position (0° to 180°) and transmitted as a small UDP packet to the ESP32's IP address.

## 📦 Dependencies

- **CameraX**: Foundation for the camera preview, image analysis, and media capture.
- **ML Kit Object Detection**: Powers the subject identification.
- **Jetpack Compose**: Modern UI toolkit used for the entire application interface.
- **Material 3**: Google's latest design system for a professional look and feel.

## ⚙️ Setup

1. **Hardware**: An ESP32 connected to a servo motor.
2. **Network**: The Android device and ESP32 must be on the same Wi-Fi network.
3. **Configuration**: Update the `esp32Ip` variable in `MainActivity.kt` with your tripod's IP address.
4. **Android OS**: Compatible with Android 8.0+ and fully optimized for **Android 15+ (16 KB Page Size)**.

## 📄 License
This project is for educational and hobbyist use in robotics and computer vision.

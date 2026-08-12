# ARCHITECTURE.md - CamX Technical Guide

This document explains how CamX works, its folder structure, and the logic behind the automated tracking system.

## 📂 Project Structure

- **`app/src/main/java/com/example/tripodtracker/`**:
    - **`MainActivity.kt`**: The core of the app. It handles the UI (Compose), CameraX setup, and coordinates the AI detection and hardware communication.
    - **`KalmanFilter.kt`**: Contains the math for predicting where the subject will be next. It smooths out "jittery" camera data and provides trajectory estimation.
    - **`NsdHelper.kt`**: Helps the phone find your ESP32 on the Wi-Fi network using Network Service Discovery.
    - **`LogManager.kt`**: Records tracking coordinates and saves them as a CSV file for analysis in Excel.
- **`app/src/main/assets/`**:
    - **`hand_landmarker.task`**: The AI model for detecting hand gestures.
- **`gradle/`**:
    - Contains dependency management via `libs.versions.toml`.

---

## 🛰️ Localisation & Tracking Algorithm

CamX uses a multi-stage pipeline to keep the subject centered:

### 1. Perception (AI Detection)
The app runs two AI models concurrently:
- **Object Detection (ML Kit)**: Identifies people in the frame and returns bounding boxes with persistent tracking IDs.
- **Hand Landmarking (MediaPipe)**: Detects 21 key points on the hand to recognize gestures.

### 2. Localization (Center Pixel Tracking)
- **Calculation**: The app calculates the exact `X` and `Y` center pixels of the subject's bounding box.
- **Normalization**: Coordinates are based on the camera resolution (e.g., 640x480) for precision.

### 3. Estimation (Predictive Kalman Filter)
To account for motor and network lag, a **Kalman Filter** is applied to the `X` coordinate:
- It tracks **Position** and **Velocity**.
- **Prediction**: It estimates where the subject will be **100ms in the future**.
- This "lead" ensures the tripod stays ahead of the motion rather than lagging behind.

### 4. Control (UDP Communication)
- **Protocol**: UDP for ultra-low latency.
- **Payload**: Sends a string `X:value,Y:value`.
- **ESP32 Side**: The microcontroller parses this string and maps the pixels to servo PWM pulses.

---

## ✋ Subject Locking (Gesture Recognition)
In a crowd, the app might see many people. The **Palm Lock** feature allows manual selection:
1. When a user raises an **Open Palm**, MediaPipe detects the extended fingers.
2. The app finds the person closest to that hand.
3. It "locks" on that specific person's ID.
4. The bounding box turns **Cyan**, and the app ignores all other subjects until unlocked.

---

## 🏗️ Folder/Code Connectivity
1. **`CameraPreviewScreen`** (MainActivity): The visual entry point.
2. **`imageAnalyzer`**: Feeds every frame to `processImageProxy`.
3. **`processImageProxy`**: The engine room. Calls detection models, runs the Kalman Filter, and calculates locking logic.
4. **`sendUdpCommand`**: Beams the final coordinates to the tripod over Wi-Fi.

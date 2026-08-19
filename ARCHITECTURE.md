# CamX Architecture and Technical Specification

This document provides a technical overview of the CamX system architecture, the localization algorithms used, and the hardware communication layer.

## System Components and Folder Structure

The project follows a modular structure to separate concerns between the UI, the vision engine, and the hardware communication.

- src/main/java/com/example/tripodtracker/:
    - MainActivity.kt: Orchestrates the Android component lifecycle, Compose UI, and high-level application state transitions.
    - KalmanFilter.kt: Implementation of a Discrete Kalman Filter for state estimation (Position/Velocity) of the tracked subject.
    - UdpSender.kt: Encapsulates a DatagramSocket within a dedicated background thread to handle non-blocking network I/O.
    - LogManager.kt: Provides thread-safe recording of subject coordinates to the local file system in CSV format.

## Localization Algorithm

The core tracking algorithm is based on the recursive Bayesian estimation of the subject's center-pixel coordinates.

### 1. Object Detection (Perception)
Subject detection is performed using a specialized model from Google ML Kit. This provides a rectangular bounding box defined by [top, left, bottom, right] coordinates in the image coordinate system. This model runs at 30Hz (every frame) for maximum responsiveness.

### 2. Gesture Recognition (Locking)
MediaPipe Hand Landmarker runs concurrently at 6Hz (every 5th frame) to identify hand keypoints. This throttling ensures system stability and reduces thermal overhead on mobile hardware.
- Gesture: "Open Palm" is detected when the four fingers (index, middle, ring, pinky) are extended.
- Logic: When a palm is raised, the system identifies the subject bounding box closest to the hand's geometric center and locks tracking to that ID.

### 3. Center-Pixel Calculation
The subject's position is localized by calculating the geometric center of the bounding box:
X_raw = (box.left + box.right) / 2
Y_raw = (box.top + box.bottom) / 2

### 4. Trajectory Prediction (Kalman Filter)
To compensate for motor latency and network delay, a 1D Kalman Filter is applied to the X-axis coordinate.
- State Vector: [Position, Velocity]
- Transition Matrix: Assumes constant velocity between frames.
- Innovation: The difference between the detected X_raw and the predicted position.
- Output: A smoothed, predicted coordinate X_predicted = X + Velocity * 0.1 (100ms lead time).

## Hardware Communication Layer

Data transmission to the ESP32 tripod is handled via UDP.

- Protocol: UDP over IPv4.
- Payload: "X:[INT],Y:[INT]"
- Rate: Commands are dispatched immediately following successful frame analysis, typically at 30Hz.

## Subject Discovery and Initialization

Subject discovery is handled manually via the connection settings. The user specifies the target IP address and Port of the ESP32 hardware to establish the UDP link.

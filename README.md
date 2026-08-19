# CamX - Automated Tripod Tracking System

CamX is a technical Android application designed to interface with motorized tripod hardware. It provides real-time subject localization using on-device computer vision and transmits precision tracking coordinates to external hardware via low-latency network protocols.

## System Features

- Open Palm Gesture Locking: Raise an open palm to lock the tracker onto a specific person in a crowd.
- Predictive Tracking: Implements a 1D Kalman Filter to estimate subject velocity and predict trajectory coordinates.
- Precision Localization: Transmits exact X and Y center-pixel coordinates to the hardware layer.
- Connectivity: Manual IP/port configuration to link with tripod hardware on the local Wi-Fi network.
- Diagnostic Tools: Includes connection testing with custom payloads and telemetry logging in CSV format.
- Comprehensive Camera Control: Provides high-resolution photo capture, video recording with audio, and hardware flip capabilities.

## Architecture and Workflow

### 1. Localization Algorithm
The localization process follows a multi-stage pipeline:
- Frame Acquisition: Frames are captured via CameraX at a resolution of 640x480.
- Subject Identification: Google ML Kit Object Detection analyzes the frame to produce subject bounding boxes.
- Gesture Recognition: MediaPipe Hand Landmarker identifies an open palm gesture to initiate subject locking.
- Center-Pixel Calculation: The mathematical center of the bounding box is calculated as the primary coordinate.
- Trajectory Estimation: A Kalman Filter processes the coordinates to filter noise and predict the subject's position 100ms into the future.

### 2. Communication Protocol
- Hardware Link: UDP (User Datagram Protocol) is utilized for minimum latency transmission.
- Payload Format: Coordinates are transmitted as a standard string: "X:[value],Y:[value]".
- Configuration: The IP address and port of the ESP32 are entered manually in the connection settings.

## Project Structure

- MainActivity.kt: Central controller managing application state, UI composition, and hardware coordination.
- KalmanFilter.kt: Mathematical implementation for state estimation and trajectory prediction.
- UdpSender.kt: Managed network worker for asynchronous hardware command transmission.
- LogManager.kt: Utility for persistent CSV-based telemetry recording.

## Setup Requirements

- Hardware: ESP32-based microcontroller with servo motor integration. See [firmware folder](./firmware) for details.
- Network: Android device and ESP32 must reside on the same subnet.
- Configuration: Tripod parameters (IP and Port) are managed via the in-app connection settings.
- AI Assets: Ensure hand_landmarker.task is present in the assets folder.

## Dependencies

- Android Jetpack CameraX (v1.4.1)
- Google ML Kit Object Detection
- MediaPipe Tasks Vision (v0.10.14)
- Android Jetpack Compose (Material 3)
- Kotlin Coroutines for asynchronous processing

## License
Educational and hobbyist use only.

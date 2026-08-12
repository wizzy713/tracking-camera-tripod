# CamX Architecture and Technical Specification

This document provides a technical overview of the CamX system architecture, the localization algorithms used, and the hardware communication layer.

## System Components and Folder Structure

The project follows a modular structure to separate concerns between the UI, the vision engine, and the hardware communication.

- src/main/java/com/example/tripodtracker/:
    - MainActivity.kt: Orchestrates the Android component lifecycle, Compose UI, and high-level application state transitions.
    - KalmanFilter.kt: Implementation of a Discrete Kalman Filter for state estimation (Position/Velocity) of the tracked subject.
    - UdpSender.kt: Encapsulates a DatagramSocket within a dedicated background thread to handle non-blocking network I/O.
    - NsdHelper.kt: Manages the registration and discovery of network services using the mDNS protocol.
    - LogManager.kt: Provides thread-safe recording of subject coordinates to the local file system in CSV format.

## Localization Algorithm

The core tracking algorithm is based on the recursive Bayesian estimation of the subject's center-pixel coordinates.

### 1. Object Detection (Perception)
Subject detection is performed using a specialized model from Google ML Kit. This provides a rectangular bounding box defined by [top, left, bottom, right] coordinates in the image coordinate system.

### 2. Center-Pixel Calculation
The subject's position is localized by calculating the geometric center of the bounding box:
X_raw = (box.left + box.right) / 2
Y_raw = (box.top + box.bottom) / 2

### 3. Trajectory Prediction (Kalman Filter)
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

CamX utilizes Network Service Discovery (NSD) to automate hardware pairing.
- Service Type: "_arduino._tcp."
- Resolution: When a service is identified, the app resolves the IP address and Port number automatically, updating the UdpSender configuration without manual user intervention.

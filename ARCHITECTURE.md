# CamX Architecture and Technical Specification

This document provides a technical overview of the CamX system architecture, the localization algorithms used, and the hardware communication layer.

## System Components and Folder Structure

The project follows a modular structure to separate concerns between the UI, the vision engine, and the hardware communication.

- src/main/java/com/example/tripodtracker/:
    - MainActivity.kt: Orchestrates the Android component lifecycle, Compose UI, frame-geometry handling, and hardware coordination.
    - KalmanFilter.kt: Discrete Kalman Filter (Position/Velocity, white-noise-acceleration process model) for state estimation of the tracked subject. One instance per axis.
    - UdpSender.kt: Encapsulates a DatagramSocket within a dedicated background thread to handle non-blocking network I/O.
    - LogManager.kt: Provides thread-safe recording of per-frame tracking state to the local file system in CSV format.

## Localization Algorithm

The core tracking algorithm is based on the recursive Bayesian estimation of the subject's center-pixel coordinates.

### 1. Object Detection (Perception)
Subject detection uses Google ML Kit's on-device Object Detection and Tracking API in `STREAM_MODE`. **This is a generic "prominent object" detector, not a person detector** -- ML Kit ships no person class, so the tripod can lock onto any sufficiently prominent object in frame, not specifically a person. `DetectedObjectInfo.label` reflects this (labelled `"Object"`, not `"Person"`). Detected objects are reported as rectangular bounding boxes in the *upright* (post-rotation) image coordinate frame. This runs at 30Hz (every frame) for maximum responsiveness.

A known follow-up (not yet implemented) is swapping this for MediaPipe's Object Detector with an EfficientDet-Lite COCO model filtered to the `person` class, or a Pose Landmarker, to actually constrain tracking to people.

### 2. Gesture Recognition (Locking)
MediaPipe Hand Landmarker runs concurrently at 6Hz (every 5th frame) to identify hand keypoints, in `RunningMode.IMAGE`. This throttling ensures system stability and reduces thermal overhead on mobile hardware.
- Gesture: "Open Palm" is detected when the four fingers (index, middle, ring, pinky) are extended above their PIP joints. This is a simple heuristic (not MediaPipe's built-in gesture classifier) and assumes a roughly upright hand.
- Logic: When a palm is raised, the system identifies the subject bounding box closest to the hand's geometric center and locks tracking to that ID.
- The frame used for hand detection is rotated to match ML Kit's upright coordinate frame (see "Frame Geometry" below), so palm coordinates and bounding-box coordinates are directly comparable.

### 3. Frame Geometry
`ImageProxy.width`/`height` are the *unrotated* sensor buffer dimensions, but ML Kit's detections (and the hand-landmarker bitmap, which is explicitly rotated to match) are in the *upright*, post-rotation frame. In portrait orientation (rotation 90/270) these differ: the effective upright frame is `imageProxy.height x imageProxy.width`, not `imageProxy.width x imageProxy.height`. All box centers, palm coordinates, and the normalized error below are computed against this corrected upright frame size, computed once per frame from `rotationDegrees`.

### 4. Normalized Error Calculation
The subject's position is localized as the geometric center of its bounding box, then converted to a normalized error relative to frame center:
```
X_raw = box.centerX()
err_X = (X_predicted - frameWidth/2) / (frameWidth/2)   // in [-1, 1]
```
The same applies to Y. Errors are computed from the *predicted* (not raw) position -- see below. For the front camera, `err_X` is mirrored (negated) to match the mirrored preview and the user's real-world left/right, since the sensor image itself is not mirrored but the on-screen preview and the pan direction the user expects are.

### 5. Trajectory Prediction (Kalman Filter)
To compensate for motor latency and network delay, a 1D Kalman Filter (constant-velocity state, discrete white-noise-acceleration process model) is applied independently to **both** the X and Y coordinates -- earlier versions only filtered X, leaving tilt visibly jerkier than pan.
- State Vector: `[Position, Velocity]`, one filter instance per axis.
- Filter input timestamp is the frame's monotonic capture time (`ImageProxy.imageInfo.timestamp`), not wall-clock time, so `dt` reflects actual capture spacing rather than processing jitter.
- Measurement noise `R` and process noise (`sigma_a`, acceleration std-dev) are tunable constructor parameters on `KalmanFilter` (see `DEFAULT_MEASUREMENT_NOISE` / `DEFAULT_ACCELERATION_NOISE`). The current defaults are reasoned estimates, not measured values -- see `LogManager`'s CSV output for tuning them against real data.
- Output: `predicted = position + velocity * PREDICTION_HORIZON_SECONDS` (currently a hardcoded 100ms placeholder -- replace with a measured end-to-end latency once that experiment is run).
- Target loss handling: if a locked target's tracking ID isn't matched for up to `MAX_COAST_FRAMES` (15) consecutive frames, the filter coasts on its own prediction rather than jumping to an arbitrary detected object; beyond that the lock is released.

## Hardware Communication Layer

Data transmission to the ESP32 tripod is handled via UDP.

- Protocol: UDP over IPv4.
- Payload Format: `"EX:[FLOAT],EY:[FLOAT],SEQ:[UINT]"` -- normalized error in `[-1, 1]` per axis, plus a monotonically increasing sequence number. This is intentionally decoupled from camera resolution, aspect ratio, and orientation: the firmware never needs to know the frame size.
- Rate: Commands are dispatched immediately following successful frame analysis, typically at 30Hz.
- Sequencing: `SEQ` lets the firmware detect and drop out-of-order/duplicate packets, and lets you measure packet loss from the gaps in `Seq` in the logged CSV.
- Firmware control: The ESP32 applies proportional (P) control -- `step = clamp(KP * err, -MAX_STEP, MAX_STEP)` per axis, not a fixed per-packet increment -- so response scales with how far off-center the subject is. See `firmware/camx_tripod/camx_tripod.ino`.

## CSV Logging Schema

When logging is enabled, one row per processed frame is recorded:

`Timestamp, FrameTimestampNanos, Seq, DetectionCount, RawX, RawY, FilteredX, FilteredY, VelocityX, VelocityY, DtSeconds`

`RawX`/`RawY` are `NaN` on frames with no fresh measurement (coasting). Because `VelocityX`/`VelocityY` are logged, a predicted position at *any* horizon can be reconstructed offline as `FilteredX + VelocityX * horizonSeconds` -- this is what a prediction-horizon sweep experiment should use, rather than re-running the app at each horizon.

## Subject Discovery and Initialization

Subject discovery is handled manually via the connection settings. The user specifies the target IP address and Port of the ESP32 hardware to establish the UDP link. The firmware still advertises itself via mDNS (`_arduino._tcp.`) for future discovery tooling, but nothing in the app currently consumes it.

## Known Limitations / Suggested Follow-ups

- **Generic object detector, not a person detector** (see above) -- highest-value fix for tracking correctness.
- **Gesture recognition is a hand-rotation-sensitive heuristic**, not MediaPipe's built-in `GestureRecognizer` (`Open_Palm` category), and runs in `RunningMode.IMAGE` (blocking) rather than `RunningMode.LIVE_STREAM`.
- **Settings (IP/port) are not persisted** across app restarts; no `ViewModel`/`DataStore` layer, so configuration and tracking state also don't survive a configuration change (e.g. rotation).
- **No automated evaluation harness.** The CSV schema above supports it, but end-to-end latency, prediction-horizon sweep, ablation (raw vs. filtered vs. predicted RMSE), and packet-loss experiments still need to be run and reported by hand.
- **Camera-analysis resolution is not pinned** (no `ResolutionSelector` on `ImageAnalysis`); it varies by device. This no longer causes correctness bugs (the frame-geometry and normalized-protocol fixes above are resolution-agnostic), but it does mean absolute pixel jitter -- and therefore the tuned `KalmanFilter` noise constants -- may vary by device.

## Ethical Considerations

CamX is a motorized camera that can autonomously detect, lock onto, and record people. Before deploying it beyond controlled testing:
- **Bystander consent**: anyone the system tracks or records, intentionally or incidentally, should be informed or the system should only be operated in contexts where this is not a concern (e.g. a single consenting subject in a private space).
- **Data retention**: the CSV logs record per-frame subject coordinates and timestamps, which is behavioral/location data about whoever was tracked. Logs should be stored only as long as needed for evaluation and not shared without the tracked subject's consent.
- **Physical safety**: the pan/tilt servos are mechanically constrained to 0-180 degrees, but anyone operating or standing near the tripod should be aware it moves autonomously and can be startled or physically obstructed by it.

## Dependencies

- Android Jetpack CameraX (v1.4.1)
- Google ML Kit Object Detection
- MediaPipe Tasks Vision (v0.10.14)
- Android Jetpack Compose (Material 3)
- Kotlin Coroutines for asynchronous processing

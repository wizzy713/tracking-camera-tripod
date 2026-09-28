# CamX Architecture and Technical Specification

This document provides a technical overview of the CamX system architecture, the localization algorithms used, and the hardware communication layer.

## System Components and Folder Structure

The project follows a modular structure to separate concerns between the UI, the vision engine, and the hardware communication.

- src/main/java/com/example/tripodtracker/:
    - MainActivity.kt: Orchestrates the Android component lifecycle, Compose UI, frame-geometry handling, and hardware coordination.
    - KalmanFilter.kt: Discrete Kalman Filter (Position/Velocity, white-noise-acceleration process model) for state estimation of the tracked subject. One instance per axis.
    - UdpSender.kt: Encapsulates a DatagramSocket for non-blocking network I/O -- one background thread serializes outbound error packets, a second daemon thread drains the firmware's replies and surfaces battery telemetry (`BatteryStatus`) via an `onBattery` callback.
    - LogManager.kt: Provides thread-safe recording of per-frame tracking state to the local file system in CSV format.

## Localization Algorithm

The core tracking algorithm is based on the recursive Bayesian estimation of the subject's center-pixel coordinates.

### 1. Object Detection (Perception)
Subject detection runs one of two ML Kit pipelines, selectable per-session as `DetectionMode` (Experiment tab, or the camera-screen toggle icon; default `FACE`):

- **`FACE`** (default): Google ML Kit Face Detection (`FaceDetectorOptions`, `PERFORMANCE_MODE_FAST`, `enableTracking()`). Locks onto faces only.
- **`OBJECT`**: Google ML Kit Object Detection and Tracking (`ObjectDetectorOptions`, `STREAM_MODE`, `enableMultipleObjects()`, no classification). **This is a generic "prominent object" detector, not a person detector** -- ML Kit ships no person class and no classification is enabled here, so the tripod can lock onto any sufficiently prominent object in frame, not specifically a person (or face). Useful for subjects a face detector can't see, e.g. the pendulum test in `TESTING.md`.

Both pipelines are mapped to a common `DetectedObjectInfo(boundingBox, trackingId)` shape in `processImageProxy` (`MainActivity.kt`) immediately on detection, so every downstream step -- target selection, hand-lock distance, the Kalman filter, the servo command -- is written once and works identically regardless of which detector produced it. `DetectedObjectInfo.label` stays the default `"Object"` in both modes (ML Kit provides no person class, and Object mode's classification is intentionally left off since nothing consumes it). Detected objects are reported as rectangular bounding boxes in the *upright* (post-rotation) image coordinate frame. Detection runs at 30Hz (every frame) for maximum responsiveness.

A known follow-up (not yet implemented) is swapping `OBJECT` mode for MediaPipe's Object Detector with an EfficientDet-Lite COCO model filtered to the `person` class, or a Pose Landmarker, to constrain tracking to people specifically while still not requiring a visible face.

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
- Measurement noise `R` and process noise (`sigma_a`, acceleration std-dev) are mutable `var` properties on `KalmanFilter` (see `DEFAULT_MEASUREMENT_NOISE` / `DEFAULT_ACCELERATION_NOISE` for their starting values), live-tunable from the Experiment tab against the running `kalmanFilterX`/`kalmanFilterY` instances -- no app restart needed. The shipped defaults are reasoned estimates, not measured values -- see `LogManager`'s CSV output (now including `ErrX`/`ErrY`, see below) for tuning them against real data.
- Output: `predicted = position + velocity * predictionHorizonSeconds`. `predictionHorizonSeconds` starts at `DEFAULT_PREDICTION_HORIZON_SECONDS` (200ms, a placeholder) and is also live-tunable from the Experiment tab -- replace it with a measured end-to-end latency once that experiment (see `TESTING.md`) is run.
- Target loss handling: if a locked target's tracking ID isn't matched for up to `MAX_COAST_FRAMES` (15) consecutive frames, the filter coasts on its own prediction rather than jumping to an arbitrary detected object; beyond that the lock is released.

## Hardware Communication Layer

Data transmission to the ESP32 tripod is handled via UDP.

- Protocol: UDP over IPv4.
- Payload Format (app -> firmware): `"EX:[FLOAT],EY:[FLOAT],SEQ:[UINT]"` -- normalized error in `[-1, 1]` per axis, plus a monotonically increasing sequence number. This is intentionally decoupled from camera resolution, aspect ratio, and orientation: the firmware never needs to know the frame size.
- Rate: Commands are dispatched immediately following successful frame analysis, typically at 30Hz.
- Sequencing: `SEQ` lets the firmware detect and drop out-of-order/duplicate packets, and lets you measure packet loss from the gaps in `Seq` in the logged CSV.
- Firmware control: The ESP32 applies PID speed control in raw microseconds -- `pulse_us = NEUTRAL_US +/- clamp(KP*err + KI*integral, -MAX_SPEED_OFFSET_US, MAX_SPEED_OFFSET_US)` per axis (KD = 0 by design) -- so the commanded rotation speed scales with how far off-center the subject is, for continuous-rotation servos. The gains are derived from an integrator-plant model (loop delay sets the gain ceiling); see the header comment in `firmware/camx_tripod/camx_tripod.ino` and "Tuning the PID" in `firmware/README.md`.

### Live Tuning (Experiment tab): `CFG` protocol

The Experiment tab (Kotlin: `ExperimentScreen.kt`) tunes both controllers live against a
running rig:

- **PID gains** (`KP`/`KI`/`KD`/`MAX_SPEED_OFFSET_US`/`DEADZONE`) live on the firmware, so
  they're set over the same UDP flow as the `EX`/`EY` packets: `CFG:KP:<v>,KI:<v>,KD:<v>,MS:<v>,DZ:<v>`
  to set all five at once, or `CFG?` to query without changing anything. The firmware
  always replies on the same flow with its full current gain set in the same format, which
  the app parses into `UdpSender.onConfig` / `MainActivity.tripodConfig` and uses to re-seed
  the Experiment tab's sliders. Gains are RAM-only on the firmware (reset to the `.ino`
  defaults on reboot), same as the pre-existing Serial-tuning path. See "CFG protocol" in
  `firmware/README.md` for the full spec.
- **Kalman noise** (`measurementNoise`/`accelerationNoise`) and the **prediction horizon**
  live entirely in the app, so the Experiment tab writes them straight into the running
  `kalmanFilterX`/`kalmanFilterY` instances and `MainActivity.predictionHorizonSeconds` --
  no network round-trip, no persistence (reset to the `DEFAULT_*` constants on app restart,
  same as the IP/port settings noted under "Known Limitations").

### Reverse channel: battery telemetry (firmware -> app)

The firmware replies on the **same UDP flow** -- back to the app's source IP/port, which the app reads on the very socket it sends from, so no extra listening port is needed. Roughly every 2s, once at least one error packet has arrived:

- Payload Format: `"BATT:[INT 0-100],MV:[INT],MA:[INT],WH:[FLOAT]"` -- state of charge %, pack terminal millivolts, pack milliamps (positive = discharging), watt-hours remaining.
- `UdpSender` parses this on its receive thread into a `BatteryStatus` and hands it to `MainActivity` via `onBattery`; the camera overlay renders `BatteryIndicator` (fill glyph + % + voltage, colour-coded by charge). With no INA219 on the tripod, no packet is ever sent and the indicator stays hidden.

## Battery Monitoring

An INA219 high-side sensor sits **before** the 5V regulator, so it measures the raw 2S Li-ion pack (2x 3.7V nominal, 2600mAh / 9.62Wh): terminal voltage and the total current the whole rig draws. The firmware fuel gauge (`camx_tripod.ino`) blends two estimates:

1. **Coulomb counting** -- integrate current out of the running charge estimate (seeded at boot from the pack's open-circuit voltage). Accurate short-term, drifts over hours.
2. **Open-circuit voltage** -- `OCV ~= V_terminal + I_discharge * R_internal`, mapped through a per-cell Li-ion resting-voltage curve. Absolute, but noisy under load and flat mid-charge.

Fusion is asymmetric: near rest the coulomb counter is pulled toward the voltage estimate; under load the voltage estimate can only pull the counter *down* (so a near-empty pack can't hide behind a stale count, but a servo-surge sag can't make the gauge jump). Outputs: SoC %, Wh remaining, and the onboard RGB LED colour (green > 50%, amber 20-50%, red < 20%, blinking < 10%). Tuning constants (`BATT_CAPACITY_MAH`, `BATT_IR_OHMS`, `INA_CURRENT_SIGN`, the OCV table) and the wiring diagram are in `firmware/README.md` ("Battery monitoring").

## CSV Logging Schema

When logging is started from the Experiment tab, the saved file begins with a `# key:
value` comment block -- the test name/notes and every tunable in effect for that run
(PID gains, Kalman noise, prediction horizon, detection mode; see
`MainActivity.buildSessionMetadata()`), so each CSV is self-describing about which
settings produced it. Logging can still be started from the Settings screen without this
metadata (the block is simply omitted).

Below that, one row per processed frame is recorded:

`Timestamp, FrameTimestampNanos, Seq, DetectionCount, RawX, RawY, FilteredX, FilteredY, VelocityX, VelocityY, DtSeconds, ErrX, ErrY`

`RawX`/`RawY` are `NaN` on frames with no fresh measurement (coasting). Because `VelocityX`/`VelocityY` are logged, a predicted position at *any* horizon can be reconstructed offline as `FilteredX + VelocityX * horizonSeconds` -- this is what a prediction-horizon sweep experiment should use, rather than re-running the app at each horizon. `ErrX`/`ErrY` are the normalized tracking error in `[-1, 1]` (subject offset from frame centre, as a fraction of half-frame) -- the actual `EX`/`EY` values sent to the tripod that frame. This is the primary tracking-error signal for evaluating a test run (e.g. RMS error over a pendulum swing); see `TESTING.md`.

## Subject Discovery and Initialization

Subject discovery is handled manually via the connection settings. The user specifies the target IP address and Port of the ESP32 hardware to establish the UDP link. The firmware still advertises itself via mDNS (`_arduino._tcp.`) for future discovery tooling, but nothing in the app currently consumes it.

## Known Limitations / Suggested Follow-ups

- **Generic object detector, not a person detector** (see above) -- highest-value fix for tracking correctness.
- **Gesture recognition is a hand-rotation-sensitive heuristic**, not MediaPipe's built-in `GestureRecognizer` (`Open_Palm` category), and runs in `RunningMode.IMAGE` (blocking) rather than `RunningMode.LIVE_STREAM`.
- **Settings (IP/port, Experiment-tab gains) are not persisted** across app restarts; no `ViewModel`/`DataStore` layer, so configuration and tracking state also don't survive a configuration change (e.g. rotation). The Experiment tab re-syncs PID gains from the firmware's `CFG` reply on open, so a reconnect after an app restart still recovers the tripod's actual live values -- only the local Kalman/prediction-horizon tuning is lost.
- **No automated evaluation harness.** The CSV schema above (now including `ErrX`/`ErrY`) and the Experiment tab's live tuning support one, but end-to-end latency, prediction-horizon sweep, ablation (raw vs. filtered vs. predicted RMSE), and packet-loss experiments still need to be run and reported by hand -- see `TESTING.md` for the suggested protocol.
- **Camera-analysis resolution is not pinned** (no `ResolutionSelector` on `ImageAnalysis`); it varies by device. This no longer causes correctness bugs (the frame-geometry and normalized-protocol fixes above are resolution-agnostic), but it does mean absolute pixel jitter -- and therefore the tuned `KalmanFilter` noise constants -- may vary by device.

## Ethical Considerations

CamX is a motorized camera that can autonomously detect, lock onto, and record people. Before deploying it beyond controlled testing:
- **Bystander consent**: anyone the system tracks or records, intentionally or incidentally, should be informed or the system should only be operated in contexts where this is not a concern (e.g. a single consenting subject in a private space).
- **Data retention**: the CSV logs record per-frame subject coordinates and timestamps, which is behavioral/location data about whoever was tracked. Logs should be stored only as long as needed for evaluation and not shared without the tracked subject's consent.
- **Physical safety**: the pan/tilt servos are mechanically constrained to 0-180 degrees, but anyone operating or standing near the tripod should be aware it moves autonomously and can be startled or physically obstructed by it.

## Dependencies

- Android Jetpack CameraX (v1.4.1)
- Google ML Kit Face Detection + Object Detection (`DetectionMode.FACE` / `.OBJECT`)
- MediaPipe Tasks Vision (v0.10.14)
- Android Jetpack Compose (Material 3)
- Kotlin Coroutines for asynchronous processing
- Firmware: Adafruit INA219 + Adafruit BusIO (battery monitoring), ESP32Servo

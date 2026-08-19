# CamX ESP32 Firmware

This folder contains the ESP32 firmware for the automated tracking tripod hardware.

## Features
- WiFi and UDP support for low-latency coordinate receiving.
- mDNS Advertisement: The device announces itself as `_arduino._tcp.` on the local network for future discovery tooling (the Android app currently uses manual IP/port entry).
- Proportional Servo Control: Smooth movement based on real-time person coordinates.

## Hardware Requirements
- ESP32 Microcontroller
- 2x High-torque Servo Motors (Pan and Tilt)
- External 5V/3A Power Supply (Do not power servos from ESP32 pins)

## Wiring Diagram (Default)
- **Pan Servo (X-Axis)**: Signal to GPIO 18
- **Tilt Servo (Y-Axis)**: Signal to GPIO 19
- **GND**: Connect ESP32 Ground and Servo Ground together.

## Setup Instructions
1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the **ESP32 Board Support** package.
3. Install the **ESP32Servo** library via Library Manager.
4. Open `camx_tripod.ino` in this folder.
5. Update `ssid` and `password` with your WiFi credentials.
6. Select your ESP32 board and click **Upload**.

## Protocol
The Android app sends UDP packets to port 4210 in the format: `EX:value,EY:value,SEQ:value`
Where `EX`/`EY` are the subject's X/Y offset from frame center, normalized to `[-1, 1]`
(0 = centered, independent of camera resolution or orientation), and `SEQ` is a
monotonically increasing packet sequence number used to detect and drop
out-of-order or duplicate packets.

The firmware applies proportional control: `step = clamp(KP * error, -MAX_STEP, MAX_STEP)`
per axis, so servo movement scales with how far off-center the subject is rather
than moving a fixed amount per packet. Tune `KP`, `MAX_STEP`, and `DEADZONE` at
the top of `camx_tripod.ino` for your servos and desired responsiveness.

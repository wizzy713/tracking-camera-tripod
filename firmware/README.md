# CamX ESP32 Firmware

This folder contains the ESP32 firmware for the automated tracking tripod hardware.

## Features
- WiFi and UDP support for low-latency coordinate receiving.
- mDNS Auto-Discovery: The Android app will automatically find this device on the network.
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
The Android app sends UDP packets to port 4210 in the format: `X:value,Y:value`
Where X and Y are pixel coordinates from a 640x480 frame.
320, 240 is considered the exact center.

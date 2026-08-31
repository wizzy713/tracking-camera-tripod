# CamX ESP32 Firmware

This folder contains the ESP32 firmware for the automated tracking tripod hardware.

## Features
- WiFi and UDP support for low-latency coordinate receiving.
- Proportional Speed Control: for continuous-rotation (360-degree) servos, which have no
  absolute position -- the firmware commands a speed/direction each update rather than an
  angle to move to and hold, and explicitly stops when the subject is centered.
- Loss-of-signal failsafe: stops both motors if no UDP packet arrives for 500ms, so a
  dropped connection or closed app can't leave a continuous-rotation servo spinning forever.

## Hardware Requirements
- ESP32-C6 Microcontroller (e.g. ESP32-C6-DevKitC-1 / DevKitM-1, WiFi 6)
- 2x Continuous-rotation ("360-degree") Servo Motors (Pan and Tilt) -- standard positional
  (0-180-degree) servos are NOT compatible with the current control scheme, since they can
  only move to and hold an angle rather than spin at a commanded speed.
- External 5V/3A Power Supply (Do not power servos from ESP32 pins)
- **Common ground is required**: tie the external supply's GND, each servo's GND wire, and
  the ESP32's GND pin together. Without a shared ground, the PWM signal has no valid
  reference against the servo's own power rail and the servo will not respond correctly.

## Wiring Diagram (Default)
- **Pan Servo (X-Axis)**: Signal to GPIO 2
- **Tilt Servo (Y-Axis)**: Signal to GPIO 3
- **GND**: Connect ESP32 Ground and Servo Ground together.

On the ESP32-C6, avoid these pins for servo signal wiring:
- GPIO4, 5, 8, 9, 15 — strapping pins, sampled at boot/reset
- GPIO8 — also drives the onboard RGB LED on DevKit boards
- GPIO12, 13 — reserved for native USB (USB_D-/USB_D+)

Other safe alternatives: GPIO0, 1, 6, 7, 10, 11, 14, 20, 21, 22, 23 (verify
against your specific board's silkscreen/pinout diagram, since breakout
boards vary in which pins are actually broken out).

## Setup Instructions
1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install the **esp32 by Espressif Systems** board package (Boards Manager), version >= 3.0, and select an ESP32-C6 board under Tools > Board.
3. Install the **ESP32Servo** library (>= 3.0) via Library Manager — older versions predate the core 3.x LEDC API and won't compile for the C6.
4. Open `camx_tripod.ino` in this folder.
5. Update `ssid` and `password` with your WiFi credentials.
6. Click **Upload**.

## Protocol
The Android app sends UDP packets to port 4210 in the format: `EX:value,EY:value,SEQ:value`
Where `EX`/`EY` are the subject's X/Y offset from frame center, normalized to `[-1, 1]`
(0 = centered, independent of camera resolution or orientation), and `SEQ` is a
monotonically increasing packet sequence number used to detect and drop
out-of-order or duplicate packets.

The firmware applies proportional speed control: `speed = NEUTRAL +/- clamp(KP * error, -MAX_SPEED_OFFSET, MAX_SPEED_OFFSET)`
per axis, so rotation speed scales with how far off-center the subject is. When `|error|`
is within `DEADZONE`, the firmware writes `NEUTRAL` (stop) instead of a speed offset.
Tune `KP`, `MAX_SPEED_OFFSET`, and `DEADZONE` at the top of `camx_tripod.ino` for your
servos and desired responsiveness.

`NEUTRAL` (default 90) is the pulse value that stops your specific continuous-rotation
servo. Cheap units are rarely trimmed exactly to 90 -- if a servo still creeps slowly with
no error signal applied, nudge `NEUTRAL` up or down by 1-2 until it holds still.

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

The firmware applies PID speed control in raw microseconds:
`pulse_us = NEUTRAL_US +/- clamp(KP*error + KI*integral, -MAX_SPEED_OFFSET_US, MAX_SPEED_OFFSET_US)`
per axis, so rotation speed scales with how far off-center the subject is. When `|error|`
is within `DEADZONE`, the firmware writes `NEUTRAL_US` (stop) instead of a speed offset.

## Tuning the PID

The gains at the top of `camx_tripod.ino` are **derived from a plant model**, not
hand-picked -- the header comment there carries the full derivation. The short version:

The plant is a **pure integrator**: pulse offset `u` (us) commands camera angular rate
`w = Ks*u`, and the app's normalized error is `angle / half_FOV`, so
`de/dt = -(Ks/half_FOV) * u`. For an integrator plant the **loop delay `L` alone** caps
the usable gain (via phase margin):

```
w_c = KP * Ks / half_FOV                        (loop crossover frequency)
PM  = 90deg - w_c*L*(180/pi) - ~10deg(I term)   (aim for PM ~= 50deg)
KP  = w_c * half_FOV / Ks
KI  = (w_c / 6) * KP
KD  = 0        (an integrator needs no D, and it only amplifies vision jitter)
```

Shipped values assume `Ks ~= 1.8 deg/s/us` (FS90R-class @ 5V, loaded), `half_FOV ~= 26deg`
(pan) / `33deg` (tilt), `L ~= 0.15 s`, giving `KP ~= 55`, `KI ~= 30`. To make it exact,
measure the three inputs on your rig and recompute:

1. **`Ks`** -- in the Serial Monitor send `P1600` (neutral + 100 us) and time one full
   revolution of the pan output with a stopwatch: `Ks = 360 / (t_seconds * 100)`. Repeat
   at `P1700` to check linearity, and do the same for tilt with `T1600`.
2. **`half_FOV`** -- mark two points a known distance `d` apart on a wall at a known
   range `r`, note what fraction `f` of the frame width they span:
   `half_FOV = atan((d/2) / r) / f`, in degrees.
3. **`L`** -- enable CSV logging in the app, step the subject sharply, and measure the lag
   from the `RawX` jump to the servo first moving (or use an LED flash + slow-motion
   video). Then `w_c = (pi/2 - PM_rad - 0.17) / L` and recompute `KP`, `KI` above.

If tilt visibly lags pan (gravity load on that axis), raise the shared `KP`/`KI` ~25% or
split them into per-axis constants. If the servo runs *away* from the subject instead of
recentering, the feedback sign is wrong for your mounting -- flip the `+`/`-` on that
axis's line in `updateTripod()`.

`PAN_NEUTRAL_US` / `TILT_NEUTRAL_US` (default 1500) is the pulse width, in microseconds,
that stops that specific continuous-rotation servo. The firmware drives the servos with
`writeMicroseconds()` rather than the 0-180 `write()` overload on purpose: `write(90)`
against the 500-2400 us attach range emits 1450 us, not 1500, and that standing ~50 us
bias is enough to make an otherwise well-centered servo creep one direction forever.

If a servo still creeps with no error signal applied, or only ever spins one way no matter
which direction the subject moves, its neutral is off. Open the Serial Monitor (115200
baud) and send `P<us>` or `T<us>` (e.g. `P1495`) to write a raw pulse width to the pan or
tilt servo directly; find the value where it holds still and set `PAN_NEUTRAL_US` /
`TILT_NEUTRAL_US` to match. Pan and tilt are separate physical units and usually need
slightly different values.

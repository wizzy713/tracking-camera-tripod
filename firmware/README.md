# CamX ESP32 Firmware

This folder contains the ESP32 firmware for the automated tracking tripod hardware.

## Features
- WiFi and UDP support for low-latency coordinate receiving.
- PID speed control for continuous-rotation (360-degree) servos, which have no absolute
  position -- the firmware commands a speed/direction each update rather than an angle to
  move to and hold, and explicitly stops when the subject is centered. Gains are
  live-tunable over Serial; see "Tuning the PID" below.
- Loss-of-signal failsafe: stops both motors if no UDP packet arrives for 500ms, so a
  dropped connection or closed app can't leave a continuous-rotation servo spinning forever.
- Divergence guard: stops the motors if the tracking error stays pinned at the frame edge
  (usually a wrong `PAN_DIR`/`TILT_DIR` sign) instead of spinning at full speed.
- Battery monitoring via an INA219 on the high side of the 2S Li-ion pack (before the
  regulator). A blended coulomb-count + open-circuit-voltage fuel gauge produces a
  state-of-charge percentage, reported back to the app as a UDP reply. See
  "Battery monitoring" below.
- Onboard LED: solid green at boot as a power/alive indicator, then it tracks battery
  charge once the gauge is running (green > 50%, amber 20-50%, red < 20%, blinking red
  < 10%). RGB LED on GPIO8 on the ESP32-C6 DevKit.

## Hardware Requirements
- ESP32-C6 Microcontroller (e.g. ESP32-C6-DevKitC-1 / DevKitM-1, WiFi 6)
- 2x Continuous-rotation ("360-degree") Servo Motors (Pan and Tilt) -- standard positional
  (0-180-degree) servos are NOT compatible with the current control scheme, since they can
  only move to and hold an angle rather than spin at a commanded speed.
- Power: a 2S Li-ion pack (2x 3.7V nominal, 2600mAh / 9.62Wh) into a 5V/3A regulator that
  feeds the servos and the ESP32. Do not power servos from ESP32 pins.
- **Common ground is required**: tie the regulator's GND, each servo's GND wire, and
  the ESP32's GND pin together. Without a shared ground, the PWM signal has no valid
  reference against the servo's own power rail and the servo will not respond correctly.
- INA219 current/voltage sensor (I2C breakout, e.g. Adafruit #904 or a clone) for battery
  monitoring, wired on the **high side of the pack, before the regulator**.

## Wiring Diagram (Default)
- **Pan Servo (X-Axis)**: Signal to GPIO 2
- **Tilt Servo (Y-Axis)**: Signal to GPIO 3
- **GND**: Connect ESP32 Ground and Servo Ground together.
- **INA219**: `VCC` -> ESP32 `3V3`, `GND` -> common ground, `SDA` -> GPIO 6, `SCL` -> GPIO 7
  (default I2C address `0x40`). Power path through the shunt:
  ```
  pack + --> [IN+] shunt [IN-] --> regulator IN (Vin)
  pack - --> common ground
  ```
  So `IN+` is the battery-positive terminal and `IN-` goes to the regulator. If the app
  shows a negative current while the rig is clearly running, the shunt is reversed --
  swap `IN+`/`IN-` or set `INA_CURRENT_SIGN = -1`.

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
4. Install the **Adafruit INA219** library via Library Manager (it will offer to pull in its **Adafruit BusIO** dependency — accept).
5. Open `camx_tripod.ino` in this folder.
6. Update `ssid` and `password` with your WiFi credentials.
7. Click **Upload**.

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

### Battery telemetry (firmware -> app)

Roughly every 2s, and only after it has received at least one `EX/EY` packet, the firmware
replies **on the same UDP flow** (back to the app's source address/port) with:

```
BATT:<percent 0-100>,MV:<pack millivolts>,MA:<pack milliamps, + = discharge>,WH:<Wh remaining>
```

e.g. `BATT:78,MV:7810,MA:640,WH:7.50`. The app reads this on the socket it sends from, so
no extra listening port is needed. Absent an INA219, the firmware just never sends these
and the app's battery indicator stays hidden.

## Battery monitoring

The INA219 sits before the regulator, so it reads the raw 2S pack: bus voltage
(~6.0-8.4V) and total current (regulator + servos + ESP32). The fuel gauge in
`camx_tripod.ino` blends two estimates every `BATT_SAMPLE_INTERVAL_MS`:

1. **Coulomb counting** -- integrate current out of `battChargeMah` (starts from the
   boot open-circuit voltage). Precise short-term; drifts over hours.
2. **Voltage / OCV** -- `OCV ~= V_terminal + I_discharge * BATT_IR_OHMS`, run through a
   per-cell Li-ion resting-voltage curve (`cellOcvToSoC`, evaluated at `OCV / BATT_CELLS`).
   Absolute but noisy under load and flat through the mid-charge plateau.

Fusion: near rest (`|I| < BATT_REST_CURRENT_MA`) the terminal voltage is ~OCV, so the
counter is pulled toward the voltage estimate (converges in ~30s). Under load the voltage
estimate can only ever drag the counter *down* -- so a nearly-flat pack can't hide behind
a stale count, but a servo-surge voltage sag can't make the gauge jump either.

Constants to check for your pack/wiring (top of `camx_tripod.ino`):

| Constant | Meaning | If it's wrong |
|---|---|---|
| `BATT_CAPACITY_MAH` / `BATT_ENERGY_WH` | pack rating (2600 mAh / 9.62 Wh) | % scales wrong |
| `BATT_CELLS` | cells in series (2) | OCV curve reads the wrong cell voltage |
| `BATT_IR_OHMS` | whole-pack internal resistance (~0.15 Ohm est.) | gauge drifts up/down under load |
| `INA_CURRENT_SIGN` | shunt orientation (+1) | current/telemetry sign flipped, gauge counts backwards |
| `INA_SDA_PIN` / `INA_SCL_PIN` | I2C pins (GPIO 6 / 7) | `INA219 NOT found` at boot |

`cellOcvToSoC` is a generic 18650 curve; for a thesis-grade number, discharge your actual
pack at a constant current, log `MV` from the telemetry, and refit the table. Send `?` over
Serial (115200) at any time to print live `%`, voltage, current, Wh and mAh.

Note: the default INA219 calibration tops out near 3.2A. Servo-stall spikes above that read
clipped -- they're too brief to matter to the coulomb count, but don't rely on `MA` for
peak-current measurements.

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

The model with `Ks ~= 1.8 deg/s/us` (FS90R-class @ 5V, loaded), `half_FOV ~= 26deg` (pan) /
`33deg` (tilt), `L ~= 0.15 s` gives `KP ~= 55`, `KI ~= 30`. That felt sluggish on the
bench (the `L` estimate is pessimistic -- it double-counts delay the app-side Kalman
look-ahead already removes), so the **shipped defaults are `KP = 85`, `KI = 50`,
`MAX_SPEED_OFFSET_US = 140`**, roughly `w_c ~= 5-6 rad/s`.

### Live tuning over Serial

`KP`, `KI`, `KD`, and `MAX_SPEED_OFFSET_US` apply immediately from the Serial Monitor
(115200 baud), no reflash -- so tune on the running rig:

```
KP120     set KP = 120
KI70      set KI = 70   (also zeroes the integrators)
KD0       set KD
MS160     set MAX_SPEED_OFFSET_US = 160
?         print current values
```

Method: raise `KP` until the camera just starts to overshoot or hunt around the subject,
then back off ~30%. Set `KI` to about `KP/1.5` and lower it if you see slow oscillation.
Leave `KD` at 0. Then copy the values you settled on back into `camx_tripod.ino`.

To make the gains exact from first principles instead, measure the three model inputs and
recompute with the formulas above:

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
split them into per-axis constants.

## Feedback direction (do this before the first app run)

The pulse written per axis is `NEUTRAL_US + DIR * offset`, where `PAN_DIR` / `TILT_DIR`
(`+1` or `-1`, top of `camx_tripod.ino`) set which way each servo turns for a given error.
This depends entirely on your servo wiring and how the pan/tilt head is assembled, and a
**wrong sign makes the loop diverge** -- the servo drives the subject further off-centre
until it is spinning at full speed. Set it deliberately:

1. Serial-send `P1560` (neutral + 60). Note which way the camera pans.
2. A subject to the **right** of frame centre must make the camera pan **right** (toward
   it). If `P1560` panned right, `PAN_DIR = +1`; if it panned left, `PAN_DIR = -1`.
3. Same for tilt with `T1560`: a subject **below** centre needs the camera to tilt **down**.

As a backstop, if `|error|` stays pinned at the frame edge for `SATURATION_TIMEOUT_MS`
(1.5 s) the firmware stops both motors and prints `Control DIVERGING` rather than spinning
forever -- but treat that as "the sign is still wrong," not a fix.

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

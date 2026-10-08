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

That PID output is a *target*: the pulse actually written ramps toward it at no more than
`MAX_SLEW_US_PER_S` (default 1000 us/s), so speed changes and direction reversals are
gradual instead of instant. The failsafe and divergence-guard stops skip the ramp.

### Manual joystick drive: `JOY` (app -> firmware)
With "Manual joystick" switched on in the app's Advanced Settings, the app sends
`JOY:x,y,SEQ:value` at about 30 Hz instead of `EX`/`EY`. `x` and `y` are the stick deflection
in `[-1, 1]`, with the same sign convention as the tracking error (positive `x` pans the way
a subject right of centre would, positive `y` tilts the way a subject below centre would).

This is open loop: `updateManual()` bypasses the PID and the divergence guard and maps
deflection straight to a speed, `offset_us = +/-(MIN_OFFSET_US + |v| * JOY_SPEED_RANGE_US)`,
so the stick starts at the edge of the servo dead band. Inside `JOY_DEADZONE` (0.08) the
axis stops. The slew limit and the loss-of-signal failsafe apply as for tracking, and `SEQ`
shares the tracking packets' counter.

### Power-safety limits and disconnect diagnosis

The servos share the 5 V regulator with the ESP32-C6. A servo slammed from full speed one
way to full speed the other is close to a stall and pulls a large current spike. Very high
gains do exactly that on noisy vision error, and the rail sag can brown out the C6 or drop
its WiFi. The LED stays lit through this, because `setup()` turns it on right after the
reboot. The firmware guards against it in two ways:

- **Slew limit** (`MAX_SLEW_US_PER_S`, see above).
- **Gain caps**, applied to both Serial and `CFG` updates: `KP <= 200`, `KI <= 100`,
  `KD <= 20`, `10 <= MS <= 400`, `DZ <= 0.5`. Out-of-range values are clamped, and the `CFG`
  reply reports the clamped values. The Experiment tab's sliders use the same maxima.

To diagnose a disconnect:

- At boot the Serial log prints `Reset reason: ...`. After a **brownout**, the LED also
  blinks red three times. If you have no Serial connection, press "Sync from Tripod" after
  a disconnect: gains back at the compiled defaults mean the board rebooted.
- WiFi drops are logged with the reason code, e.g.
  `WiFi DISCONNECTED, reason 200 (BEACON_TIMEOUT), RSSI was -78 dBm`. Modem power-save
  is off (`WiFi.setSleep(false)`). If the core's auto-reconnect hasn't recovered after
  `WIFI_RECONNECT_INTERVAL_MS` (10 s), the firmware forces a reconnect.
- Brownouts are a hardware problem that the limits only mask. Put 470-1000 uF across the
  servo 5 V rail near the servos, or power the servos from their own regulator with a
  common ground.

### CFG protocol: live PID tuning from the app's Experiment tab (app <-> firmware)

Same `KEY:value` convention as `EX`/`EY` and `BATT`, on the same UDP flow:

- **Set**, app -> firmware: `CFG:KP:18.00,KI:0.00,KD:0.00,MS:300.0,DZ:0.030,MO:52.0` -- the values together
  (`TD:<1|-1>` tilt direction and `SL:<v>` pulse ramp limit are optional extras) (the app always sends its full current set). Applying a set also zeroes
  `panIntegral`/`tiltIntegral`, same as the Serial `KI<v>` handler.
- **Query**, app -> firmware: `CFG?` -- returns the current gains without changing anything
  (used by the Experiment tab's "Sync from Tripod").
- **Reply**, firmware -> app: same format as the set message, sent immediately (not
  rate-limited like `BATT`) after handling either a set or a query, so the app's sliders
  always reflect what the firmware actually has -- including gains changed over Serial.

Gains set this way are RAM-only, same as the Serial path below -- they reset to the
defaults in `camx_tripod.ino` on reboot.

### Discovery: auto-connect (app <-> firmware)

- **Probe**, app -> broadcast: `DISCOVER`, sent to 255.255.255.255 and each network's
  directed broadcast (e.g. 10.47.140.255) on the tripod port, every 2 s while the app has
  heard nothing from its current tripod address for 5 s.
- **Announce**, firmware -> app: `TRIPOD:<tripod IP>`, sent straight back to the probing
  socket. The app adopts the reply's *source* address, saves it (SharedPreferences), and
  shows a "Tripod found at ..." toast. `DISCOVER` is handled per packet in the receive
  loop, so it's never dropped behind tracking packets.

This is request/reply rather than the tripod broadcasting unprompted because Android
(Samsung especially) filters incoming broadcasts unless the app holds a multicast lock,
while a unicast reply to a socket the app sent from always arrives.

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

The gains at the top of `camx_tripod.ino` were **measured on the rig** (pendulum
calibration of 2026-10-06; see `Tests/calibration.m` and the header comment in the
sketch). The first set (`KP = 110`, `KI = 50`, `KD = 5`) came from a model that assumed a
linear servo and a 0.15 s loop delay. Neither held:

- the servos have a dead band of about 52 us either side of neutral, and beyond it the
  whole useful speed range is only about 8 us wide;
- the loop delay was 0.42 s, of which 0.14 s was the app's Kalman filter, about 0.11 s the
  pulse ramp across the dead band, and about 0.17 s camera, network and servo.

The **shipped defaults are `KP = 18`, `KI = 0`, `KD = 0`, `MIN_OFFSET_US = 52`,
`MAX_SPEED_OFFSET_US = 300`**, used with the app's filter at `R = 5`, `sigma_a = 300` and a
prediction horizon of 0.20 s. `KI` and `KD` were not needed on the pendulum and have not
been tuned.

### Servo dead-band compensation (`MIN_OFFSET_US`)

The MG996R continuous-rotation servos do not move for pulses within about 52 us of
neutral. With a plain PID that makes low gains do nothing at all, and the first gain that
does move the camera is already high enough to hunt. Outside `DEADZONE`, the firmware
therefore adds `MIN_OFFSET_US` to the PID output in the direction of the error, so the
command starts at the edge of the dead band. `MO0` restores the plain PID.

To measure the dead band and the servo speed on a rig, hang a ball still in view and send
`KP0`, `DZ0` and `MO<v>`: the output is then a fixed `+-v` us, and the camera rocks across
the ball. The slope of the logged offset is the camera speed at that pulse, and the time
from the ball crossing centre to the camera reversing is the loop delay. Measured values:
52 us gives 0.06 / 0.17 half-frames/s (the two directions), 60 us gives about 1.2.

### Live tuning over Serial (or the app's Experiment tab)

`KP`, `KI`, `KD`, `MAX_SPEED_OFFSET_US`, and `DEADZONE` apply immediately from the Serial
Monitor (115200 baud), no reflash -- so tune on the running rig:

```
KP120     set KP = 120   (KP/KI/KD/MS/DZ are clamped to the power-safety caps above)
KI70      set KI = 70   (also zeroes the integrators)
KD0       set KD
MS160     set MAX_SPEED_OFFSET_US = 160
DZ0.05    set DEADZONE = 0.05
MO52      set MIN_OFFSET_US = 52   (servo dead-band compensation, 0..80, 0 = off)
TD-1      set TILT_DIR (1 or -1)
SL3000    set MAX_SLEW_US_PER_S (500..20000)
?         print current values
```

The same five gains are also live-tunable from the phone, over UDP, via the Experiment
tab -- see "CFG protocol" above. Either path updates the same in-memory values, and either
one's changes show up in the other (`?` over Serial, or "Sync from Tripod" in the app).

Method that worked on this rig: set `MIN_OFFSET_US` to the measured dead-band edge, cut
the app's filter lag (`R = 5`), start `KP` low (about 18) and add prediction horizon until
the share of wrong-direction commands in the log approaches 10%. Then copy the values you
settled on back into `camx_tripod.ino`.

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

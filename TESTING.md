# CamX Suggested Test Protocol

This is a practical evaluation protocol for tuning and characterizing CamX's tracking
loop (Kalman filter + PID), using the Experiment tab (`ExperimentScreen.kt`) and the
CSV logging it drives. See `ARCHITECTURE.md` for the underlying algorithms and the
`ErrX`/`ErrY` CSV schema, and `firmware/README.md` for the PID model and the `CFG`
live-tuning protocol.

Every test below follows the same shape:
1. Open the Experiment tab, set/sync the gains you're testing, fill in a **test name**
   and **notes** describing the run (rig geometry, subject, anything that isn't a gain
   value), and set the **detection mode** the test needs.
2. Start logging, run the physical motion, stop logging.
3. Pull the CSV from `Documents/CamX_Logs` and analyze `ErrX`/`ErrY` (the tracking error
   actually sent to the tripod that frame) -- RMS error, peak error, and where in the
   motion the error is worst are the main things to look at across all tests below.

## 1. Bench safety / setup

- Mount the tripod securely; the pan/tilt head is mechanically limited to 0-180 degrees
  but the continuous-rotation servos have no position feedback, so a wrong `PAN_DIR`/
  `TILT_DIR` or a genuinely diverging loop will drive the servo at full speed until the
  firmware's `SATURATION_*` guard trips (~1.5 s) or the signal-loss failsafe does
  (500 ms with no packets). Stand clear of the swing radius for anything below, and keep
  the Serial monitor or the Experiment tab's "Sync" handy to confirm gains before pushing
  `KP`/`KI` up.
- See `ARCHITECTURE.md`'s "Ethical Considerations" before recording anyone but yourself.

## 2. Step-response test (tune KP/KI first)

Do this before anything else -- the pendulum and constant-velocity tests below are only
meaningful once the loop isn't hunting or sluggish.

- Subject (face or object, matching your current `DetectionMode`) steps a known distance
  sideways from center and holds still.
- In the Experiment tab, raise `KP` (in small steps, "Apply to Tripod" each time) until
  you see the first sign of overshoot/hunting in the live `ErrX`/`ErrY` trace (or just by
  watching the tripod), then back off ~30%, per `firmware/README.md`'s existing guidance
  -- now doable per-run from the phone instead of round-tripping to the Serial monitor.
  Then set `KI ~= KP / 1.5` and back it off if you see slow oscillation. Leave `KD` at 0
  unless overshoot remains after `KP`/`KI` are set.

## 3. Pendulum test (swinging subject, varied height/velocity)

The core test the Experiment tab's PID/Kalman tuning is meant to support: a mechanically
repeatable motion profile with a wide, controllable range of velocity and acceleration,
which a person walking or a hand-held subject can't reliably reproduce run-to-run.

**Rig**: a ball (or any plain, visually simple object with good contrast against the
background -- ML Kit's generic object detector works best against an uncluttered scene,
not a specific shape) on a string or rigid rod, swinging in the plane the camera sees.
Switch **Detection mode -> Object** in the Experiment tab first, since a ball has no face.

**Sweep two independent parameters**:
- **String/rod length** `L` (e.g. 0.3 m / 0.6 m / 1.0 m) -- changes the period
  `T = 2*pi*sqrt(L/g)` and, for a fixed release angle, the peak (bottom-of-swing) velocity.
- **Release angle/amplitude** at a fixed length -- changes peak velocity independently of
  period (larger swing = faster bottom-of-swing crossing, same period for small-angle
  approximation).

Run one logged test per `(L, angle)` combination, naming each in the Experiment tab (e.g.
`pendulum_L0.6_theta30`) so the CSV's metadata header records exactly which run it is,
alongside whatever gains were in effect.

**Analysis**: use `ErrX`/`ErrY` directly as the tracking-error signal.
- Compare RMS and peak error **at the bottom of the swing** (maximum velocity, ~zero
  acceleration -- stresses the PID's speed limit and `MAX_SPEED_OFFSET_US`) against error
  **near the turnarounds** (near-zero velocity, maximum acceleration / direction reversal
  -- stresses the Kalman filter's constant-velocity assumption, which is exactly where a
  constant-velocity model should lag most).
- This is also the right rig for sweeping `predictionHorizonSeconds` and the Kalman noise
  constants (`measurementNoise`/`accelerationNoise`) against a *repeatable* motion: re-run
  the same physical swing at a few different Experiment-tab settings and compare `ErrX`/
  `ErrY` RMS between runs, rather than re-deriving predictions offline from a single log
  (`VelocityX`/`VelocityY` are still logged too, so an offline horizon sweep from one
  recording, per `ARCHITECTURE.md`, is still possible if you'd rather not re-run the rig).

## 4. Constant-velocity pass test

Walk/carry the subject across the frame at a roughly constant speed. This is a rough
cross-check against the pendulum's near-bottom (locally constant-velocity) segment, using
a motion that's easier to set up but much less repeatable/controllable -- prefer the
pendulum for anything you need to compare across gain settings.

## 5. Occlusion / coast test

Briefly hide the subject (hand over the lens, walk behind an obstruction) while locked.
Confirm in the CSV that `RawX`/`RawY` go `NaN` while `FilteredX`/`FilteredY` keep coasting
on the last Kalman estimate, for up to `MAX_COAST_FRAMES` (15 frames) before the lock
releases -- see `ARCHITECTURE.md`'s "Target loss handling".

## 6. Rest / deadzone jitter test

Stationary subject, nothing moving. Confirm `ErrX`/`ErrY` stay inside `DEADZONE` and the
servos stay stopped (no audible buzzing/hunting at rest). If they don't, `DEADZONE` is too
tight for the current Kalman jitter level -- widen it slightly in the Experiment tab.

## 7. End-to-end latency measurement

The existing `ARCHITECTURE.md` TODO: replace the placeholder prediction horizon with a
measured one. Either an LED-flash + slow-motion-video measurement, or the sharp-step
method described in `firmware/README.md`'s "Tuning the PID" (step 3: log a sharp subject
step, measure the lag from the `RawX` jump to the servo first moving). Once you have a
number, set it directly via the Experiment tab's prediction-horizon slider -- no code
change or rebuild needed to try it.

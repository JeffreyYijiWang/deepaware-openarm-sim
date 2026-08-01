# Simulation safety subsystem

This subsystem is an explicit guard around the seven selected MuJoCo channels.
It is **simulation safety only**. It is not a certified hardware safety system,
does not establish a safety integrity level, and cannot remove motor power. A
real arm requires an independent, physically accessible E-stop and a
power-isolation path that does not depend on this Python process, CAN traffic,
or the motor controller firmware.

## Three limit classes

1. **Planning limits** validate desired position, desired velocity, and maximum
   desired-position change per control sample. They are deliberately inside the
   mechanical joint limits and are never used as actuator torque limits.
2. **Normal command limits** clip ordinary requested torque and limit torque
   slew. Clipping is non-faulting, but normal saturation and rate limiting are
   logged separately and included in the aggregate saturation signal.
3. **Absolute fault limits** reject hard position, hard velocity, absolute
   requested torque, non-finite applied torque, and persistent tracking error.
   They are fault thresholds, not normal operating targets.

All configurable values and provenance classifications are in
`config/openarm_limits.yaml`. The selected torque-rate, trajectory velocity,
discontinuity, persistence, watchdog, timing-tolerance, and zero-torque fault
policies are `ASSUMED` project choices, not OpenArm hardware specifications.

## Fault and watchdog behavior

`SafetyMonitor` starts in `RUNNING`. Every seven-joint input must have shape
`(7,)` and finite values. The configured timestep must be positive, simulation
time must advance by the nominal fixed step within tolerance, command and
feedback timestamps must increase, and neither source may be missing, future
dated, or stale.

The first fault is latched as a `SafetyEvent` containing its reason, simulation
timestamp, and detail. The active-trajectory flag becomes false. The experiment
stops iterating, writes zero torque to the seven selected simulated actuators,
updates MuJoCo, emits a `<prefix>_fault.json` record, and returns a nonzero CLI
status. Zero simulated actuator torque is a deterministic failure response; it
is not a hardware safe-torque-off function and does not guarantee that a
gravity-loaded physical arm will remain stationary.

Tracking error must exceed the configured per-joint threshold continuously for
25 samples (0.05 s at 500 Hz) before faulting. One transient sample resets on
the next healthy sample. A reversed controller sign produces sustained error
and therefore reaches the same persistent-divergence fault.

The name-to-index mapping is also fingerprinted. A recomputed joint ID, qpos
address, DoF address, or actuator ID that differs from startup produces a
`mapping_drift` fault instead of silently controlling another channel.

## Why PD plus bias compensation, not PID

MuJoCo bias-force compensation analytically removes the modeled gravity,
Coriolis, and centrifugal terms instead of asking integral feedback to build up
an opposing torque. Omitting the I term also avoids windup interactions with
normal torque saturation, torque-rate limiting, and persistent tracking-
divergence faults.

On real hardware the dynamics parameters are only approximately known. After
the independent hardware safety chain and saturation behavior are validated, a
small anti-windup integral term would likely be appropriate to correct residual
model error. MuJoCo `qfrc_bias` also excludes the model's joint damping and
`frictionloss`, so a small simulated steady-state residual is expected here.

## Verification

The deterministic tests intentionally violate vector shape, finite-value,
timing, timestamp, desired trajectory, hard state, torque, torque-rate,
tracking persistence, watchdog, applied-force, and mapping-stability rules.
They also prove valid endpoints, zero endpoint velocity, valid in-limit paths,
positive and negative normal clipping, one-sample tracking recovery, latched
FAULT state, trajectory stop, zero-torque response, and timestamped reason
logging.

Run:

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

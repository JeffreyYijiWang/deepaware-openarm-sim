# Controlled latency and sensor-noise perturbation

This is a controlled robustness test, not a claim that the injected values
reproduce a physical OpenArm. The real encoder noise, velocity-estimation noise,
CAN timing, drive processing delay, and mechanical response latency remain
unmeasured.

## Exact perturbation semantics

Baseline and perturbation modes use the same official model, initial state,
quintic trajectory, controller gains, 2 ms simulation step, safety policy, and
in-memory torque-actuator adaptation. Baseline creates no random generator,
injects no noise, and bypasses the latency FIFO.

For `latency_noise`:

- Queue length is derived, never hardcoded:
  `0.008 s / 0.002 s = 4 control samples`.
- Implemented latency is verified as `4 * 0.002 s = 0.008 s = 8 ms`.
- The FIFO receives the safety-constrained torque command. Its output is written
  to the seven selected MuJoCo actuator controls. Desired position is not
  delayed.
- The FIFO starts with four zero-torque samples. A command issued at time `t`
  reaches the actuator at `t + 8 ms`.
- Independent zero-mean Gaussian noise is added to each joint-position and
  joint-velocity measurement using NumPy's deterministic generator and seed
  `42`. Configured standard deviations are `0.001 rad` and `0.01 rad/s`.
- Sensor measurements are **noisy but not delayed**. `measurement_delay_ms` is
  explicitly recorded as zero. The noisy values feed PD and the state safety
  checks.
- Tracking metrics use the true MuJoCo joint state. CSV columns `q_sensor` and
  `dq_sensor` preserve the noisy controller inputs separately from `q` and `dq`.
- MuJoCo `qfrc_bias` is evaluated from the simulator's true internal state; the
  test perturbs feedback measurements, not the simulator's dynamics estimate.

`tau_requested` is the PD-plus-bias request, `tau_command` is the constrained
command entering the FIFO, and `tau_applied` is actuator-force feedback after
the FIFO. Normal torque saturation and torque-rate limiting have separate CSV
and JSON metrics.

## Threshold rationale

All thresholds below are `ASSUMED` engineering policies recorded with units and
notes in `config/openarm_limits.yaml`. They avoid exact floating-point golden
values while catching meaningful regressions.

| Threshold | Value | Reasoning |
| --- | ---: | --- |
| Short trajectory / hold | `2.0 / 0.5 s` | Exercises all joints, the four-sample FIFO, and 250 settling samples without writing ordinary-test artifacts. |
| Overall RMS error | `< 0.10 rad` | Below half the roughly 0.2 rad commanded span; a lost tracker fails clearly. |
| Maximum error | `< 0.25 rad` | Allows expected lag but remains far below the `0.75 rad` persistent-divergence fault. |
| Maximum final error | `< 0.10 rad` | Requires clear convergence while allowing friction, delay, and assumed noise. |
| Maximum velocity | `< 1.0 rad/s` | More than ten times the full-run motion, yet below the smallest hard velocity limit. |
| Aggregate saturation | `< 20%` | Allows brief slew intervention but rejects sustained controller struggle. |
| Repeat RMS tolerance | `< 1e-9 rad` | Allows negligible arithmetic variation while detecting seed, queue, or dynamics changes. |
| Material RMS increase | ratio `> 1.25` and delta `> 0.001 rad` | Requires both meaningful relative and absolute degradation. |
| Oscillation | `> 2` added significant error crossings and hold velocity `> 0.05 rad/s` | Avoids calling noise-scale crossings oscillation without residual motion. Error crossings use a `0.001 rad` deadband. |
| Material overshoot increase | `> 0.005 rad` | Ignores numerical-scale endpoint crossing. |

The regression also requires completion, finite output, zero safety faults,
positive position/velocity margins, and requested torques below each joint's
absolute limit. A separate stale-command injection must latch `FAULT`, stop the
active trajectory, record `stale_command` and its timestamp, and return the
configured zero simulated torque.

## Measuring real values

- **Position noise:** support and disable motion, log raw encoder position for
  long stationary intervals at several joint angles and temperatures, remove
  slow drift, then report quantization, robust standard deviation, peak-to-peak
  spread, and spectrum.
- **Velocity noise:** repeat the stationary test, then drive one supported joint
  at several low constant speeds in both directions. Compare reported velocity
  with a high-resolution external encoder and fit bias, scale, standard
  deviation, and bandwidth.
- **Communication and actuation latency:** synchronize host/CAN timestamps with
  a logic analyzer, command small bounded torque steps or a safe binary sequence
  on one supported midrange joint, and measure CAN transmission, motor reply,
  and encoder/accelerometer onset. Report median, p95, p99, and jitter rather
  than substituting the assumed 8 ms.

Use low torque and speed, support gravity loads, keep generous joint margins,
clear the workspace, and keep an independent physical E-stop within reach. The
full identification plan is in `docs/PARAMETER_PROVENANCE.md`.

## Reproduce

```powershell
.\.venv\Scripts\python.exe -m src.experiment --mode compare --headless --output results
```

This produces the baseline, perturbation, and comparison artifacts in one
deterministic command.

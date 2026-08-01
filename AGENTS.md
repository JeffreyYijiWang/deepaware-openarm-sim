# AGENTS.md

## Repository purpose

This repository is a small, reproducible OpenArm v2 MuJoCo take-home project.
Its primary deliverable is a defensible, deterministic, headless-first Python
simulation of joint-space trajectory tracking for the **left seven-DoF arm
only**, with explicit safety checks, measured results, regression tests, a
hardware/CAN bridge design, and concise engineering documentation.

The time budget is intentionally small (approximately 1-2 development hours),
so prefer the smallest correct implementation over a broad architecture.
Depth, evidence, reproducibility, and verifiability matter more than feature
count.

## Allowed stack

- Python 3.11
- MuJoCo through the official `mujoco` Python package
- Official `enactic/openarm_mujoco` OpenArm v2 assets
- NumPy, PyYAML, pandas, matplotlib, pytest, and ruff
- Standard-library modules
- A small C++17/Linux SocketCAN design artifact for the later hardware bridge

Keep robot configuration separate from controller code. Keep headless execution
as the primary path; a MuJoCo viewer may be optional.

## Disallowed scope and dependencies

Do not add:

- ROS or ROS 2
- Isaac Lab
- reinforcement learning
- inverse kinematics
- Cartesian impedance control
- object manipulation
- gripper control
- right-arm control
- a custom GUI
- unnecessary abstraction layers or dependencies

Do not modify official/vendor OpenArm files. A project-owned runtime adaptation
or derived asset is allowed only when its need, exact behavior, and provenance
are explicit and tested.

## Target repository structure

```text
.
|-- README.md
|-- requirements.txt
|-- requirements-lock.txt
|-- AGENTS.md
|-- config/
|   `-- openarm_limits.yaml
|-- src/
|   |-- __init__.py
|   |-- model_mapping.py
|   |-- trajectory.py
|   |-- controller.py
|   |-- safety.py
|   |-- experiment.py
|   |-- metrics.py
|   `-- record_demo.py
|-- tests/
|   |-- test_model_mapping.py
|   |-- test_trajectory.py
|   |-- test_safety.py
|   `-- test_tracking.py
|-- hardware/
|   |-- can_control_loop.cpp
|   `-- HARDWARE_BRIDGE.md
|-- docs/
|   |-- PARAMETER_PROVENANCE.md
|   |-- AI_WORKFLOW.md
|   `-- SIMULATION_IN_CI.md
`-- results/
    `-- generated experiment outputs
```

Create only the files needed by the current increment. Do not create empty
architecture for its own sake.

## Engineering rules

1. Inspect before editing.
2. Do not invent physical parameters silently.
3. Label important physical and experiment parameters `SOURCED`, `DERIVED`,
   `MODEL_ESTIMATE`, `ASSUMED`, or `MISSING`.
4. Never assume joint ID, qpos index, DoF/qvel index, and actuator index are
   interchangeable.
5. Resolve joint and actuator mappings once at startup using exact names and
   MuJoCo named access/metadata. Do not hardcode raw state/control offsets in
   controller or safety code.
6. Validate that selected joints, qpos addresses, DoF addresses, actuators, and
   transmissions are unique and correct.
7. Do not modify vendor OpenArm files unless absolutely necessary; prefer a
   documented project-owned in-memory adaptation.
8. Keep robot/model configuration separate from control-law implementation.
9. Run headless by default.
10. Use a fixed random seed for stochastic experiments.
11. Run commands and tests; do not claim success from inspection alone.
12. Record assumptions, commands, failures, fixes, unresolved uncertainty, and
    measured output.
13. Every safety rule needs at least one negative test proving detection.
14. Every factual README claim must be supported by source evidence or measured
    output.
15. Continue fixing failures until the current acceptance criteria pass or a
    genuine environment blocker is demonstrated.
16. Preserve unrelated user changes in a dirty worktree.
17. A safety fault must stop stepping and cause a nonzero process exit.

## Approved evidence and major decisions to preserve

The provenance investigation is recorded in `docs/PARAMETER_PROVENANCE.md` and
`config/openarm_limits.yaml`. Its commit-pinned official sources are the source
of truth for implementation.

### Model and selection

- Primary headless model: official OpenArm MuJoCo v2
  `v2/openarm_bimanual.xml` from `openarm-mujoco==2.0.1`.
- Optional viewing scene: `v2/cell.xml`; do not use its indices for headless
  experiment mappings.
- There is no official left-only v2 MJCF. Load the bimanual asset and select
  only these exact joints:
  `openarm_left_joint1` through `openarm_left_joint7`.
- Select only these exact upstream actuators:
  `left_joint1_ctrl` through `left_joint7_ctrl`.
- The standalone model was compiled and measured as `nq=18`, `nv=18`, `nu=16`,
  timestep `0.002 s`, gravity `[0, 0, -9.81] m/s^2`. These counts are
  validation evidence, not permission to hardcode offsets.

### Critical actuator-interface decision

The official v2 arm actuators are MuJoCo `<position>` actuators. Therefore the
vendor-model `data.ctrl` values are position references, **not direct joint
torques**, even though gear is one and force ranges match peak motor torques.
A torque PD controller must not write torque into the unmodified position-servo
interface.

For this project, use a documented, project-owned **in-memory actuator
adaptation** after model load:

- resolve the seven actuators and target joints by exact name;
- verify each is a joint transmission to the expected unique joint with scalar
  gear `1`;
- change only the selected compiled-model actuator dynamics to fixed gain `1`
  and no bias, so `ctrl` represents joint torque;
- replace the selected control ranges with conservative software torque ranges;
- keep the existing actuator/joint peak force limits as hard upper bounds;
- assert after adaptation that the built-in position feedback/bias is absent;
- verify the torque sign for every selected joint before the tracking run.

Do not edit the vendor XML. Do not use `qfrc_applied` for the normal controller,
because it bypasses the selected actuator interface and its model force clamps.

### Dynamics and controller decision

Use joint-space PD feedback plus the corresponding selected entries from
MuJoCo `data.qfrc_bias`:

```text
tau_feedback = kp * (q_desired - q_measured)
             + kd * (dq_desired - dq_measured)
tau_requested = tau_feedback + qfrc_bias[selected_dof_addresses]
```

Resolve the `qfrc_bias` entries with the stored DoF mapping. MuJoCo bias force
contains gravity, Coriolis, and centrifugal terms. It does **not** contain joint
damping or `frictionloss`. Small residual steady-state error is therefore
expected when those model terms are nonzero. Distinguish this from mapping,
sign, or gain errors. Do not raise gains aggressively merely to suppress
friction-induced error; integral action or explicit friction feedforward is the
appropriate later remedy.

PD plus bias compensation was selected over PID because analytic bias
compensation removes modeled gravity/Coriolis error without integral buildup,
and omitting I avoids windup interaction with torque saturation and tracking
divergence faults. Real hardware may eventually warrant a small anti-windup
integral term for residual model error.

### Limits and assumptions

- URDF and MJCF position limits are consistent after official left-arm
  reflection and minor decimal rounding.
- MJCF/URDF force limits correspond to motor **peak** torque. Use the configured
  80% of **rated** torque policy for normal experiment torque caps; never use CAN
  protocol packing ranges as safety limits.
- Use an initial planning margin of `0.0872665 rad` (5 degrees) per side.
- Baseline control frequency: `500 Hz`; trajectory duration: `5 s`; these are
  `ASSUMED` experiment choices aligned with the 2 ms model step.
- Noise assumptions and the 8 ms latency case belong to a later experiment;
  keep seed `42` deterministic.
- Link inertials, damping, friction loss, and armature are model estimates, not
  experimentally validated physical-arm parameters.
- J7 has a source conflict: the MJCF joint inherits `DM3507` passive values while
  the actuator and official hardware mapping identify a DM4310. Preserve and
  report this uncertainty.

## Current implementation increment

Implement the smallest baseline experiment that:

- resolves and validates all seven mappings once at startup;
- starts from a valid, collision-free model configuration;
- follows a conservative in-limit quintic joint trajectory with zero endpoint
  velocity and acceleration, then holds the endpoint;
- performs finite-value, position, planning, velocity, torque, saturation, and
  tracking-divergence checks before each step;
- logs desired/actual state, feedback/bias/requested/applied torque, saturation,
  limit margins, controller state, and safety state;
- writes baseline CSV, JSON metrics, tracking plot, and torque plot;
- supports `--headless` and `--viewer`;
- exits nonzero on a safety fault;
- includes negative tests for every safety rule.

Tune only enough for stable, clear, non-oscillatory tracking. Record measured
results and update this file incrementally when a later prompt introduces a
major decision.

## Baseline implementation outcome (2026-08-01)

- The in-memory torque adaptation was implemented and startup-verified for all
  seven named actuators; vendor MJCF remains unchanged.
- Configured gains are `kp=[40,40,35,35,8,8,6]` and
  `kd=[5,5,4,4,1,1,0.8]`.
- The baseline completed 3001 samples with overall RMS position error
  `0.00526052 rad`, maximum requested/applied torque `4.29030 N m`, no torque
  saturation, no safety violations, and no non-finite samples.
- Final absolute errors are `0.000598-0.004370 rad`; the remaining residual is
  consistent with model damping/friction omitted from `qfrc_bias`.
- Regression acceptance uses greater than 50% peak-to-final error-norm reduction
  (measured 59%) rather than driving gains upward to chase friction residual.
- Ruff passes and 23 deterministic tests pass, including negative tests for all
  implemented safety faults and a nonzero CLI exit on safety fault.
- Both the headless baseline and a bounded six-second `--viewer` run exit zero.

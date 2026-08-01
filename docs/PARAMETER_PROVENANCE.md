# OpenArm v2 Parameter Provenance

Recorded 2026-08-01. This is an evidence report, not a hardware calibration
certificate. The investigation preceded controller implementation; the later
measured-baseline section records the approved implementation result. No vendor
file was modified.

## Scope and source revisions

This report covers only the left seven-degree-of-freedom arm. The gripper,
right arm, lifter, ROS, and Cartesian control are out of scope.

| Official source | Revision investigated | Role |
| --- | --- | --- |
| [enactic/openarm](https://github.com/enactic/openarm/tree/990fda921c82ae9d12b00f23e449793a9a313afd) | `990fda921c82ae9d12b00f23e449793a9a313afd` | Canonical repository index |
| [enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco/tree/ebc5cd29a957c8253887aab222ff3f7dc5907d4a) | `ebc5cd29a957c8253887aab222ff3f7dc5907d4a` | v2 MJCF and packaged launcher |
| [enactic/openarm_description](https://github.com/enactic/openarm_description/tree/6c7b720f1ba48e8bafa3a3dc752c45f397b42221) | `6c7b720f1ba48e8bafa3a3dc752c45f397b42221` | v2 topology, URDF/Xacro configuration, limits, inertials |
| [enactic/openarm_can](https://github.com/enactic/openarm_can/tree/98666042b5e9cd5b55d0bd1d7fc3aa5c42caae4d) | `98666042b5e9cd5b55d0bd1d7fc3aa5c42caae4d` | SocketCAN/DAMIAO command and state interface |

Revision IDs were obtained with `git -c http.sslBackend=schannel ls-remote
<repository-url> HEAD`. The installed artifact inspected locally was
`openarm-mujoco==2.0.1` with MuJoCo 3.11.0. The installed
`v2/openarm_bimanual.xml` Git blob hash is
`0a4e8e96550a634a81d8ac8bc74f73c125006c15`, identical to the file at the
recorded upstream revision.

Evidence labels mean:

- **SOURCED**: explicitly present in an official OpenArm source or linked motor
  documentation.
- **DERIVED**: calculated directly from sourced data or read from the compiled
  model without fitting.
- **MODEL_ESTIMATE**: present in the simulation/description but no physical-arm
  validation evidence was found.
- **ASSUMED**: an intentional project choice, not claimed as a robot fact.
- **MISSING**: required physical evidence was not found.

Confidence describes confidence in the transcription and interpretation, not
necessarily real-hardware fidelity.

## Selected model and scene

The shortest official headless path is
[`v2/openarm_bimanual.xml`](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml).
It is self-contained apart from its packaged assets and avoids the lifter,
table, and cell geometry. There is no official left-only v2 MJCF, so code must
load this bimanual model and select exactly seven left-arm names.

The official viewing scene is
[`v2/cell.xml`](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/cell.xml).
It attaches the same bimanual model to the OpenArm Cell and adds the lifter and
environment. It is useful for viewing, but is not the primary experiment model.
The standalone model compiles with `nq=18`, `nv=18`, `nu=16`, a 0.002 s
timestep, and MuJoCo's default gravity `[0, 0, -9.81] m/s^2`. The cell scene
sets a 0.001 s timestep and changes address ordering by adding the lifter.

## Joint, address, actuator, and passive-parameter evidence

All seven joints are revolute in the v2 description and compile as MuJoCo hinge
joints. Direct addresses below are **only** for standalone
`openarm_bimanual.xml`. Reliable code must resolve names, then read
`model.jnt_qposadr[jid]`, `model.jnt_dofadr[jid]`, and the actuator's
`model.actuator_trnid[aid]`; joint ID, qpos address, DoF address, and actuator ID
must never be treated as interchangeable. See the official [MuJoCo model field
definitions](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjtobj).

| J | Joint; parent -> child | Direct IDs/addresses (`jid,qpos,dof`) | Actuator (`aid`); transmission target | Position range / actuator control range (rad) | Actuator force range (N m) | Model passive values (`damping`, `frictionloss`, `armature`) | Associated real motor | Position actuator gains (`kp`,`kv`) | Evidence, confidence, engineering note |
| ---: | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | `openarm_left_joint1`; `openarm_left_base_link` -> `openarm_left_link1` | `0,0,0`; lookup by exact name | `left_joint1_ctrl` (`0`); joint 0, gear 1 | `[-3.4907, 1.3963]` / `[-3.49066, 1.39626]` | `[-40, 40]` | `1.0`, `0.2`, `0.0081` | DM-J8009P-2EC | `230`, `2.7` | Limits/actuator **SOURCED**, addresses **DERIVED**, passive values **MODEL_ESTIMATE**; high transcription confidence, unvalidated physical passives. [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml) [URDF limits](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v2.0/config/arm/joint/joint_limits.yaml) |
| 2 | `openarm_left_joint2`; `openarm_left_link1` -> `openarm_left_link2` | `1,1,1`; lookup by exact name | `left_joint2_ctrl` (`1`); joint 1, gear 1 | `[-3.3161, 0.17453]` / `[-3.31613, 0.174533]` | `[-40, 40]` | `1.0`, `0.2`, `0.0081` | DM-J8009P-2EC | `230`, `2.7` | Same classifications as J1. Left J1/J2 limits are the reflected form of the description limits. [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml) [left preset](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v2.0/config/robot_presets/left_arm.yaml) |
| 3 | `openarm_left_joint3`; `openarm_left_link2` -> `openarm_left_link3` | `2,2,2`; lookup by exact name | `left_joint3_ctrl` (`2`); joint 2, gear 1 | `[-1.5708, 1.5708]` / same | `[-27, 27]` | `0.9`, `0.1`, `0.1600` | DM-J4340P-2EC | `190`, `2.2` | Limits/actuator **SOURCED**, addresses **DERIVED**, passives **MODEL_ESTIMATE**; high transcription confidence. [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml) [topology](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v2.0/config/arm/struct/topology.yaml) |
| 4 | `openarm_left_joint4`; `openarm_left_link3` -> `openarm_left_link4` | `3,3,3`; lookup by exact name | `left_joint4_ctrl` (`3`); joint 3, gear 1 | `[0, 2.4435]` / `[0, 2.44346]` | `[-27, 27]` | `0.9`, `0.1`, `0.1600` | DM-J4340-2EC | `190`, `2.2` | Same classifications as J3; high transcription confidence. [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml) [motor mapping](https://docs.openarm.dev/setup/openarm-setup/run-demo/) |
| 5 | `openarm_left_joint5`; `openarm_left_link4` -> `openarm_left_link5` | `4,4,4`; lookup by exact name | `left_joint5_ctrl` (`4`); joint 4, gear 1 | `[-1.5708, 1.5708]` / same | `[-7, 7]` | `0.9`, `0.04`, `0.0100` | DM-J4310-2EC V1.1 | `30`, `1.5` | Limits/actuator **SOURCED**, addresses **DERIVED**, passives **MODEL_ESTIMATE**; high transcription confidence. [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml) [motor specifications](https://docs.openarm.dev/hardware/openarm-2.0/motor/) |
| 6 | `openarm_left_joint6`; `openarm_left_link5` -> `openarm_left_link6` | `5,5,5`; lookup by exact name | `left_joint6_ctrl` (`5`); joint 5, gear 1 | `[-0.7854, 0.7854]` / `[-0.785398, 0.785398]` | `[-7, 7]` | `0.9`, `0.04`, `0.0100` | DM-J4310-2EC V1.1 | `30`, `1.5` | Same classifications as J5; high transcription confidence. [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml) [motor mapping](https://docs.openarm.dev/setup/openarm-setup/run-demo/) |
| 7 | `openarm_left_joint7`; `openarm_left_link6` -> MJCF `openarm_left_ee_base_link` (description child `link7`) | `6,6,6`; lookup by exact name | `left_joint7_ctrl` (`6`); joint 6, gear 1 | `[-1.5708, 1.5708]` / same | `[-7, 7]` | `0.01`, `0.01`, `0.0049` | DM-J4310-2EC V1.1 | `30`, `1.5` | Actuator/limits **SOURCED**, addresses **DERIVED**, passives **MODEL_ESTIMATE**; **source conflict:** joint inherits `motor_DM3507` passives while its actuator and hardware mapping say DM4310. Do not use J7 passives as measured hardware facts. [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml) [motor mapping](https://docs.openarm.dev/setup/openarm-setup/run-demo/) |

MuJoCo's `<position>` shortcut makes `ctrl` a desired position, with an
internally generated force based on the configured gains and velocity term; it
is not a direct torque command. This follows the official [position-actuator
definition](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position).
The upstream actuator `gear="1"` means the scalar actuator force maps one-to-one
to the target joint generalized force, after the position-servo law and force
clamp have been evaluated; it does not change the semantics of `data.ctrl`.

## Motor ratings and consistency

The official 24 V values are below. rad/s values are **DERIVED** from the
sourced rpm values using `rpm * 2*pi/60`. The official setup example maps J1-2
to DM8009, J3-4 to DM4340, and J5-7 to DM4310. The J3 P/non-P distinction is a
bearing/package distinction; the public motor table gives one DM4340-series
rating.

| Motor; joints | Rated / peak torque (N m) | Rated velocity | Maximum no-load velocity | Gear ratio | Evidence, confidence, note |
| --- | ---: | ---: | ---: | ---: | --- |
| DM-J8009P-2EC; J1-2 | `20 / 40` | `100 rpm / 10.472 rad/s` at 24 V | `160 rpm / 16.755 rad/s` at 24 V | `9:1` | Torque/rpm/ratio **SOURCED**, rad/s **DERIVED**; medium confidence because OpenArm explicitly warns that the linked sheet is DM8009 while the arm uses DM8009P and calls them nearly identical. [Official motor page](https://docs.openarm.dev/hardware/openarm-2.0/motor/) |
| DM-J4340 series; J3-4 | `9 / 27` | `36 rpm / 3.770 rad/s` | `52 rpm / 5.445 rad/s` | `40:1` | Torque/rpm/ratio **SOURCED**, rad/s **DERIVED**; high transcription, medium physical confidence pending unit-level validation. [Official motor page](https://docs.openarm.dev/hardware/openarm-2.0/motor/) |
| DM-J4310-2EC V1.1; J5-7 | `3 / 7` | `120 rpm / 12.566 rad/s` | `200 rpm / 20.944 rad/s` | `10:1` | Torque/rpm/ratio **SOURCED**, rad/s **DERIVED**; high transcription, medium physical confidence pending unit-level validation. [Official motor page](https://docs.openarm.dev/hardware/openarm-2.0/motor/) |

The v2 description's `effort` values are 40, 27, and 7 N m and its `velocity`
values are 16.755, 5.4454, and 20.944 rad/s by motor group. Therefore its effort
limits correspond to **peak**, not rated, torque, and its velocities correspond
to maximum no-load, not rated, speed. The MJCF force ranges match those peak
torques. The MJCF has no actuator velocity clamp, so motor and MJCF limits are
only partially consistent.

The CAN library's `MOTOR_LIMIT_PARAMS` are protocol packing ranges, not safe
robot operating limits: DM4310 uses position/velocity/torque spans
`12.5 rad / 30 rad/s / 10 N m`, DM4340 `12.5 / 10 / 28`, and DM8009
`12.5 / 45 / 54`. The encoder/decoder clamps and maps MIT frames against those
ranges. They exceed several motor/robot limits and must not be copied into the
safety configuration. Sources: [constants header](https://github.com/enactic/openarm_can/blob/98666042b5e9cd5b55d0bd1d7fc3aa5c42caae4d/include/openarm/damiao_motor/dm_motor_constants.hpp),
[control implementation](https://github.com/enactic/openarm_can/blob/98666042b5e9cd5b55d0bd1d7fc3aa5c42caae4d/src/openarm/damiao_motor/dm_motor_control.cpp),
and [CAN documentation](https://docs.openarm.dev/api-reference/can/).

## Link inertial evidence

The following child-body inertials come from the selected MJCF. `I` lists the
six independent entries `[Ixx,Iyy,Izz,Ixy,Ixz,Iyz]` of the inertia tensor in
the body frame in kg m^2; those entries were **DERIVED** from the MJCF principal
inertias and inertial quaternion. All physical inertials are classified
**MODEL_ESTIMATE** because no weighing, pendulum, CAD-to-build comparison, or
system-identification result is published with them. Confidence is high that
the table reproduces the model and low-to-medium that a particular assembled
arm matches it.

| Joint child body | Mass (kg) | Center of mass in body frame (m) | Inertia tensor entries `[Ixx,Iyy,Izz,Ixy,Ixz,Iyz]` (kg m^2) | Source/classification/note |
| --- | ---: | --- | --- | --- |
| J1 `openarm_left_link1` | `1.14167` | `[0.00090035, 0.044466, 0.0000288441]` | `[1.094326e-3, 7.591993e-4, 6.706591e-4, -9.416377e-6, 4.534275e-8, -6.667535e-8]` | [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml), **MODEL_ESTIMATE** |
| J2 `openarm_left_link2` | `0.277509` | `[-0.0247185, -0.0000000760578, -0.0312643]` | `[1.535892e-4, 1.130620e-4, 8.229993e-5, 2.944250e-10, 5.210580e-6, -3.758140e-10]` | [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml), **MODEL_ESTIMATE** |
| J3 `openarm_left_link3` | `1.07386` | `[-0.00300862, -0.000514813, -0.109368]` | `[1.289393e-3, 1.140192e-3, 5.597657e-4, -2.299716e-7, -6.761588e-5, -1.062605e-5]` | [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml), **MODEL_ESTIMATE** |
| J4 `openarm_left_link4` | `0.634853` | `[-0.00310326, -0.00136539, -0.0574601]` | `[4.690039e-4, 4.339264e-4, 2.452117e-4, 5.636289e-6, -1.778799e-6, 3.417705e-6]` | [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml), **MODEL_ESTIMATE** |
| J5 `openarm_left_link5` | `0.615659` | `[-0.000737189, 0.000412407, -0.0470938]` | `[2.796537e-4, 3.345994e-4, 1.766629e-4, -3.317425e-6, 7.680976e-7, 1.499339e-7]` | [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml), **MODEL_ESTIMATE** |
| J6 `openarm_left_link6` | `0.475203` | `[0.0000227438, -0.000686496, 0.0000544]` | `[1.792759e-4, 1.362479e-4, 1.353402e-4, -6.586873e-8, -3.371043e-8, -3.325524e-8]` | [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml), **MODEL_ESTIMATE** |
| J7 `openarm_left_ee_base_link` | `0.52832` | `[0.0137902, -0.0084073, -0.05938]` | `[2.191100e-4, 1.562705e-4, 1.659005e-4, -4.168266e-6, 1.404309e-6, 1.512407e-6]` | [MJCF](https://github.com/enactic/openarm_mujoco/blob/ebc5cd29a957c8253887aab222ff3f7dc5907d4a/v2/openarm_bimanual.xml), **MODEL_ESTIMATE**; this is the gripper/EE-base body, not an independently documented arm-only `link7` inertia. |

The description's [nominal inertial
file](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v2.0/config/arm/inertials/nominal.yaml)
agrees closely with the MJCF for the bodies it covers. Agreement between two
model files is not experimental validation. The description's [friction
file](https://github.com/enactic/openarm_description/blob/6c7b720f1ba48e8bafa3a3dc752c45f397b42221/assets/robot/openarm_v2.0/config/arm/control/friction.yaml)
contains no identified physical friction parameters.

## Answers to the explicit investigation questions

1. **Does the selected actuator command represent joint torque directly?** No.
   Each selected upstream actuator is `<position>`, so `data.ctrl[aid]` is a
   position reference. For torque PD, implementation must deliberately use a
   torque-native path, such as a project-owned derived MJCF with motor
   actuators, or `data.qfrc_applied[dof]` with the vendor position actuators
   neutralized. The latter bypasses actuator force clamps, so software clamps
   become mandatory.
2. **One actuator per selected joint?** Yes: exactly one selected position
   actuator targets each of the seven selected joints. The full model also has
   left/right gripper and right-arm actuators (`nu=16`).
3. **Gravity already included?** Yes. The standalone file does not override
   MuJoCo's default `[0,0,-9.81] m/s^2`; the compiled model confirms it.
4. **Damping, friction loss, and armature included?** Yes, with the per-joint
   values in the evidence table. They are model values, not validated hardware
   measurements. Static friction, backlash, and compliance are not modeled.
5. **Force limits enforced by the model?** Yes for actuator-generated force:
   `forcerange` and matching joint `actuatorfrcrange` are enabled. No for force
   injected through `qfrc_applied`; that path needs explicit software limits.
6. **URDF and MJCF joint limits consistent?** Yes after applying the official
   left-arm reflection to asymmetric J1/J2 limits. Differences are decimal
   rounding no larger than approximately `4e-5 rad`.
7. **Motor datasheet and MJCF actuator limits consistent?** Peak torque ranges
   match. Rated torque is lower and is not represented as a separate MJCF
   limit. Motor velocity limits are absent from MJCF. J7's joint default class
   (`DM3507`) conflicts with its DM4310 actuator and official hardware mapping.
8. **Which values appear experimentally validated?** No published arm-level
   validation procedure or results were found for inertials, passive terms,
   latency, noise, compliance, backlash, or tracking gains. Manufacturer motor
   ratings are sourced specifications, not validation of a built arm.
9. **Which values appear to be model estimates?** Link mass/COM/inertia,
   damping, friction loss, armature, collision geometry, and the position-servo
   gains. The description and MJCF gains also differ, consistent with different
   control layers rather than one validated controller.
10. **Which values are missing?** Measured encoder/velocity noise, timing
    distributions, true viscous/Coulomb/static friction, temperature-dependent
    torque capability, current-to-torque accuracy, mechanical-stop
    repeatability, zero-calibration uncertainty, backlash/compliance, effective
    joint inertia, and an independently validated arm-only J7 terminal inertia.
11. **Which values should software treat conservatively?** Use rated torque,
    not peak torque, for normal operation; retain a normal-operation derating;
    hard-clamp at or below peak/model force limits; enforce position and
    velocity limits independently; distrust CAN packing ranges as safety
    limits; and treat all passives/inertials, particularly J7, as approximate.
12. **Software margin inside mechanical limits?** Start with **5 degrees
    (`0.0872665 rad`) per side**, classified **ASSUMED**, for simulation and
    planning. Before hardware motion, measure zero and stop repeatability and
    increase the margin if the uncertainty, braking distance, backlash, or
    harness clearance demands it. The margin is a provisional operating limit,
    not a sourced physical property.

## Controller choice: PD plus bias-force compensation

PD with MuJoCo bias-force compensation is selected over PID because bias
compensation analytically removes the modeled gravity/Coriolis contribution
rather than requiring integral action to accumulate against it. MuJoCo exposes
these terms in `qfrc_bias`; see the official [equations of
motion](https://mujoco.readthedocs.io/en/stable/computation/index.html#general-framework).
`qfrc_bias` does **not** include joint `damping` or `frictionloss`. Because the
selected model defines both, PD plus bias compensation is expected to retain a
small friction-induced steady-state error. That bounded residual must be
distinguished from incorrect gains, incorrect bias indexing, or incorrect
actuator mapping. Gains should not be raised aggressively merely to suppress
it; explicit friction feedforward or a small anti-windup integral term is the
appropriate real-world remedy.
Omitting the I term avoids integral windup interactions with torque-saturation
and tracking-divergence safety faults. On real hardware, where dynamics
parameters are only approximately known, a small anti-windup integral term
would likely be warranted to correct residual model error after the basic
controller and safety behavior are validated.

## Identification plan for estimated or missing parameters

Every procedure below is required before claiming hardware fidelity. Repeat
tests across relevant temperatures and at least three arm configurations where
gravity or reflected inertia can change the result.

| Parameter; current evidence | Equipment | Arm configuration | Input | Measurements | Fitting/calculation | Safety precautions |
| --- | --- | --- | --- | --- | --- | --- |
| Encoder noise; **MISSING** | CAN interface with hardware timestamps, stable 24 V supply, rigid fixture, optional external optical encoder | One joint at a time, torque disabled or mechanically supported, several midrange poses | No motion command; collect long stationary records | Raw position, motor temperature, timestamps, external encoder if available | Remove slow thermal drift; report robust standard deviation, peak-to-peak, quantization, PSD, and pose/temperature dependence | Support links, disable torque before fixtures are installed, E-stop in reach, stay away from pinch points |
| Velocity noise; **MISSING** | Same CAN logger, high-resolution external encoder/IMU | Stationary first, then isolated constant low velocities in both directions | Zero velocity, then safe constant-velocity sweeps | Reported velocity, differentiated position, external velocity, timestamps | Compare stationary sigma and PSD; fit bias and scale against external velocity; select filter from bandwidth/noise tradeoff | Low speed and rated-torque cap; generous software joint margin; abort on missed frames |
| Communication and actuation latency; **MISSING** | Logic analyzer/CAN hardware timestamps, synchronized host clock, external encoder or accelerometer | Single supported joint near midrange, other joints locked or supported | Small bounded torque pulse or pseudorandom binary sequence | Host send time, bus frame time, motor reply time, encoder/acceleration onset | Separate command transport, sensor-return, and mechanical response; use change-point and cross-correlation estimates; report median, p95, p99, and jitter | Begin below 10% rated torque, clear workspace, hard time/position/velocity cutoff, E-stop |
| Viscous friction; MJCF **MODEL_ESTIMATE** | Calibrated torque/current feedback, external encoder, temperature logging | One joint at a time; choose poses minimizing gravity coupling or subtract a validated gravity model | Multiple steady velocities of both signs, away from zero | Torque, velocity, position, temperature | After removing gravity/inertial terms, regress torque versus velocity; slope is viscous coefficient; inspect speed dependence | Rated-torque/velocity caps, stay well inside limits, short runs to avoid heating |
| Coulomb friction; MJCF `frictionloss` **MODEL_ESTIMATE** | Same as viscous test | Same isolated joint and multiple poses | Slow constant velocities in both directions | Steady torque and velocity | Jointly fit `tau = F_c*tanh(k*dq) + F_v*dq + offset`; average signed intercepts to separate Coulomb friction from bias | Avoid the stick-slip region initially; low speed, support gravity load, abort oscillation |
| Static/breakaway friction; **MISSING** | Torque-command interface, high-resolution encoder, external motion indicator | Supported joint at several poses and temperatures | Very slow monotonic torque ramps in each direction from rest | Torque at first repeatable motion, displacement, temperature | Estimate breakaway-torque distribution and hysteresis over repeated trials; keep separate from Coulomb term | Strict low torque ceiling, no personnel in sweep, stop immediately after breakaway, prevent falling under gravity |
| Software joint limits and stop margin; margin **ASSUMED**, stop repeatability **MISSING** | Calibrated encoders, manual/CAD stop references, feeler/dial indicator if accessible | Unloaded and supported; one joint at a time | Quasi-static, very low torque approach from both directions | Encoder at first contact, repeatability, backlash, cable/harness clearance, braking overshoot | Set soft limits inside the worst observed stop/clearance bound by calibration uncertainty + repeatability + worst-case stopping distance; never encode one universal margin without evidence | Prefer manual unpowered characterization; never strike a stop; current/torque limit; spotter and E-stop |
| Link mass and COM; **MODEL_ESTIMATE** | Calibrated scale, balancing knife edges or suspension rig, CAD/BOM, torque sensor | Disassembled link characterization, then assembled static poses | No dynamic command; optional low-gain gravity-hold validation | Component mass, balance points, joint torque over static poses | Compute COM from balance/suspension lines; compare predicted and measured gravity torque; update only project-owned parameters with uncertainty | Lockout/tagout power for disassembly, support every link, follow fastener torque procedure, no unsupported arm |
| Effective joint inertia; MJCF body/armature **MODEL_ESTIMATE** | Torque command/feedback, high-rate encoder, optional accelerometer | One joint centered, other joints locked/supported; repeat at several configurations | Small band-limited sine/chirp or bounded PRBS torque | Torque, position, velocity, acceleration, timing | Fit `tau = J_eff*qdd + b*qd + tau_f + tau_g`; cross-check with frequency-response inertia; report configuration dependence | Low amplitude, frequency and acceleration caps, resonance abort, rated torque cap, clear workspace |
| Compliance and backlash; **MISSING** | Calibrated torque wrench/load cell, external optical tracker or dial indicator, encoder logger | De-energized or low-stiffness joint, gravity supported, several poses | Slowly cycle known positive/negative torques | Motor encoder, output-link displacement, applied torque | Hysteresis loop: deadband gives backlash; loaded slope gives torsional compliance; repeat after direction reversal and at temperatures | Mechanically support link, no impulsive loading, remain below rated torque, do not use body weight as input |

## Source conflicts and unresolved uncertainty

- Current official MuJoCo documentation still says actuators use torque control,
  but current v2 MJCF uses `<position>` actuators. The model file controls this
  implementation decision; the prose is stale for v2.
- J7 is assigned `motor_DM3507` joint passives in MJCF while its actuator class,
  official seven-motor setup, and hardware documentation identify DM4310.
- MJCF and description controller gains differ substantially. They belong to
  different simulation/control layers and neither has published validation
  evidence; they are not portable PD gains for this project.
- The installed PyPI 2.0.1 bimanual asset matches the recorded upstream file,
  but its installed `cell.xml` does not match the later upstream cell blob.
  Therefore experiments must load the pinned bimanual model directly, not infer
  reproducibility from the launcher scene.
- The nominal description inertials and MJCF are mutually consistent but remain
  unvalidated model estimates. The J7/EE payload definition is especially
  uncertain for a left-arm-only experiment.
- Noise, latency, friction decomposition, calibration error, thermal behavior,
  compliance, backlash, and effective inertia remain unmeasured.

## Reproducible install and launch commands

Official package installation and viewer launch are:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install openarm-mujoco==2.0.1 mujoco==3.11.0
.\.venv\Scripts\openarm-mujoco-launch.exe --no-sheet
```

These commands assume `.venv` does not already exist. In the current workspace,
the previously created venv points to a Microsoft Store Python alias that has
since moved, so `.venv\Scripts\python.exe` must be recreated before the normal
demo command is used. This evidence-only step did not clear or rewrite the
environment. Read-only model validation instead used the currently installed
Python 3.11 executable with the existing site-packages and explicit asset path.

The reproducible headless load path for the selected model is:

```powershell
.\.venv\Scripts\python.exe -c "from openarm_mujoco.v2 import openarm_bimanual_xml; import mujoco; p=openarm_bimanual_xml(); m=mujoco.MjModel.from_xml_path(p); d=mujoco.MjData(m); mujoco.mj_forward(m,d); print(p, m.nq, m.nv, m.nu)"
```

The viewer launches the cell scene. Headless experiments must use
`openarm_bimanual_xml()` and select the names below, never assume cell-scene
indices.

## Implementation gate

- **Exact model:** `v2/openarm_bimanual.xml` from `openarm-mujoco==2.0.1`,
  matching upstream blob `0a4e8e96550a634a81d8ac8bc74f73c125006c15`.
- **Exact joints:** `openarm_left_joint1` through `openarm_left_joint7`.
- **Exact upstream actuators:** `left_joint1_ctrl` through
  `left_joint7_ctrl`.
- **Simulation implementation can proceed safely:** **yes, conditionally**.
  It must resolve every mapping by name, choose an explicit torque-native path,
  clamp torque/velocity/position/finite values in software, preserve the vendor
  files, and treat model passives/inertials as estimates. Writing PD torque to
  the current position-actuator `data.ctrl` is not safe or correct.
- **Hardware deployment can proceed safely from this evidence alone:** **no**.
  The identification procedures, zero/stop calibration, real-time CAN fault
  handling, and hardware-specific limits must be completed first.

## Measured baseline implementation result

The baseline implementation uses a project-owned in-memory conversion of only
the seven selected position actuators to fixed gain-one, zero-bias torque
actuators. Startup tests verify the target joint, scalar `+1` gear, zero built-in
position feedback, actuator-force magnitude, and joint-force sign. The vendor
MJCF remains unchanged.

With configured `kp=[40,40,35,35,8,8,6]` and
`kd=[5,5,4,4,1,1,0.8]`, the measured headless run completed 3001 samples with
overall RMS error `0.00526052 rad`, maximum requested/applied torque
`4.29030 N m`, zero saturation, zero safety violations, and zero non-finite
samples. Exact per-joint results are recorded in
`results/baseline_metrics.json`.

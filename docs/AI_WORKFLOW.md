# AI-agent workflow

## Agent and tools used

The primary implementation agent was **OpenAI Codex in the Codex desktop app**.
One primary agent performed the repository inspection, source research,
implementation, command execution, and review. No secondary review agent was
spawned, so this submission does not imply independent model review.

The agent used repository file inspection, PowerShell, `git diff --check`,
`apply_patch`, official web/GitHub evidence gathered during the read-only
provenance phase, MuJoCo's Python API, pytest, Ruff, a C++17 compiler, FFmpeg,
and manual image/video-frame inspection. It did not use ROS, reinforcement
learning, or hardware access.

## Decomposition strategy

The project was intentionally split into evidence-gated increments:

1. inspect official OpenArm model, description, motor, and CAN sources;
2. record classifications and unresolved conflicts before controller code;
3. resolve seven joint/qpos/DoF/actuator channels by exact names and test them;
4. implement the smallest PD-plus-bias baseline and measure it;
5. add stateful safety with one negative test per rule;
6. add exactly one controlled latency/noise perturbation and comparison;
7. create a fail-closed hardware/CAN design without pretending it is a driver;
8. package, rerun, and manually review the submission artifacts.

Each increment preserved vendor files and kept configuration/provenance separate
from controller code. Claims were added only after commands produced evidence.

## Representative prompts

These are reported excerpts from development task prompts. The repository does
not contain an immutable conversation export, so this audit cannot authenticate
their exact wording or prove that they are complete.

**Prompt 1 — evidence first**

> Perform a focused, read-only investigation of the official OpenArm
> repositories and documentation. Do not edit project files yet.

This forced the actuator semantics, joint limits, motor groups, CAN packing
ranges, and missing measurements to be established before implementation.

**Implementation prompt — safety**

> Implement and verify a small explicit safety subsystem for the simulation.
> The safety subsystem must distinguish planning limits, normal command limits,
> and absolute fault limits.

The agent decomposed this into immutable policies, a latched monitor, watchdogs,
mapping drift checks, fault logging, and deterministic positive/negative tests.

**Review prompt — do not game tests**

> Do not weaken a test merely to make it pass. Fix implementation errors or
> document a genuine model limitation.

This instruction was applied directly when tests or rendering checks failed.
Most regression thresholds are broad engineering policies rather than exact
output snapshots. The baseline's greater-than-50% peak-to-final reduction
threshold was selected after observing 59% and is therefore treated as weak,
partly circular evidence rather than an independent acceptance argument.

## Commands actually run

Representative successful commands included:

```powershell
py -3.11 -m venv .review-venv
.\.review-venv\Scripts\python.exe -m pip install -r requirements.txt
.\.review-venv\Scripts\python.exe -m ruff check src tests
.\.review-venv\Scripts\python.exe -m pytest -q
.\.review-venv\Scripts\python.exe -m src.experiment --mode baseline --headless --output results/baseline
.\.review-venv\Scripts\python.exe -m src.experiment --mode compare --headless --output results
.\.review-venv\Scripts\python.exe -m src.record_demo --check-only --width 1280 --height 720
.\.review-venv\Scripts\python.exe -m src.record_demo --output results/demo.mp4 --width 1280 --height 720 --fps 30
g++ -std=c++17 -Wall -Wextra -Wpedantic -fsyntax-only hardware/can_control_loop.cpp
```

The optional viewer was also run for the complete six-second baseline using a
temporary output prefix. The repository's original ignored `.venv` became tied
to a moved Microsoft Store launcher during development; final clone commands
were therefore revalidated in the fresh `.review-venv` above instead of hiding
that environment issue.

## Verification used

- 63 deterministic pytest cases covering named mapping, duplicate address/ID
  rejection, trajectory endpoints, limit validation, clipping, torque slew,
  finite values, hard limits, persistent divergence, watchdogs, fault latching,
  mapping drift, non-equal joint/qpos/DoF indexing, actuator/joint force clamps,
  controller signs, final hold indexing, baseline tracking, four-sample latency
  semantics, and the shortened fixed-seed perturbation regression.
- Ruff on all Python source and tests.
- Two full 3,001-sample headless experiments and JSON/CSV artifact checks.
- C++17 `-Wall -Wextra -Wpedantic -fsyntax-only` for the portable CAN skeleton;
  Linux SocketCAN and hardware behavior remain explicitly unverified.
- Manual inspection of baseline, latency, comparison plots and representative
  demo-video frames.
- `ffprobe` verification of demo codec, resolution, frame rate, duration, and
  size.

### Verification ledger

| Claim or risk | Evidence used before accepting it | Human review boundary |
| --- | --- | --- |
| Seven-joint mapping is correct | `tests/test_model_mapping.py` resolves every named joint, qpos address, DoF address, actuator ID, transmission target, uniqueness constraint, and startup fingerprint against compiled MuJoCo metadata. | Read the resolved names and transmission checks; did not infer indices from XML order. |
| Bias compensation is wired to the correct coordinates | `tests/test_controller.py::test_pd_feedback_signs_and_bias_is_added_exactly_once` checks sign and one-time addition; `ArmMapping.read_bias` indexes `qfrc_bias` with the separately resolved DoF addresses; full tracking then runs with finite logged `tau_bias`. | Inspected the `jnt_dofadr`-based lookup and logged bias columns. A low tracking error alone was not treated as proof of correct bias indexing. |
| Safety rules reject unsafe inputs | Negative cases in `tests/test_safety.py` cover discontinuity, NaN/Inf, hard limits, absolute torque, divergence, and stale command/feedback; `tests/test_model_mapping.py` covers mapping drift. | Checked that each test asserted the specific fault reason rather than accepting any exception. |
| Latency delays commands, not desired positions | Tracking regression asserts a four-sample queue from `0.008/0.002` and reported `implemented_actuation_latency_ms == 8.0`; generated CSV/JSON were inspected. | Read the queue placement between safety limiting and actuator write. |
| Results are real outputs | The `compare` command regenerated both 3,001-row CSVs, metric JSON, and plots; README values were cross-checked against those JSON fields. | Inspected tracking/torque/comparison plots for clipping, illegible axes, and qualitative oscillation. |
| Demo is genuinely HD | The check-only command printed original buffer `640x480`, adjusted buffer and renderer `1280x720`; `ffprobe` reported H.264, `1280x720`, 30 fps, 14.03 s. | Inspected initial-pose, motion, hold/plot, and hardware-slide frames rather than trusting encoder success. |
| CAN artifact compiles | `g++ -std=c++17 -Wall -Wextra -Wpedantic -fsyntax-only hardware/can_control_loop.cpp` exited zero. | Treated syntax as syntax only; no protocol or hardware behavior was inferred. |

## Source validation and what was not trusted

The agent followed the official repository index to commit-pinned
`enactic/openarm_mujoco`, `openarm_description`, and `openarm_can` files plus
official motor documentation. Values were cross-checked between MJCF,
description, motor pages, compiled MuJoCo metadata, and measured outputs.

The following were not trusted automatically:

- prose suggesting torque control when the selected MJCF actuators compile as
  position servos;
- CAN protocol packing spans as physical safety limits;
- agreement between two model files as experimental validation;
- generated plots without visual review;
- simulation success as hardware safety evidence;
- C++ protocol placeholders as a hardware driver;
- README metrics not traceable to generated JSON/CSV.
- a visually plausible trajectory as proof that `qfrc_bias` used the correct
  seven DoF addresses; named lookup, controller unit tests, and logged bias
  signals were required;
- a passing renderer constructor as proof of output resolution; reported model
  buffer dimensions, `renderer.width/height`, `ffprobe`, and decoded frames were
  checked separately.

## Real mistakes and corrections

1. A first `test_single_tracking_transient_does_not_fault` stimulus changed
   desired position by `0.6 rad` between adjacent samples. The full pytest run
   did not merely report a generic failure: the test raised the independently
   correct `SafetyFault` reason `desired_discontinuity` when it expected no
   fault. Reading the traceback and the two adjacent desired samples showed
   that the test setup violated a different safety rule. The stimulus—not the
   discontinuity threshold—was corrected to a one-sample measured-state error;
   the dedicated discontinuity negative test remained unchanged, and the full
   suite then passed.
2. The first
   `python -m src.record_demo --check-only --width 1280 --height 720` run failed
   before video encoding with `Image width 1280 > framebuffer width 640`.
   Inspecting `model.vis.global_.offwidth/offheight` returned `640x480`, proving
   that an explicit renderer size alone did not enlarge the OpenGL buffer. The
   script now reports the original buffer, raises those two compiled-model
   fields in memory to `1280x720`, constructs the explicit-size renderer, and
   verifies `renderer.width/height`. The rerun printed
   `ADJUSTED_OFFSCREEN_BUFFER=1280x720`, `RENDERER_RESOLUTION=1280x720`, and
   `RENDER_CHECK=passed`; `ffprobe` and decoded-frame inspection provided
   independent output checks. Vendor XML remains unchanged.
3. The official MJCF actuator interface was an important uncertainty: the
   selected upstream actuators were `<position>` actuators, not direct-torque
   commands. Named compiled-model inspection and force/sign probes detected
   this. The implementation performs a tested, in-memory gain-one/zero-bias
   conversion on only the seven selected actuators rather than writing torque
   into a position-reference channel.

4. The final adversarial audit found that configured absolute torque was not
   explicitly compared with both compiled MuJoCo force-clamp layers. Startup
   now rejects a mismatch or disabled actuator/joint actuator-force clamp, with
   negative tests. A synthetic free-joint fixture also proves state reads use
   qpos and DoF addresses when those differ from joint IDs.

## Adversarial AI validation

The final review used a separate clean `.audit-venv`, independently generated
baseline/comparison artifacts outside the repository results directory, and
three temporary mutations. Each mutation was restored immediately:

- reducing only J7 absolute torque to `2.0 N m` made the baseline regression
  fail because normal torque (`2.4 N m`) exceeded the absolute limit;
- reversing the proportional error sign made the end-to-end baseline fault at
  `t=1.578 s` on measured velocity; this proves fault detection, not that the
  divergence rule is always the first detector;
- duplicating `left_joint1_ctrl` in the seven-actuator selection made mapping
  startup fail on the uniqueness rule.

The review also checked the official motor page against configured rated/peak
torque and rpm-derived rad/s values, inspected the installed official v2 MJCF,
checked generated JSON/CSV and plots, decoded representative demo frames, and
compiled the C++ skeleton in syntax-only mode. Full commands and unresolved
items are in [`FINAL_REVIEW.md`](FINAL_REVIEW.md).

## Human review procedure

Before submission, the author should personally inspect `git diff`, rerun the
clean installation/tests/experiments, compare README values against generated
JSON/CSV, review plots and the complete demo, open the cited official sources,
and confirm that hardware, timing, collision, and safety limitations remain
explicit. Human sign-off must not be inferred from an AI test report; this
repository does not independently prove that the author completed that step.

The final AI reviewer independently inspected source links and conflicts;
model/joint/actuator selection; actuator conversion and sign tests; safety fault
semantics; generated metrics; plot readability; six representative demo frames;
the hardware design boundary; and final README claims.

Still unvalidated by this project: a real arm, CAN electrical layer, wire
frames against hardware, motor identities/zeros/signs, current-loop
dynamics, temperature thresholds, physical E-stop/power isolation, stopping
distance, collision fidelity, inertial accuracy, backlash, or compliance.

## Confidence and remaining uncertainty

Confidence is high that the checked-in Python experiment deterministically
reproduces the reported MuJoCo results and detects the tested software faults.
Confidence is moderate that the selected model/passive parameters represent a
particular assembled OpenArm, because no experimental validation was found.
Confidence in hardware readiness is intentionally low: the CAN artifact is a
reviewable design skeleton, and physical commissioning requires measurement,
independent protection validation, and formal risk assessment.

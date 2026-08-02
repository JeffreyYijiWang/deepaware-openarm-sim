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

These are excerpts from the actual task prompts, not reconstructed success
stories.

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

This instruction was applied directly when tests or rendering checks failed;
thresholds were tied to engineering meaning rather than exact output snapshots.

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

- 57 deterministic pytest cases covering named mapping, duplicate address/ID
  rejection, trajectory endpoints, limit validation, clipping, torque slew,
  finite values, hard limits, persistent divergence, watchdogs, fault latching,
  mapping drift, baseline tracking, four-sample latency semantics, and the
  shortened fixed-seed perturbation regression.
- Ruff on all Python source and tests.
- Two full 3,001-sample headless experiments and JSON/CSV artifact checks.
- C++17 `-Wall -Wextra -Wpedantic -fsyntax-only` for the portable CAN skeleton;
  Linux SocketCAN and hardware behavior remain explicitly unverified.
- Manual inspection of baseline, latency, comparison plots and representative
  demo-video frames.
- `ffprobe` verification of demo codec, resolution, frame rate, duration, and
  size.

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

## Real mistakes and corrections

1. A first “single tracking transient” negative test changed desired position by
   `0.6 rad` between adjacent samples. The independent discontinuity rule
   correctly faulted, producing one failed test. The stimulus—not the safety
   threshold—was corrected to a transient measured-state error, after which the
   full suite passed.
2. The demo script initially passed `1280x720` to `mujoco.Renderer`, but the
   selected model's compiled offscreen buffer remained `640x480`; the render
   check failed with “image width 1280 > framebuffer width 640.” The script now
   reports the original buffer, raises `model.vis.global_.offwidth/offheight`
   in memory to `1280x720`, constructs the explicit-size renderer, and verifies
   the actual resolution. Vendor XML remains unchanged.
3. The official MJCF actuator interface was an important uncertainty: the
   selected upstream actuators were `<position>` actuators, not direct-torque
   commands. Named compiled-model inspection and force/sign probes detected
   this. The implementation performs a tested, in-memory gain-one/zero-bias
   conversion on only the seven selected actuators rather than writing torque
   into a position-reference channel.

## Manual review

Manually reviewed: source links and conflicts; model/joint/actuator selection;
the actuator conversion and torque sign tests; safety fault semantics; all
generated metric JSON; plot readability; five representative demo frames; the
hardware design's fail-closed boundary; and final README links/claims.

Not manually or automatically validated: a real arm, CAN electrical layer,
wire frames against hardware, motor identities/zeros/signs, current-loop
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

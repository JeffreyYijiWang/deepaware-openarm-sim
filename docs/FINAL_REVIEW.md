# Final skeptical review

Review date: 2026-08-01  
Scope: repository state in the local `main` worktree, OpenArm v2 MuJoCo simulation, generated artifacts, tests, and proposed hardware bridge.

## 1. Executive verdict

The minimum **simulation** acceptance criteria passed after narrowly scoped corrections: a clean Python 3.11 environment installed the declared dependencies, 67 tests passed, both requested headless experiments completed, expected artifacts were regenerated, and the intentional torque-limit, controller-sign, and duplicate-actuator mutations each caused a relevant failure before being restored. The final count includes four later model-level tests for the explicitly requested optional left-arm/gripper showcase.

This is acceptable evidence for a simulation take-home. It is **not evidence of hardware readiness, certified safety, or general closed-loop stability**. No physical OpenArm, CAN interface, real-time scheduler, independent E-stop, or power-isolation path was tested. The latency/noise result is one deterministic synthetic case, and its `remained_stable` field is a threshold classification rather than a stability proof.

The submission should not be called final-ready until the author personally reviews the diff and complete demo, reruns the documented commands on the submission commit, and commits/pushes the reviewed corrections. That human sign-off is currently unverified.

## 2. Commands run

The audit used the following commands from the repository root. `.audit-venv` was a new Python 3.11 environment used independently of the development environment.

```powershell
py -3.11 -m venv .audit-venv
.\.audit-venv\Scripts\python.exe -m pip install -r requirements.txt
.\.audit-venv\Scripts\python.exe -m pip install --use-feature=truststore -r requirements.txt
.\.audit-venv\Scripts\python.exe -m pytest -q
.\.audit-venv\Scripts\python.exe -m pytest -q tests/test_model_mapping.py tests/test_controller.py tests/test_metrics.py
.\.audit-venv\Scripts\python.exe -m src.experiment --mode baseline --headless --output C:\tmp\deepaware-audit-initial\baseline
.\.audit-venv\Scripts\python.exe -m src.experiment --mode compare --headless --output C:\tmp\deepaware-audit-initial\compare
.\.audit-venv\Scripts\python.exe -m src.experiment --mode baseline --headless --output results/baseline
.\.audit-venv\Scripts\python.exe -m src.experiment --mode compare --headless --output results
.\.audit-venv\Scripts\python.exe -m ruff check src tests
.\.audit-venv\Scripts\python.exe -m ruff format --check src tests
g++ -std=c++17 -Wall -Wextra -Wpedantic -fsyntax-only hardware/can_control_loop.cpp
ffprobe results/demo.mp4
```

For each adversarial mutation, `apply_patch` changed one value or expression, the targeted pytest command was run, and `apply_patch` immediately restored the original. A final search confirmed that none of the mutation text remained.

Environment observations:

- Creating the venv inside the restricted Windows sandbox initially resolved the Microsoft Store Python launcher incorrectly; creating it in the normal host context worked. This is a Codex sandbox/launcher interaction, not a repository defect.
- The first dependency install failed on the host certificate chain. Retrying with pip's OS trust-store support succeeded without disabling TLS verification. The README now documents this fallback.
- FFmpeg/`ffprobe` and a C++17 compiler are prerequisites only for video verification and the optional hardware-skeleton syntax check; they are not required for the Python tests or headless experiments.

## 3. Tests and experiments reproduced

| Check | Result |
| --- | --- |
| Original test suite before audit corrections | `57 passed in 30.49 s` |
| Final full suite | `67 passed in 21.06 s` (latest post-showcase run) |
| Final Ruff lint and format checks | Passed; 17 files already formatted |
| Baseline experiment | Completed, 3,001 samples from `0.0` through `6.0 s`, zero faults |
| Latency/noise experiment | Completed, 3,001 samples, zero faults |
| Baseline overall RMS error | `0.005260518892045125 rad` |
| Baseline maximum final error | `0.004369799751320375 rad` |
| Latency/noise overall RMS error | `0.005220033619381512 rad` |
| Latency/noise maximum final error | `0.0037792890361541542 rad` |
| Implemented latency | Four simulation samples at `0.002 s`, reported as `8.0 ms` |
| Perturbed torque-rate intervention | 55 samples, `1.832722%`; normal torque clipping remained zero |
| C++ hardware skeleton | C++17 syntax and warning check passed |
| Demo file | H.264, 1280x720, 30 fps, 14.03 s, 421 frames |
| Optional showcase video | H.264, 1280x720, 30 fps, 15.60 s; visually inspected |

The initial audit outputs written under `C:\tmp` matched the committed baseline CSV/JSON content byte-for-byte. Final runs regenerated these expected files:

- `results/baseline.csv`, `baseline_metrics.json`, `baseline_tracking.png`, and `baseline_torque.png`;
- `results/latency_noise.csv`, `latency_noise_metrics.json`, `latency_noise_tracking.png`, and `latency_noise_torque.png`;
- `results/comparison_metrics.json`, `baseline_vs_latency.png`, and `demo.mp4`.

Tracking and torque plots were visually inspected. The baseline was smooth, did not show qualitative oscillation, stayed below its normal torque limits, and reached the one-second endpoint hold. Six representative video frames were decoded and inspected; successful encoding alone was not treated as visual verification.

## 4. Critical issues found

1. **Configured absolute torque was not tied to both compiled MuJoCo clamp layers. Corrected.** Before the audit, configuration validation enforced relationships among configured limits but did not prove that the compiled actuator `forcerange` and joint `actfrcrange` matched those absolute limits or were enabled. A future configuration/model drift could therefore have invalidated the claimed absolute torque boundary. Startup now rejects disabled or mismatched clamp metadata.

2. **The old mapping tests could not detect joint-ID/address confusion. Corrected.** The production mapping already used `jnt_qposadr` and `jnt_dofadr` correctly, but in the selected model the relevant joint IDs happen to equal their addresses. The tests therefore did not independently demonstrate correct indexing. A synthetic model with a preceding free joint now forces joint IDs, qpos addresses, and DoF addresses to differ and verifies state reads against the proper addresses.

3. **Some documentation implied stronger provenance or review evidence than the repository could authenticate. Corrected.** Reported prompt excerpts cannot be proven verbatim without an immutable conversation export, and prior human/manual-review wording could be read as evidence of author sign-off. The workflow now labels those excerpts as reported, describes the AI inspection actually performed, and makes author sign-off an explicit unverified requirement.

No uncorrected critical defect was found in the scoped simulation path. That statement does not extend to hardware operation.

## 5. Corrections made

- Added startup validation that each selected actuator and joint has an enabled force clamp matching the configured absolute torque limit.
- Passed configured peak/absolute torque values into actuator-interface adaptation.
- Added a non-equal joint/qpos/DoF address fixture and negative clamp-mismatch/disabled-clamp tests.
- Added direct controller tests for position-error sign, velocity-error sign, units-compatible arithmetic, and adding bias exactly once.
- Added metric tests proving final error uses the final hold sample rather than the move endpoint and that hold metrics cover the intended interval.
- The audit updated README and workflow test counts from 57 to 63; the later
  optional showcase added four model-level tests, producing the current 67.
- Added the README deliverables list, adversarial AI-validation description, human-review procedure, trust-store install fallback, and more precise limitations.
- Removed or narrowed claims that could be mistaken for authenticated prompt history, completed human sign-off, general stability, or hardware validation.

All changes were limited to validation, focused regression tests, and documentation. The controller architecture, experiment definition, vendor model, gains, trajectory, and result thresholds were not broadly redesigned.

## 6. Noncritical issues

- The baseline regression's greater-than-50% peak-to-final error-reduction threshold was chosen after observing approximately 59%. It is weak and partly circular evidence, with a modest margin. Completion, explicit RMS/final-error bounds, limit checks, and mutation tests are stronger evidence.
- The comparison's `remained_stable` result uses declared thresholds and one seed. It should remain a regression classification, not be promoted to a robust-control or Lyapunov stability claim.
- The CLI nonzero-fault test monkeypatches the experiment function. It verifies exception-to-exit-code wiring, but not an actual subprocess, OS signal, CAN shutdown, or physical drive-disable sequence.
- Several integration assertions primarily confirm completion and artifact generation. They are useful reproducibility checks but do not independently validate physical correctness.
- `requirements.txt` is directly pinned and reproduced successfully, but the dependency install is not hash-locked. Package-index availability and host certificate configuration remain external dependencies.
- The official bimanual asset leaves the right arm present but uncontrolled. The selected left-arm actuators are isolated correctly; computational and collision side effects of retaining the full bimanual scene were not studied.
- J7 has a source conflict: the joint's passive model class is labeled DM3507 while the actuator/hardware assignment is DM4310. The project documents the conflict and uses the official actuator/motor assignment, but physical resolution remains necessary.

## 7. Unsupported claims removed or revised

- “Verbatim prompt excerpts” became reported excerpts whose exact wording and completeness are unauthenticated.
- Human/manual review is no longer implied to be author approval; the actual AI frame/plot inspection and the required future human sign-off are separated.
- “Stable” is explicitly limited to the current threshold-based, fixed-seed simulation comparison.
- The hardware bridge remains a proposed design skeleton; syntax compilation is not represented as protocol, timing, electrical, or physical validation.
- Latency and sensor noise remain declared assumptions, not measured OpenArm properties.
- Model damping, friction, armature, inertia, and collision parameters remain model estimates unless an official measured source is cited.
- Torque-interface documentation now states that startup verifies both compiled force-clamp layers instead of merely assuming configuration and model agreement.

## 8. Remaining uncertainties

The following are explicitly **unverified**:

- real motor IDs, motor-to-joint wiring, encoder zeros, directions, gear signs, command units, and drive-disable semantics;
- measured CAN latency, jitter, packet loss, encoder noise, Linux scheduling behavior, and physical watchdog timing;
- current-loop dynamics, peak-torque duration, thermal limits, supply behavior, stopping distance, and regenerative-energy handling;
- physical E-stop, contactor/power isolation, fault containment, and risk-assessment compliance;
- assembled-arm masses, centers of mass, inertias, friction, damping, backlash, compliance, collision geometry, and payload effects;
- whether the official model's J7 DM3507/DM4310 inconsistency matches the particular arm;
- robustness over other trajectories, payloads, seeds, latency distributions, noise spectra, model errors, contacts, or disturbances;
- interactive viewer behavior on another graphics stack and the proposed Linux `vcan` CI path;
- playback of the README's hosted GitHub video attachment from a clean external account;
- author completion of the human-review checklist and publication of the current local corrections.

## 9. Interview questions the author should be ready to answer

1. Why must joint ID, qpos address, DoF address, and actuator ID be resolved separately, and how does the synthetic free-joint test prove that?
2. Why is `qfrc_bias` indexed by DoF address, what forces does it contain, and why is adding it once not double gravity compensation here?
3. The upstream actuators are position servos. Exactly which compiled fields are changed in memory to create the selected torque interface, and which clamp fields remain active?
4. Why are the normal torque limits 80% of rated torque while absolute limits use peak/model force limits? What measurements would be required before using either on hardware?
5. Explain why a four-slot initialized command queue at a 2 ms step implements an 8 ms actuation delay rather than 6 or 10 ms.
6. Which signal is delayed, which signals receive noise, and how does baseline mode prove it has neither injection?
7. Why did reversing the proportional sign trip the velocity rule first? What does that mutation prove, and what does it not prove about the divergence detector?
8. How are move, endpoint hold, RMS error, hold velocity, and final error intervals indexed? Why is `error[-1]` important?
9. What is the distinction among normal clipping, absolute torque faulting, slew-rate intervention, watchdog faulting, and the final zero-torque fault state?
10. Which safety mechanisms run only in the synchronous simulation process, and what independent hardware protections are still required?
11. Why is the current latency/noise outcome not a general stability proof, even though its measured RMS error is slightly lower than baseline?
12. How would you resolve the J7 motor-class conflict and validate motor assignments without trusting XML order or CAN packing ranges?

## 10. Final submission checklist

| Item | Status |
| --- | --- |
| README quick-start commands are valid for Python 3.11 | **PASS**, with documented Windows trust-store fallback |
| Declared Python dependencies install in a clean environment | **PASS** |
| Full unit/integration suite passes | **PASS: 63 tests** |
| Baseline headless experiment completes and writes expected artifacts | **PASS** |
| 8 ms latency/noise comparison completes and writes expected artifacts | **PASS** |
| README numerical results match generated JSON/CSV | **PASS** |
| OpenArm v2 left joints/actuators and transmission targets validated by name | **PASS** |
| qpos/DoF indexing and bias indexing independently exercised | **PASS** |
| Selected torque interface and both absolute-force clamps validated | **PASS** |
| Every implemented simulation safety rule has a negative test | **PASS** |
| Required intentional mutations fail and are restored | **PASS** |
| Demo exists locally and its encoded properties/representative frames were checked | **PASS** |
| Hardware bridge clearly separated from measured hardware behavior | **PASS** |
| C++ bridge skeleton syntax-checks | **PASS**, syntax only |
| Physical hardware/CAN/safety validation | **UNVERIFIED / OUT OF SCOPE** |
| Hosted demo playback from a clean external account | **UNVERIFIED** |
| Author reviewed the full diff, plots, complete video, and cited sources | **UNVERIFIED — author action required** |
| Current corrections committed and pushed to the submission repository | **PENDING — local worktree contains changes** |

**Final status:** the corrected simulation evidence passes the scoped minimum checks, but the repository must not be described as hardware-ready or fully submission-ready while the human sign-off and publication items remain incomplete.

# Generated result summary

Regenerate this directory with:

```powershell
.\.review-venv\Scripts\python.exe -m src.experiment --mode compare --headless --output results
```

The values below come from `baseline_metrics.json`,
`latency_noise_metrics.json`, and `comparison_metrics.json`; they are not design
targets or hand-selected best runs.

| Metric | Baseline | 8 ms actuator latency + assumed sensor noise |
| --- | ---: | ---: |
| Samples | 3,001 | 3,001 |
| Completion | completed | completed |
| Overall RMS position error | 0.0052605189 rad | 0.0052200336 rad |
| Maximum position error | 0.0112240517 rad | 0.0112934515 rad |
| Maximum final absolute error | 0.0043697998 rad | 0.0037792890 rad |
| Maximum measured velocity | 0.0754294371 rad/s | 0.0804136081 rad/s |
| Maximum requested torque | 4.2902964141 N m | 4.5011533871 N m |
| Aggregate command intervention | 0% | 1.83272243% |
| Normal torque clipping | 0% | 0% |
| Torque-rate limiting | 0% | 1.83272243% (55 samples) |
| Safety violations / non-finite samples | 0 / 0 | 0 / 0 |
| Significant hold-error zero crossings | 0 | 0 |
| Maximum endpoint overshoot | 0 rad | 0.0000702377 rad |

The classified comparison reports: no material tracking-error increase, no
material overshoot, no oscillation, no safety-limit trigger, and stable
completion. The slightly lower perturbed RMS is one deterministic stochastic
trial and is not evidence that noise improves control.

Primary artifacts:

- `baseline.csv`, `baseline_metrics.json`, `baseline_tracking.png`,
  `baseline_torque.png`
- `latency_noise.csv`, `latency_noise_metrics.json`,
  `latency_noise_tracking.png`, `latency_noise_torque.png`
- `comparison_metrics.json`, `baseline_vs_latency.png`
- `demo.mp4` — 14.03 s, H.264, 1280x720, 30 fps
- `left_arm_showcase.mp4`, `left_arm_showcase_metrics.json` — optional 15.60 s
  scripted left-arm horizontal/bend/rotation and full-MJCF-range claw cycle;
  explicitly not a controller-validation run

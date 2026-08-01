"""Deterministic end-to-end baseline regression tests."""

from __future__ import annotations

import json

import pytest

from src import experiment
from src.safety import SafetyFault


def test_baseline_tracking_is_safe_stable_and_deterministic(tmp_path) -> None:
    first_prefix = tmp_path / "first" / "baseline"
    second_prefix = tmp_path / "second" / "baseline"
    first = experiment.run_baseline(first_prefix, make_plots=False)
    second = experiment.run_baseline(second_prefix, make_plots=False)

    assert first["experiment_completion_status"] == "completed"
    assert first["safety_violations"] == 0
    assert first["nonfinite_samples"] == 0
    assert first["torque_saturated_samples"] == 0
    # The remaining milliradian error is expected from model damping/friction,
    # which qfrc_bias does not compensate. Do not force aggressive gain tuning.
    assert first["peak_to_final_error_reduction_fraction"] > 0.5
    assert first["maximum_hold_velocity_rad_s"] < 0.05
    assert first["overall_rms_position_error_rad"] == pytest.approx(
        second["overall_rms_position_error_rad"], abs=1e-15
    )
    assert first["final_position_error_per_joint_rad"] == pytest.approx(
        second["final_position_error_per_joint_rad"], abs=1e-15
    )
    assert first_prefix.with_suffix(".csv").is_file()
    metrics_path = first_prefix.parent / "baseline_metrics.json"
    assert metrics_path.is_file()
    assert (
        json.loads(metrics_path.read_text(encoding="utf-8"))[
            "experiment_completion_status"
        ]
        == "completed"
    )


def test_cli_returns_nonzero_after_safety_fault(monkeypatch, tmp_path) -> None:
    def faulting_run(*args, **kwargs):
        raise SafetyFault("test_fault", "injected by regression test")

    monkeypatch.setattr(experiment, "run_baseline", faulting_run)
    exit_code = experiment.main(
        ["--mode", "baseline", "--headless", "--output", str(tmp_path / "fault")]
    )
    assert exit_code != 0

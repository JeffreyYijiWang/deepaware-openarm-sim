"""Deterministic end-to-end baseline regression tests."""

from __future__ import annotations

import json

import numpy as np
import pytest

from src import experiment
from src.safety import SafetyEvent, SafetyFault, SafetyMonitor


def test_baseline_tracking_is_safe_stable_and_deterministic(tmp_path) -> None:
    first_prefix = tmp_path / "first" / "baseline"
    second_prefix = tmp_path / "second" / "baseline"
    first = experiment.run_baseline(first_prefix, make_plots=False)
    second = experiment.run_baseline(second_prefix, make_plots=False)

    assert first["experiment_completion_status"] == "completed"
    assert first["safety_violations"] == 0
    assert first["nonfinite_samples"] == 0
    assert first["torque_saturated_samples"] == 0
    assert first["latency_queue_length_samples"] == 0
    assert first["implemented_actuation_latency_ms"] == 0.0
    assert first["position_measurement_noise_standard_deviation_rad"] == 0.0
    assert first["velocity_measurement_noise_standard_deviation_rad_s"] == 0.0
    assert first["measurement_delay_ms"] == 0.0
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
        raise SafetyFault(
            SafetyEvent("test_fault", 1.25, "injected by regression test"),
            np.zeros(7),
        )

    monkeypatch.setattr(experiment, "run_baseline", faulting_run)
    exit_code = experiment.main(
        ["--mode", "baseline", "--headless", "--output", str(tmp_path / "fault")]
    )
    assert exit_code != 0


def test_run_fault_stops_trajectory_and_writes_safe_behavior(
    monkeypatch, tmp_path
) -> None:
    def faulting_evaluate(self, **kwargs):
        self.trip("injected_fault", "integration regression", kwargs["simulation_time"])

    monkeypatch.setattr(experiment.SafetyMonitor, "evaluate", faulting_evaluate)
    prefix = tmp_path / "faulted" / "baseline"
    with pytest.raises(SafetyFault, match="injected_fault"):
        experiment.run_baseline(prefix, make_plots=False)

    fault_record = json.loads(
        (prefix.parent / "baseline_fault.json").read_text(encoding="utf-8")
    )
    assert fault_record["state"] == "FAULT"
    assert fault_record["reason"] == "injected_fault"
    assert fault_record["timestamp"] == pytest.approx(0.0)
    assert fault_record["active_trajectory"] is False
    assert fault_record["safe_simulated_behavior"] == "zero_torque"
    assert fault_record["safe_torque_nm"] == [0.0] * 7


def test_actuation_delay_is_derived_as_four_samples_and_delays_commands() -> None:
    delay = experiment.ActuationDelay(0.008, 0.002)
    assert delay.queue_length == 4
    assert delay.implemented_latency_seconds * 1000.0 == pytest.approx(8.0)

    commands = [np.full(7, float(index)) for index in range(1, 6)]
    outputs = [delay.push(command) for command in commands]
    assert all(np.array_equal(output, np.zeros(7)) for output in outputs[:4])
    assert np.array_equal(outputs[4], commands[0])


def test_short_latency_noise_regression_is_stable_and_artifact_free(tmp_path) -> None:
    policy = experiment.load_regression_policy()
    run_arguments = {
        "make_plots": False,
        "write_artifacts": False,
        "trajectory_duration": policy["shortened_trajectory_duration"],
        "hold_duration": policy["shortened_hold_duration"],
    }
    first = experiment.run_latency_noise(tmp_path / "first", **run_arguments)
    second = experiment.run_latency_noise(tmp_path / "second", **run_arguments)

    assert first["experiment_completion_status"] == "completed"
    assert first["random_seed"] == 42
    assert first["latency_queue_length_samples"] == 4
    assert first["implemented_actuation_latency_ms"] == pytest.approx(8.0)
    assert first["measurement_delay_ms"] == 0.0
    assert first["nonfinite_samples"] == 0
    assert first["safety_violations"] == 0
    assert first["minimum_position_limit_margin_rad"] > 0.0
    assert first["minimum_velocity_limit_margin_rad_s"] > 0.0
    for joint_name, hard_limit in zip(
        first["joint_names"], first["hard_torque_limits_nm"], strict=True
    ):
        assert first["maximum_requested_torque_per_joint_nm"][joint_name] < hard_limit
    assert first["overall_rms_position_error_rad"] < policy["maximum_overall_rms_error"]
    assert first["maximum_position_error_rad"] < policy["maximum_position_error"]
    assert (
        first["maximum_final_position_error_rad"]
        < policy["maximum_final_position_error"]
    )
    assert first["maximum_measured_velocity_rad_s"] < policy["maximum_velocity"]
    assert (
        first["torque_saturated_samples_percent"]
        < policy["maximum_torque_saturation_percentage"]
    )
    assert (
        abs(
            first["overall_rms_position_error_rad"]
            - second["overall_rms_position_error_rad"]
        )
        < policy["deterministic_repeat_rms_tolerance"]
    )
    assert not list(tmp_path.rglob("*"))


def test_configured_stale_command_fault_stops_active_control_and_records_reason() -> (
    None
):
    parameters = experiment.load_parameters()
    monitor = SafetyMonitor(experiment.build_safety_config(parameters, 0.002))
    zeros = np.zeros(7)

    with pytest.raises(SafetyFault, match="stale_command") as raised:
        monitor.evaluate(
            simulation_time=0.012,
            command_timestamp=0.0,
            feedback_timestamp=0.012,
            simulation_timestep=0.002,
            desired_position=zeros,
            desired_velocity=zeros,
            measured_position=zeros,
            measured_velocity=zeros,
            feedback_torque=zeros,
            bias_torque=zeros,
            requested_torque=zeros,
        )

    assert not monitor.active_trajectory
    assert monitor.events[-1].reason == "stale_command"
    assert monitor.events[-1].timestamp == pytest.approx(0.012)
    assert np.array_equal(raised.value.safe_torque, np.zeros(7))

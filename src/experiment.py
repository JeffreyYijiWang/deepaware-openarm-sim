"""Deterministic OpenArm v2 left-arm joint-space tracking experiment."""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import deque
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np
import pandas as pd
import yaml

from .controller import JointSpacePD
from .metrics import signal_column, write_comparison_outputs, write_outputs
from .model_mapping import (
    ArmMapping,
    adapt_selected_actuators_to_torque,
    load_bimanual_model,
    resolve_arm_mapping,
    verify_torque_interface,
)
from .safety import (
    AbsoluteFaultLimits,
    NormalCommandLimits,
    PlanningLimits,
    SafetyConfig,
    SafetyFault,
    SafetyMonitor,
    TemporalSafetyConfig,
)
from .trajectory import QuinticTrajectory

if TYPE_CHECKING:
    from collections.abc import Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = REPOSITORY_ROOT / "config/openarm_limits.yaml"


def _value(node: dict[str, Any]) -> Any:
    return node["value"]


@dataclass(frozen=True)
class ExperimentParameters:
    """Only the source-backed/configured values needed by the baseline."""

    joint_names: tuple[str, ...]
    hard_lower: np.ndarray
    hard_upper: np.ndarray
    rated_torque: np.ndarray
    peak_torque: np.ndarray
    rated_velocity: np.ndarray
    control_frequency: float
    trajectory_duration: float
    hold_duration: float
    start_position: np.ndarray
    target_offset: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    planning_margin: float
    normal_torque_fraction: float
    hard_velocity_fraction: float
    trajectory_velocity_fraction: float
    desired_position_step_limit: np.ndarray
    torque_rate_limit: np.ndarray
    tracking_divergence: float
    tracking_persistence_samples: int
    command_timeout: float
    feedback_timeout: float
    timestep_tolerance_fraction: float
    safe_behavior: str
    endpoint_tolerance: float
    actuation_latency: float
    position_noise_standard_deviation: float
    velocity_noise_standard_deviation: float
    random_seed: int

    @property
    def planning_lower(self) -> np.ndarray:
        return self.hard_lower + self.planning_margin

    @property
    def planning_upper(self) -> np.ndarray:
        return self.hard_upper - self.planning_margin

    @property
    def normal_torque(self) -> np.ndarray:
        return self.rated_torque * self.normal_torque_fraction

    @property
    def hard_velocity_limit(self) -> np.ndarray:
        return self.rated_velocity * self.hard_velocity_fraction

    @property
    def trajectory_velocity_limit(self) -> np.ndarray:
        return self.rated_velocity * self.trajectory_velocity_fraction


def load_parameters(config_path: Path = DEFAULT_CONFIG_PATH) -> ExperimentParameters:
    """Load configured values while keeping provenance metadata in YAML."""
    with Path(config_path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    joints = config["joints"]
    joint_names = tuple(joints)
    if joint_names != tuple(f"openarm_left_joint{i}" for i in range(1, 8)):
        raise ValueError(f"Configuration has unexpected joint order: {joint_names}")

    experiment = config["experiment"]
    return ExperimentParameters(
        joint_names=joint_names,
        hard_lower=np.asarray(
            [_value(joints[name]["position_min"]) for name in joint_names]
        ),
        hard_upper=np.asarray(
            [_value(joints[name]["position_max"]) for name in joint_names]
        ),
        rated_torque=np.asarray(
            [_value(joints[name]["rated_torque"]) for name in joint_names]
        ),
        peak_torque=np.asarray(
            [_value(joints[name]["peak_torque"]) for name in joint_names]
        ),
        rated_velocity=np.asarray(
            [_value(joints[name]["rated_velocity"]) for name in joint_names]
        ),
        control_frequency=float(_value(experiment["control_frequency"])),
        trajectory_duration=float(_value(experiment["trajectory_duration"])),
        hold_duration=float(_value(experiment["endpoint_hold_duration"])),
        start_position=np.asarray(_value(experiment["start_position"]), dtype=float),
        target_offset=np.asarray(_value(experiment["target_offset"]), dtype=float),
        kp=np.asarray(_value(experiment["controller_kp"]), dtype=float),
        kd=np.asarray(_value(experiment["controller_kd"]), dtype=float),
        planning_margin=float(_value(experiment["planning_limit_margin"])),
        normal_torque_fraction=float(
            _value(experiment["normal_torque_derating_fraction"])
        ),
        hard_velocity_fraction=float(
            _value(experiment["hard_velocity_fraction_of_rated"])
        ),
        trajectory_velocity_fraction=float(
            _value(experiment["trajectory_velocity_fraction_of_rated"])
        ),
        desired_position_step_limit=np.asarray(
            _value(experiment["desired_position_step_limit"]), dtype=float
        ),
        torque_rate_limit=np.asarray(
            _value(experiment["torque_rate_limit"]), dtype=float
        ),
        tracking_divergence=float(_value(experiment["tracking_divergence_threshold"])),
        tracking_persistence_samples=int(
            _value(experiment["tracking_divergence_persistence_samples"])
        ),
        command_timeout=float(_value(experiment["command_watchdog_timeout"])),
        feedback_timeout=float(_value(experiment["feedback_watchdog_timeout"])),
        timestep_tolerance_fraction=float(
            _value(experiment["timestep_tolerance_fraction"])
        ),
        safe_behavior=str(_value(experiment["safe_simulated_behavior"])),
        endpoint_tolerance=float(_value(experiment["endpoint_error_tolerance"])),
        actuation_latency=float(_value(experiment["test_actuation_latency"])),
        position_noise_standard_deviation=float(
            _value(experiment["position_noise_standard_deviation"])
        ),
        velocity_noise_standard_deviation=float(
            _value(experiment["velocity_noise_standard_deviation"])
        ),
        random_seed=int(_value(experiment["random_seed"])),
    )


def load_regression_policy(
    config_path: Path = DEFAULT_CONFIG_PATH,
) -> dict[str, float]:
    """Load classified broad regression thresholds without duplicating values."""
    with Path(config_path).open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    return {name: float(_value(node)) for name, node in config["regression"].items()}


def load_comparison_thresholds(
    config_path: Path = DEFAULT_CONFIG_PATH,
) -> dict[str, float]:
    """Load classified comparison assessment thresholds."""
    with Path(config_path).open(encoding="utf-8") as stream:
        experiment = yaml.safe_load(stream)["experiment"]
    return {
        "material_rms_ratio": float(
            _value(experiment["comparison_material_rms_ratio"])
        ),
        "material_rms_absolute_increase_rad": float(
            _value(experiment["comparison_material_rms_absolute_increase"])
        ),
        "oscillation_zero_crossing_increase": float(
            _value(experiment["comparison_oscillation_zero_crossing_increase"])
        ),
        "oscillation_maximum_hold_velocity_rad_s": float(
            _value(experiment["comparison_oscillation_hold_velocity"])
        ),
        "material_overshoot_increase_rad": float(
            _value(experiment["comparison_material_overshoot_increase"])
        ),
    }


class ActuationDelay:
    """Fixed-step FIFO that delays constrained actuator torque commands."""

    def __init__(self, latency_seconds: float, simulation_timestep: float) -> None:
        if not np.isfinite(latency_seconds) or latency_seconds < 0:
            raise ValueError("Actuation latency must be finite and non-negative")
        if not np.isfinite(simulation_timestep) or simulation_timestep <= 0:
            raise ValueError("Simulation timestep must be finite and positive")
        exact_steps = latency_seconds / simulation_timestep
        self.queue_length = round(exact_steps)
        if not np.isclose(exact_steps, self.queue_length, atol=1e-12, rtol=0):
            raise ValueError(
                "Actuation latency must be an integer number of simulation steps"
            )
        self.implemented_latency_seconds = self.queue_length * simulation_timestep
        if not np.isclose(
            self.implemented_latency_seconds, latency_seconds, atol=1e-12, rtol=0
        ):
            raise ValueError("Implemented latency differs from requested latency")
        self._commands: deque[np.ndarray] = deque(
            [np.zeros(7, dtype=float) for _ in range(self.queue_length)]
        )

    def push(self, command: np.ndarray) -> np.ndarray:
        """Enqueue the current command and return the command due at the actuator."""
        command = np.asarray(command, dtype=float)
        if command.shape != (7,) or not np.all(np.isfinite(command)):
            raise ValueError(
                "Delayed actuator command must contain seven finite values"
            )
        if self.queue_length == 0:
            return command.copy()
        delayed = self._commands.popleft()
        self._commands.append(command.copy())
        return delayed


def build_safety_config(
    parameters: ExperimentParameters, simulation_timestep: float
) -> SafetyConfig:
    """Build safety policy from classified configuration values."""
    return SafetyConfig(
        planning=PlanningLimits(
            position_lower=parameters.planning_lower,
            position_upper=parameters.planning_upper,
            velocity=parameters.trajectory_velocity_limit,
            max_position_step=parameters.desired_position_step_limit,
        ),
        normal=NormalCommandLimits(
            torque=parameters.normal_torque,
            torque_rate=parameters.torque_rate_limit,
        ),
        absolute=AbsoluteFaultLimits(
            position_lower=parameters.hard_lower,
            position_upper=parameters.hard_upper,
            velocity=parameters.hard_velocity_limit,
            torque=parameters.peak_torque,
            tracking_error=np.full(7, parameters.tracking_divergence),
        ),
        temporal=TemporalSafetyConfig(
            nominal_timestep=simulation_timestep,
            timestep_tolerance_fraction=parameters.timestep_tolerance_fraction,
            tracking_persistence_samples=parameters.tracking_persistence_samples,
            command_timeout=parameters.command_timeout,
            feedback_timeout=parameters.feedback_timeout,
        ),
        safe_behavior=parameters.safe_behavior,
    )


def _set_initial_state(
    data: mujoco.MjData, mapping: ArmMapping, start_position: np.ndarray
) -> None:
    data.qpos[mapping.qpos_addresses] = start_position
    data.qvel[mapping.dof_addresses] = 0.0
    data.ctrl[mapping.actuator_ids] = 0.0


def _selected_arm_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[str]:
    """Return contacts involving selected arm-chain geoms, excluding fingers."""
    prefixes = (
        "base_link_left",
        "link1_left",
        "link2_left",
        "link3_left",
        "link4_left",
        "link5_left",
        "link6_left",
        "ee_base_link_left",
    )
    contacts: list[str] = []
    for index in range(data.ncon):
        contact = data.contact[index]
        first = model.geom(int(contact.geom1)).name or "unnamed"
        second = model.geom(int(contact.geom2)).name or "unnamed"
        if first.startswith(prefixes) or second.startswith(prefixes):
            contacts.append(f"{first} <-> {second}")
    return contacts


def _log_row(
    *,
    time_s: float,
    controller_state: str,
    mapping: ArmMapping,
    desired_position: np.ndarray,
    measured_position: np.ndarray,
    sensor_position: np.ndarray,
    desired_velocity: np.ndarray,
    measured_velocity: np.ndarray,
    sensor_velocity: np.ndarray,
    feedback_torque: np.ndarray,
    bias_torque: np.ndarray,
    requested_torque: np.ndarray,
    constrained_torque: np.ndarray,
    applied_torque: np.ndarray,
    saturated: np.ndarray,
    normal_saturated: np.ndarray,
    rate_limited: np.ndarray,
    position_margin: np.ndarray,
    velocity_margin: np.ndarray,
    normal_torque_limit: np.ndarray,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "time": time_s,
        "controller_state": controller_state,
        "safety_fault_state": "none",
    }
    values = (
        ("q_des", desired_position),
        ("q", measured_position),
        ("q_sensor", sensor_position),
        ("dq_des", desired_velocity),
        ("dq", measured_velocity),
        ("dq_sensor", sensor_velocity),
        ("tau_feedback", feedback_torque),
        ("tau_bias", bias_torque),
        ("tau_requested", requested_torque),
        ("tau_command", constrained_torque),
        ("tau_applied", applied_torque),
        ("torque_saturated", saturated),
        ("normal_torque_saturated", normal_saturated),
        ("torque_rate_limited", rate_limited),
        ("position_limit_margin", position_margin),
        ("velocity_limit_margin", velocity_margin),
        ("normal_torque_limit", normal_torque_limit),
    )
    for signal, vector in values:
        for joint_name, value in zip(mapping.joint_names, vector, strict=True):
            row[signal_column(signal, joint_name)] = (
                bool(value)
                if signal
                in {
                    "torque_saturated",
                    "normal_torque_saturated",
                    "torque_rate_limited",
                }
                else float(value)
            )
    return row


def run_experiment(
    output_prefix: Path,
    *,
    mode: str,
    viewer: bool = False,
    make_plots: bool = True,
    write_artifacts: bool = True,
    config_path: Path = DEFAULT_CONFIG_PATH,
    trajectory_duration: float | None = None,
    hold_duration: float | None = None,
) -> dict[str, Any]:
    """Run one deterministic experiment through the shared control path."""
    if mode not in {"baseline", "latency_noise"}:
        raise ValueError(f"Unsupported experiment mode: {mode}")
    parameters = load_parameters(config_path)
    trajectory_duration = (
        parameters.trajectory_duration
        if trajectory_duration is None
        else float(trajectory_duration)
    )
    hold_duration = (
        parameters.hold_duration if hold_duration is None else float(hold_duration)
    )
    if trajectory_duration <= 0 or hold_duration <= 0:
        raise ValueError("Trajectory and hold durations must be positive")
    model, model_path = load_bimanual_model()
    mapping = resolve_arm_mapping(model)
    if mapping.joint_names != parameters.joint_names:
        raise ValueError("Model/config joint ordering mismatch")

    expected_timestep = 1.0 / parameters.control_frequency
    if not np.isclose(model.opt.timestep, expected_timestep, atol=1e-12, rtol=0):
        raise ValueError(
            f"Model timestep {model.opt.timestep} does not match configured "
            f"control frequency {parameters.control_frequency} Hz"
        )
    model_ranges = model.jnt_range[mapping.joint_ids]
    if not np.allclose(
        model_ranges[:, 0], parameters.hard_lower, atol=5e-5
    ) or not np.allclose(model_ranges[:, 1], parameters.hard_upper, atol=5e-5):
        raise ValueError("Configured hard position limits do not match selected model")

    trajectory = QuinticTrajectory(
        parameters.start_position,
        parameters.start_position + parameters.target_offset,
        trajectory_duration,
    )
    trajectory.validate_limits(parameters.planning_lower, parameters.planning_upper)
    controller = JointSpacePD(parameters.kp, parameters.kd)
    limits = build_safety_config(parameters, expected_timestep)
    safety = SafetyMonitor(limits)
    delay = ActuationDelay(
        parameters.actuation_latency if mode == "latency_noise" else 0.0,
        model.opt.timestep,
    )
    random_generator = (
        np.random.default_rng(parameters.random_seed)
        if mode == "latency_noise"
        else None
    )
    adapt_selected_actuators_to_torque(model, mapping, parameters.normal_torque)
    data = mujoco.MjData(model)
    _set_initial_state(data, mapping, parameters.start_position)
    mujoco.mj_forward(model, data)
    arm_contacts = _selected_arm_contacts(model, data)
    if arm_contacts:
        raise ValueError(
            f"Initial selected arm configuration has contacts: {arm_contacts}"
        )
    verify_torque_interface(model, data, mapping, parameters.normal_torque)
    safety.verify_mapping(model, mapping, 0.0)

    total_duration = trajectory_duration + hold_duration
    step_count = round(total_duration / model.opt.timestep)
    rows: list[dict[str, Any]] = []

    viewer_context: Any = nullcontext(None)
    if viewer:
        from mujoco import viewer as mujoco_viewer

        viewer_context = mujoco_viewer.launch_passive(model, data)

    with viewer_context as viewer_handle:
        try:
            for step in range(step_count + 1):
                time_s = step * model.opt.timestep
                mujoco.mj_forward(model, data)
                true_position, true_velocity = mapping.read_state(data)
                if random_generator is None:
                    sensor_position = true_position
                    sensor_velocity = true_velocity
                else:
                    sensor_position = true_position + random_generator.normal(
                        0.0, parameters.position_noise_standard_deviation, size=7
                    )
                    sensor_velocity = true_velocity + random_generator.normal(
                        0.0, parameters.velocity_noise_standard_deviation, size=7
                    )
                desired = trajectory.sample(time_s)
                command = controller.compute(
                    desired.position,
                    desired.velocity,
                    sensor_position,
                    sensor_velocity,
                    mapping.read_bias(data),
                )
                snapshot = safety.evaluate(
                    simulation_time=time_s,
                    command_timestamp=time_s,
                    feedback_timestamp=time_s,
                    simulation_timestep=model.opt.timestep,
                    desired_position=desired.position,
                    desired_velocity=desired.velocity,
                    measured_position=sensor_position,
                    measured_velocity=sensor_velocity,
                    feedback_torque=command.feedback,
                    bias_torque=command.bias,
                    requested_torque=command.requested,
                )
                actuator_command = delay.push(snapshot.applied_torque)
                data.ctrl[mapping.actuator_ids] = actuator_command
                mujoco.mj_forward(model, data)
                actual_applied = data.actuator_force[mapping.actuator_ids].copy()
                safety.verify_applied_torque(
                    actual_applied, time_s, expected=actuator_command
                )

                rows.append(
                    _log_row(
                        time_s=time_s,
                        controller_state=(
                            "tracking" if time_s < trajectory_duration else "holding"
                        ),
                        mapping=mapping,
                        desired_position=desired.position,
                        measured_position=true_position,
                        sensor_position=sensor_position,
                        desired_velocity=desired.velocity,
                        measured_velocity=true_velocity,
                        sensor_velocity=sensor_velocity,
                        feedback_torque=command.feedback,
                        bias_torque=command.bias,
                        requested_torque=command.requested,
                        constrained_torque=snapshot.applied_torque,
                        applied_torque=actual_applied,
                        saturated=snapshot.torque_saturated,
                        normal_saturated=snapshot.normal_torque_saturated,
                        rate_limited=snapshot.torque_rate_limited,
                        position_margin=snapshot.position_limit_margin,
                        velocity_margin=snapshot.velocity_limit_margin,
                        normal_torque_limit=parameters.normal_torque,
                    )
                )
                if step < step_count:
                    mujoco.mj_step(model, data)
                    if viewer_handle is not None:
                        viewer_handle.sync()
                        time.sleep(model.opt.timestep)
        except SafetyFault as exc:
            data.ctrl[mapping.actuator_ids] = exc.safe_torque
            mujoco.mj_forward(model, data)
            if write_artifacts:
                fault_path = (
                    Path(output_prefix).parent
                    / f"{Path(output_prefix).name}_fault.json"
                )
                fault_path.parent.mkdir(parents=True, exist_ok=True)
                fault_path.write_text(
                    json.dumps(
                        {
                            "state": "FAULT",
                            "reason": exc.rule,
                            "timestamp": exc.timestamp,
                            "detail": exc.detail,
                            "active_trajectory": safety.active_trajectory,
                            "safe_simulated_behavior": parameters.safe_behavior,
                            "safe_torque_nm": exc.safe_torque.tolist(),
                        },
                        indent=2,
                        sort_keys=True,
                    )
                    + "\n",
                    encoding="utf-8",
                )
            raise

    metrics = write_outputs(
        rows,
        mapping.joint_names,
        Path(output_prefix),
        trajectory_duration=trajectory_duration,
        endpoint_tolerance=parameters.endpoint_tolerance,
        safety_violations=safety.violation_count,
        nonfinite_samples=safety.nonfinite_sample_count,
        make_plots=make_plots,
        write_artifacts=write_artifacts,
        scenario_name=mode.replace("_", " + "),
    )
    metrics["model_path"] = str(model_path)
    metrics["joint_names"] = list(mapping.joint_names)
    metrics["actuator_names"] = list(mapping.actuator_names)
    metrics["controller_kp"] = parameters.kp.tolist()
    metrics["controller_kd"] = parameters.kd.tolist()
    metrics["normal_torque_limits_nm"] = parameters.normal_torque.tolist()
    metrics["hard_torque_limits_nm"] = parameters.peak_torque.tolist()
    metrics["mode"] = mode
    metrics["random_seed"] = parameters.random_seed
    metrics["requested_actuation_latency_ms"] = (
        1000.0 * parameters.actuation_latency if mode == "latency_noise" else 0.0
    )
    metrics["latency_queue_length_samples"] = delay.queue_length
    metrics["implemented_actuation_latency_ms"] = (
        1000.0 * delay.implemented_latency_seconds
    )
    metrics["position_measurement_noise_standard_deviation_rad"] = (
        parameters.position_noise_standard_deviation if mode == "latency_noise" else 0.0
    )
    metrics["velocity_measurement_noise_standard_deviation_rad_s"] = (
        parameters.velocity_noise_standard_deviation if mode == "latency_noise" else 0.0
    )
    metrics["measurement_delay_ms"] = 0.0
    metrics["trajectory_duration_s"] = trajectory_duration
    metrics["endpoint_hold_duration_s"] = hold_duration
    if write_artifacts:
        metrics_path = (
            Path(output_prefix).parent / f"{Path(output_prefix).name}_metrics.json"
        )
        metrics_path.write_text(
            json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return metrics


def run_baseline(
    output_prefix: Path,
    *,
    viewer: bool = False,
    make_plots: bool = True,
    write_artifacts: bool = True,
    config_path: Path = DEFAULT_CONFIG_PATH,
    trajectory_duration: float | None = None,
    hold_duration: float | None = None,
) -> dict[str, Any]:
    """Run the exact no-latency, no-noise baseline mode."""
    return run_experiment(
        output_prefix,
        mode="baseline",
        viewer=viewer,
        make_plots=make_plots,
        write_artifacts=write_artifacts,
        config_path=config_path,
        trajectory_duration=trajectory_duration,
        hold_duration=hold_duration,
    )


def run_latency_noise(
    output_prefix: Path,
    *,
    viewer: bool = False,
    make_plots: bool = True,
    write_artifacts: bool = True,
    config_path: Path = DEFAULT_CONFIG_PATH,
    trajectory_duration: float | None = None,
    hold_duration: float | None = None,
) -> dict[str, Any]:
    """Run 8 ms actuator-command latency with noisy, non-delayed measurements."""
    return run_experiment(
        output_prefix,
        mode="latency_noise",
        viewer=viewer,
        make_plots=make_plots,
        write_artifacts=write_artifacts,
        config_path=config_path,
        trajectory_duration=trajectory_duration,
        hold_duration=hold_duration,
    )


def run_comparison(
    output_directory: Path,
    *,
    make_plots: bool = True,
    config_path: Path = DEFAULT_CONFIG_PATH,
) -> dict[str, Any]:
    """Run both full scenarios and write their measured comparison."""
    output_directory = Path(output_directory)
    baseline_prefix = output_directory / "baseline"
    latency_prefix = output_directory / "latency_noise"
    baseline_metrics = run_baseline(
        baseline_prefix, make_plots=make_plots, config_path=config_path
    )
    latency_metrics = run_latency_noise(
        latency_prefix, make_plots=make_plots, config_path=config_path
    )
    return write_comparison_outputs(
        pd.read_csv(baseline_prefix.with_suffix(".csv")),
        pd.read_csv(latency_prefix.with_suffix(".csv")),
        baseline_metrics,
        latency_metrics,
        tuple(baseline_metrics["joint_names"]),
        output_directory,
        load_comparison_thresholds(config_path),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("baseline", "latency_noise", "compare"),
        default="baseline",
    )
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--headless", action="store_true", help="Run without a viewer")
    display.add_argument(
        "--viewer", action="store_true", help="Open a passive MuJoCo viewer"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output prefix, or output directory for compare mode",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.mode == "compare":
            if args.viewer:
                raise ValueError("Compare mode is headless-only")
            metrics = run_comparison(
                args.output or Path("results"), config_path=args.config
            )
        elif args.mode == "latency_noise":
            metrics = run_latency_noise(
                args.output or Path("results/latency_noise"),
                viewer=args.viewer,
                config_path=args.config,
            )
        else:
            metrics = run_baseline(
                args.output or Path("results/baseline"),
                viewer=args.viewer,
                config_path=args.config,
            )
    except SafetyFault as exc:
        print(f"SAFETY FAULT: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:  # noqa: BLE001
        print(f"Experiment failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(metrics, indent=2, sort_keys=True))
    if args.mode == "compare":
        return 0 if metrics["assessment"]["remained_stable"] else 1
    return 0 if metrics["experiment_completion_status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

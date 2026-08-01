"""Deterministic OpenArm v2 left-arm joint-space tracking experiment."""

from __future__ import annotations

import argparse
import json
import sys
import time
from contextlib import nullcontext
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np
import yaml

from .controller import JointSpacePD
from .metrics import signal_column, write_outputs
from .model_mapping import (
    ArmMapping,
    adapt_selected_actuators_to_torque,
    load_bimanual_model,
    resolve_arm_mapping,
    verify_torque_interface,
)
from .safety import SafetyFault, SafetyLimits, SafetyMonitor
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
    velocity_fraction: float
    tracking_divergence: float
    endpoint_tolerance: float
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
    def velocity_limit(self) -> np.ndarray:
        return self.rated_velocity * self.velocity_fraction


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
        velocity_fraction=float(
            _value(experiment["operational_velocity_fraction_of_rated"])
        ),
        tracking_divergence=float(_value(experiment["tracking_divergence_threshold"])),
        endpoint_tolerance=float(_value(experiment["endpoint_error_tolerance"])),
        random_seed=int(_value(experiment["random_seed"])),
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
    desired_velocity: np.ndarray,
    measured_velocity: np.ndarray,
    feedback_torque: np.ndarray,
    bias_torque: np.ndarray,
    requested_torque: np.ndarray,
    applied_torque: np.ndarray,
    saturated: np.ndarray,
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
        ("dq_des", desired_velocity),
        ("dq", measured_velocity),
        ("tau_feedback", feedback_torque),
        ("tau_bias", bias_torque),
        ("tau_requested", requested_torque),
        ("tau_applied", applied_torque),
        ("torque_saturated", saturated),
        ("position_limit_margin", position_margin),
        ("velocity_limit_margin", velocity_margin),
        ("normal_torque_limit", normal_torque_limit),
    )
    for signal, vector in values:
        for joint_name, value in zip(mapping.joint_names, vector, strict=True):
            row[signal_column(signal, joint_name)] = (
                bool(value) if signal == "torque_saturated" else float(value)
            )
    return row


def run_baseline(
    output_prefix: Path,
    *,
    viewer: bool = False,
    make_plots: bool = True,
    config_path: Path = DEFAULT_CONFIG_PATH,
) -> dict[str, Any]:
    """Run the deterministic baseline and write the requested artifacts."""
    parameters = load_parameters(config_path)
    np.random.seed(parameters.random_seed)
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
        parameters.trajectory_duration,
    )
    trajectory.validate_limits(parameters.planning_lower, parameters.planning_upper)
    controller = JointSpacePD(parameters.kp, parameters.kd)
    limits = SafetyLimits(
        hard_position_lower=parameters.hard_lower,
        hard_position_upper=parameters.hard_upper,
        planning_position_lower=parameters.planning_lower,
        planning_position_upper=parameters.planning_upper,
        velocity=parameters.velocity_limit,
        normal_torque=parameters.normal_torque,
        hard_torque=parameters.peak_torque,
        tracking_divergence=parameters.tracking_divergence,
    )
    safety = SafetyMonitor(limits)
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

    total_duration = parameters.trajectory_duration + parameters.hold_duration
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
                measured_position, measured_velocity = mapping.read_state(data)
                desired = trajectory.sample(time_s)
                command = controller.compute(
                    desired.position,
                    desired.velocity,
                    measured_position,
                    measured_velocity,
                    mapping.read_bias(data),
                )
                snapshot = safety.evaluate(
                    desired_position=desired.position,
                    desired_velocity=desired.velocity,
                    measured_position=measured_position,
                    measured_velocity=measured_velocity,
                    feedback_torque=command.feedback,
                    bias_torque=command.bias,
                    requested_torque=command.requested,
                )
                data.ctrl[mapping.actuator_ids] = snapshot.applied_torque
                mujoco.mj_forward(model, data)
                actual_applied = data.actuator_force[mapping.actuator_ids].copy()
                if not np.allclose(actual_applied, snapshot.applied_torque, atol=1e-12):
                    raise SafetyFault(
                        "actuator_force_mismatch",
                        "applied actuator force differs from clamped torque command",
                    )

                rows.append(
                    _log_row(
                        time_s=time_s,
                        controller_state=(
                            "tracking"
                            if time_s < parameters.trajectory_duration
                            else "holding"
                        ),
                        mapping=mapping,
                        desired_position=desired.position,
                        measured_position=measured_position,
                        desired_velocity=desired.velocity,
                        measured_velocity=measured_velocity,
                        feedback_torque=command.feedback,
                        bias_torque=command.bias,
                        requested_torque=command.requested,
                        applied_torque=actual_applied,
                        saturated=snapshot.torque_saturated,
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
        except SafetyFault:
            data.ctrl[mapping.actuator_ids] = 0.0
            raise

    metrics = write_outputs(
        rows,
        mapping.joint_names,
        Path(output_prefix),
        trajectory_duration=parameters.trajectory_duration,
        endpoint_tolerance=parameters.endpoint_tolerance,
        safety_violations=safety.violation_count,
        nonfinite_samples=safety.nonfinite_sample_count,
        make_plots=make_plots,
    )
    metrics["model_path"] = str(model_path)
    metrics["joint_names"] = list(mapping.joint_names)
    metrics["actuator_names"] = list(mapping.actuator_names)
    metrics["controller_kp"] = parameters.kp.tolist()
    metrics["controller_kd"] = parameters.kd.tolist()
    metrics["normal_torque_limits_nm"] = parameters.normal_torque.tolist()
    metrics["hard_torque_limits_nm"] = parameters.peak_torque.tolist()
    metrics_path = (
        Path(output_prefix).parent / f"{Path(output_prefix).name}_metrics.json"
    )
    metrics_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metrics


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("baseline",), default="baseline")
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--headless", action="store_true", help="Run without a viewer")
    display.add_argument(
        "--viewer", action="store_true", help="Open a passive MuJoCo viewer"
    )
    parser.add_argument(
        "--output", type=Path, default=Path("results/baseline"), help="Output prefix"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        metrics = run_baseline(
            args.output,
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
    return 0 if metrics["experiment_completion_status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

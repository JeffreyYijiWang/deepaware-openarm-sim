"""Experiment metrics, CSV output, and deterministic headless plots."""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
from matplotlib import pyplot as plt

if TYPE_CHECKING:
    from collections.abc import Sequence


def signal_column(prefix: str, joint_name: str) -> str:
    """Return the stable CSV column name for a per-joint signal."""
    return f"{prefix}_{joint_name}"


def calculate_metrics(
    frame: pd.DataFrame,
    joint_names: Sequence[str],
    *,
    trajectory_duration: float,
    endpoint_tolerance: float,
    safety_violations: int,
    nonfinite_samples: int,
) -> dict[str, Any]:
    """Calculate all requested tracking and safety metrics."""
    desired = frame[[signal_column("q_des", name) for name in joint_names]].to_numpy()
    actual = frame[[signal_column("q", name) for name in joint_names]].to_numpy()
    velocity = frame[[signal_column("dq", name) for name in joint_names]].to_numpy()
    requested = frame[
        [signal_column("tau_requested", name) for name in joint_names]
    ].to_numpy()
    applied = frame[
        [signal_column("tau_applied", name) for name in joint_names]
    ].to_numpy()
    saturated = frame[
        [signal_column("torque_saturated", name) for name in joint_names]
    ].to_numpy(dtype=bool)
    error = desired - actual

    rms_per_joint = np.sqrt(np.mean(error**2, axis=0))
    maximum_per_joint = np.max(np.abs(error), axis=0)
    final_per_joint = error[-1]
    max_velocity_per_joint = np.max(np.abs(velocity), axis=0)
    max_requested_per_joint = np.max(np.abs(requested), axis=0)
    max_applied_per_joint = np.max(np.abs(applied), axis=0)
    saturated_control_samples = np.any(saturated, axis=1)
    hold_mask = frame["time"].to_numpy() >= trajectory_duration
    hold_velocity = velocity[hold_mask]
    error_norm = np.linalg.norm(error, axis=1)
    peak_error_norm = float(np.max(error_norm))
    final_error_norm = float(error_norm[-1])
    reduction = (
        0.0 if peak_error_norm == 0 else 1.0 - final_error_norm / peak_error_norm
    )

    completion = (
        safety_violations == 0
        and nonfinite_samples == 0
        and bool(np.all(np.abs(final_per_joint) <= endpoint_tolerance))
    )
    return {
        "rms_position_error_per_joint_rad": dict(
            zip(joint_names, rms_per_joint.tolist(), strict=True)
        ),
        "overall_rms_position_error_rad": float(np.sqrt(np.mean(error**2))),
        "maximum_position_error_per_joint_rad": dict(
            zip(joint_names, maximum_per_joint.tolist(), strict=True)
        ),
        "final_position_error_per_joint_rad": dict(
            zip(joint_names, final_per_joint.tolist(), strict=True)
        ),
        "maximum_measured_velocity_per_joint_rad_s": dict(
            zip(joint_names, max_velocity_per_joint.tolist(), strict=True)
        ),
        "maximum_measured_velocity_rad_s": float(np.max(np.abs(velocity))),
        "maximum_requested_torque_per_joint_nm": dict(
            zip(joint_names, max_requested_per_joint.tolist(), strict=True)
        ),
        "maximum_requested_torque_nm": float(np.max(np.abs(requested))),
        "maximum_applied_torque_per_joint_nm": dict(
            zip(joint_names, max_applied_per_joint.tolist(), strict=True)
        ),
        "maximum_applied_torque_nm": float(np.max(np.abs(applied))),
        "torque_saturated_samples": int(np.count_nonzero(saturated_control_samples)),
        "torque_saturated_samples_percent": float(
            100.0 * np.mean(saturated_control_samples)
        ),
        "torque_saturated_joint_samples": int(np.count_nonzero(saturated)),
        "torque_saturated_samples_per_joint": dict(
            zip(joint_names, np.count_nonzero(saturated, axis=0).tolist(), strict=True)
        ),
        "safety_violations": int(safety_violations),
        "nonfinite_samples": int(nonfinite_samples),
        "experiment_completion_status": "completed" if completion else "failed",
        "endpoint_tolerance_rad": float(endpoint_tolerance),
        "peak_tracking_error_norm_rad": peak_error_norm,
        "final_tracking_error_norm_rad": final_error_norm,
        "peak_to_final_error_reduction_fraction": float(reduction),
        "maximum_hold_velocity_rad_s": float(np.max(np.abs(hold_velocity))),
        "sample_count": len(frame),
    }


def _plot_tracking(frame: pd.DataFrame, joint_names: Sequence[str], path: Path) -> None:
    figure, axes = plt.subplots(4, 2, figsize=(12, 12), sharex=True)
    for axis, joint_name in zip(axes.flat, joint_names, strict=False):
        axis.plot(
            frame["time"],
            frame[signal_column("q_des", joint_name)],
            "--",
            label="desired",
        )
        axis.plot(frame["time"], frame[signal_column("q", joint_name)], label="actual")
        axis.set_title(joint_name)
        axis.set_ylabel("position [rad]")
        axis.grid(True, alpha=0.3)
    axes.flat[-1].axis("off")
    axes.flat[0].legend(loc="best")
    axes[-1, 0].set_xlabel("time [s]")
    figure.suptitle("OpenArm v2 left-arm baseline tracking")
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def _plot_torque(frame: pd.DataFrame, joint_names: Sequence[str], path: Path) -> None:
    figure, axes = plt.subplots(4, 2, figsize=(12, 12), sharex=True)
    for axis, joint_name in zip(axes.flat, joint_names, strict=False):
        axis.plot(
            frame["time"],
            frame[signal_column("tau_requested", joint_name)],
            "--",
            label="requested",
        )
        axis.plot(
            frame["time"],
            frame[signal_column("tau_applied", joint_name)],
            label="applied",
        )
        limit = float(frame[signal_column("normal_torque_limit", joint_name)].iloc[0])
        axis.axhline(limit, color="black", linewidth=0.7, alpha=0.5)
        axis.axhline(-limit, color="black", linewidth=0.7, alpha=0.5)
        axis.set_title(joint_name)
        axis.set_ylabel("torque [N m]")
        axis.grid(True, alpha=0.3)
    axes.flat[-1].axis("off")
    axes.flat[0].legend(loc="best")
    axes[-1, 0].set_xlabel("time [s]")
    figure.suptitle("OpenArm v2 left-arm requested and applied torque")
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def write_outputs(
    rows: list[dict[str, Any]],
    joint_names: Sequence[str],
    output_prefix: Path,
    *,
    trajectory_duration: float,
    endpoint_tolerance: float,
    safety_violations: int,
    nonfinite_samples: int,
    make_plots: bool = True,
) -> dict[str, Any]:
    """Write CSV, JSON metrics, and the two required plots."""
    output_prefix = Path(output_prefix)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    frame = pd.DataFrame.from_records(rows)
    metrics = calculate_metrics(
        frame,
        joint_names,
        trajectory_duration=trajectory_duration,
        endpoint_tolerance=endpoint_tolerance,
        safety_violations=safety_violations,
        nonfinite_samples=nonfinite_samples,
    )
    csv_path = output_prefix.with_suffix(".csv")
    metrics_path = output_prefix.parent / f"{output_prefix.name}_metrics.json"
    tracking_path = output_prefix.parent / f"{output_prefix.name}_tracking.png"
    torque_path = output_prefix.parent / f"{output_prefix.name}_torque.png"
    frame.to_csv(csv_path, index=False)
    metrics_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if make_plots:
        _plot_tracking(frame, joint_names, tracking_path)
        _plot_torque(frame, joint_names, torque_path)
    return metrics

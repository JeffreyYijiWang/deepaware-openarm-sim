"""Headless installation smoke test for the OpenArm v2 MuJoCo model."""

from __future__ import annotations

import sys
from pathlib import Path


def main() -> int:
    """Load the bimanual model and advance its physics by one safe step."""
    stage = "importing MuJoCo"
    try:
        import mujoco

        stage = "importing openarm_mujoco.v2"
        from openarm_mujoco import v2

        stage = "resolving OpenArm v2 asset paths"
        bimanual_path = Path(v2.openarm_bimanual_xml()).resolve()
        cell_path = Path(v2.openarm_cell_xml()).resolve()

        for asset_name, asset_path in (
            ("bimanual model", bimanual_path),
            ("cell model", cell_path),
        ):
            if not asset_path.is_file():
                raise FileNotFoundError(f"{asset_name} does not exist: {asset_path}")

        stage = "loading the bimanual MuJoCo model"
        model = mujoco.MjModel.from_xml_path(str(bimanual_path))

        stage = "creating MuJoCo data"
        data = mujoco.MjData(model)

        stage = "running mj_forward"
        mujoco.mj_forward(model, data)

        stage = "running one mj_step"
        mujoco.mj_step(model, data)
    # This executable must turn every setup/runtime failure into a clear exit 1.
    except Exception as exc:  # noqa: BLE001
        print(f"OpenArm smoke test failed while {stage}: {exc}", file=sys.stderr)
        return 1

    print(f"MuJoCo version: {mujoco.__version__}")
    print(f"Bimanual model path: {bimanual_path}")
    print(f"Cell model path: {cell_path}")
    print(f"nq: {model.nq}")
    print(f"nv: {model.nv}")
    print(f"nu: {model.nu}")
    print(f"timestep: {model.opt.timestep}")
    print("Model load success: true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

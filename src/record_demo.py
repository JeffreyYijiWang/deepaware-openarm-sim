"""Render a short, evidence-backed OpenArm demo from generated baseline data."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

from .metrics import signal_column
from .model_mapping import load_bimanual_model, resolve_arm_mapping

if TYPE_CHECKING:
    from collections.abc import Iterator, Sequence


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = REPOSITORY_ROOT / "results"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 30


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = (
        "C:/Windows/Fonts/arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for candidate in candidates:
        if Path(candidate).is_file():
            return ImageFont.truetype(candidate, size=size)
    return ImageFont.load_default()


def _title_frame(frame: np.ndarray, title: str, subtitle: str) -> np.ndarray:
    image = Image.fromarray(frame).convert("RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rectangle((0, 0, image.width, 92), fill=(8, 15, 25, 220))
    draw.text((28, 15), title, font=_font(30), fill=(255, 255, 255, 255))
    draw.text((29, 55), subtitle, font=_font(18), fill=(180, 210, 235, 255))
    return np.asarray(image)


def _slide(path: Path, title: str, width: int, height: int) -> np.ndarray:
    source = Image.open(path).convert("RGB")
    canvas = Image.new("RGB", (width, height), (10, 17, 27))
    source.thumbnail((width - 60, height - 110), Image.Resampling.LANCZOS)
    canvas.paste(source, ((width - source.width) // 2, 85))
    draw = ImageDraw.Draw(canvas)
    draw.text((30, 22), title, font=_font(32), fill=(245, 248, 252))
    return np.asarray(canvas)


def _hardware_slide(width: int, height: int) -> np.ndarray:
    canvas = Image.new("RGB", (width, height), (10, 17, 27))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (32, 24),
        "Proposed embedded-Linux / CAN bridge",
        font=_font(32),
        fill=(245, 248, 252),
    )
    labels = [
        "Trajectory",
        "Host PD +\nsafety",
        "SocketCAN",
        "Motor MCU +\ncurrent loop",
        "Physical\njoint",
        "Encoder\nfeedback",
    ]
    margin = 45
    gap = 18
    box_width = (width - 2 * margin - gap * (len(labels) - 1)) // len(labels)
    top, bottom = 265, 430
    for index, label in enumerate(labels):
        left = margin + index * (box_width + gap)
        right = left + box_width
        draw.rounded_rectangle(
            (left, top, right, bottom),
            radius=14,
            fill=(28, 68, 105),
            outline=(103, 190, 255),
            width=3,
        )
        text_box = draw.multiline_textbbox(
            (0, 0), label, font=_font(20), align="center"
        )
        text_width = text_box[2] - text_box[0]
        text_height = text_box[3] - text_box[1]
        draw.multiline_text(
            ((left + right - text_width) / 2, (top + bottom - text_height) / 2),
            label,
            font=_font(20),
            fill=(255, 255, 255),
            align="center",
        )
        if index < len(labels) - 1:
            arrow_start = (right + 3, (top + bottom) // 2)
            arrow_end = (right + gap - 3, (top + bottom) // 2)
            draw.line((arrow_start, arrow_end), fill=(255, 190, 70), width=4)
            draw.polygon(
                (
                    arrow_end,
                    (arrow_end[0] - 10, arrow_end[1] - 7),
                    (arrow_end[0] - 10, arrow_end[1] + 7),
                ),
                fill=(255, 190, 70),
            )
    draw.text(
        (45, 515),
        "Design skeleton only — independent E-stop, power isolation, and hardware validation required",
        font=_font(22),
        fill=(255, 190, 120),
    )
    return np.asarray(canvas)


def _repeat(frame: np.ndarray, seconds: float, fps: int) -> Iterator[np.ndarray]:
    for _ in range(round(seconds * fps)):
        yield frame


def _write_video(
    frames: Iterator[np.ndarray], output: Path, width: int, height: int, fps: int
) -> None:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is unavailable; use the documented manual steps")
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        ffmpeg,
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "21",
        "-pix_fmt",
        "yuv420p",
        str(output),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE)
    assert process.stdin is not None
    try:
        for frame in frames:
            if frame.shape != (height, width, 3) or frame.dtype != np.uint8:
                raise ValueError(
                    f"Unexpected frame format: {frame.shape}, {frame.dtype}"
                )
            process.stdin.write(frame.tobytes())
    finally:
        process.stdin.close()
    return_code = process.wait()
    if return_code != 0:
        raise RuntimeError(f"ffmpeg exited with status {return_code}")


def render_demo(
    output: Path,
    *,
    results_directory: Path = DEFAULT_RESULTS,
    width: int = DEFAULT_WIDTH,
    height: int = DEFAULT_HEIGHT,
    fps: int = DEFAULT_FPS,
    check_only: bool = False,
) -> None:
    """Check rendering or create the complete short demonstration."""
    model, model_path = load_bimanual_model()
    mapping = resolve_arm_mapping(model)
    source_width = int(model.vis.global_.offwidth)
    source_height = int(model.vis.global_.offheight)
    print(f"MODEL={model_path}")
    print(f"MODEL_OFFSCREEN_BUFFER={source_width}x{source_height}")
    print(f"REQUESTED_RENDER_RESOLUTION={width}x{height}")
    if width <= 0 or height <= 0 or fps <= 0:
        raise ValueError("Width, height, and FPS must be positive")
    if width % 2 or height % 2:
        raise ValueError("H.264 yuv420p output requires even width and height")
    model.vis.global_.offwidth = max(source_width, width)
    model.vis.global_.offheight = max(source_height, height)
    model.vis.headlight.ambient[:] = 0.35
    model.vis.headlight.diffuse[:] = 0.75
    model.vis.headlight.specular[:] = 0.2
    print(
        "ADJUSTED_OFFSCREEN_BUFFER="
        f"{int(model.vis.global_.offwidth)}x{int(model.vis.global_.offheight)}"
    )

    data = mujoco.MjData(model)
    frame_data = pd.read_csv(results_directory / "baseline.csv")
    positions = frame_data[
        [signal_column("q", name) for name in mapping.joint_names]
    ].to_numpy()
    times = frame_data["time"].to_numpy()
    if not np.all(np.isfinite(positions)) or not np.all(np.diff(times) > 0):
        raise ValueError("Baseline demo data must be finite with increasing time")

    renderer = mujoco.Renderer(model, height=height, width=width)
    print(f"RENDERER_RESOLUTION={renderer.width}x{renderer.height}")
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = 135.0
    camera.elevation = -12.0
    camera.distance = 0.72
    camera.lookat[:] = (0.02, 0.13, -0.18)

    def robot_frame(position: np.ndarray, title: str, subtitle: str) -> np.ndarray:
        data.qpos[mapping.qpos_addresses] = position
        data.qvel[mapping.dof_addresses] = 0.0
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        return _title_frame(renderer.render(), title, subtitle)

    initial = robot_frame(
        positions[0], "OpenArm v2 — initial pose", "Left seven-DoF arm selected by name"
    )
    if check_only:
        print("RENDER_CHECK=passed")
        renderer.close()
        return

    baseline_plot = _slide(
        results_directory / "baseline_tracking.png",
        "Baseline tracking — measured output",
        width,
        height,
    )
    comparison_plot = _slide(
        results_directory / "baseline_vs_latency.png",
        "Baseline vs 8 ms actuator latency + assumed sensor noise",
        width,
        height,
    )
    hardware = _hardware_slide(width, height)

    def frames() -> Iterator[np.ndarray]:
        yield from _repeat(initial, 1.0, fps)
        duration = float(times[-1])
        for frame_index in range(round(duration * fps) + 1):
            time_s = frame_index / fps
            sample_index = int(np.searchsorted(times, time_s, side="left"))
            sample_index = min(sample_index, len(times) - 1)
            phase = (
                "commanded quintic motion" if time_s < 5.0 else "stable endpoint hold"
            )
            yield robot_frame(
                positions[sample_index],
                "OpenArm v2 — baseline simulation",
                f"t={time_s:4.1f} s | {phase}",
            )
        final = robot_frame(
            positions[-1], "OpenArm v2 — endpoint", "One-second stable hold completed"
        )
        yield from _repeat(final, 1.0, fps)
        yield from _repeat(baseline_plot, 2.0, fps)
        yield from _repeat(comparison_plot, 2.0, fps)
        yield from _repeat(hardware, 2.0, fps)

    try:
        _write_video(frames(), Path(output), width, height, fps)
    finally:
        renderer.close()
    print(f"DEMO_VIDEO={Path(output).resolve()}")
    print(f"DEMO_RESOLUTION={width}x{height}")
    print(f"DEMO_FPS={fps}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_RESULTS / "demo.mp4")
    parser.add_argument("--results", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument("--check-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        render_demo(
            args.output,
            results_directory=args.results,
            width=args.width,
            height=args.height,
            fps=args.fps,
            check_only=args.check_only,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"Demo recording failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

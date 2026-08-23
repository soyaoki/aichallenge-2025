#!/usr/bin/env python3
"""Run driving_supercombo on synthetic frames, without ROS or the simulator.

Checks that the model file is present and matches its checksum, that the chosen
onnxruntime provider loads, and that the plan-to-action conversion produces
finite commands at a usable rate.

    python3 scripts/smoke_test.py [--provider auto] [--steps 40]
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from openpilot_controller.openpilot_controller_core import OpenPilotCore  # noqa: E402
from openpilot_controller.transforms import focal_from_hfov, intrinsics_from_focal  # noqa: E402

DEFAULT_SHA256 = '659727c4d4839adc4992a254409a54259a8756a743f2d567bf5fdc6579f8009b'


def synthetic_road(width: int, height: int, offset: float) -> np.ndarray:
    """A crude straight road that shifts sideways, enough to exercise the pipeline."""
    image = np.empty((height, width, 3), dtype=np.uint8)
    image[:height // 2] = (120, 160, 210)  # sky
    image[height // 2:] = (40, 110, 60)    # grass
    rows = np.arange(height // 2, height)
    t = (rows - height // 2) / (height / 2)
    half = (20 + t * (width / 3)).astype(int)
    centre = (width / 2 + offset * (1 - t) * (width / 4)).astype(int)
    for row, c, h in zip(rows, centre, half):
        image[row, max(0, c - h):min(width, c + h)] = (90, 90, 95)
    return image


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default=os.path.join(here, '..', 'models', 'driving_supercombo.onnx'))
    parser.add_argument('--sha256', default=DEFAULT_SHA256, help='pass "" to skip verification')
    parser.add_argument('--provider', default='auto', help='auto | cpu | cuda | tensorrt')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--hfov-deg', type=float, default=60.0)
    parser.add_argument('--steps', type=int, default=40)
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        print(f"model not found: {args.model}\nRun `make openpilot-models` first.", file=sys.stderr)
        return 1

    core = OpenPilotCore(args.model, provider=args.provider, expected_sha256=args.sha256)
    core.set_intrinsics(intrinsics_from_focal(args.width, args.height,
                                              focal_from_hfov(args.width, args.hfov_deg)))
    print(f"providers : {core.runner.active_providers}")
    print(f"checkpoint: {core.runner.model_checkpoint}")

    frames = [synthetic_road(args.width, args.height, 0.6 * np.sin(i / 8.0))
              for i in range(min(args.steps, 20))]
    for frame in frames[:5]:
        core.process(frame, v_ego=8.0)

    durations = []
    for step in range(args.steps):
        started = time.perf_counter()
        action = core.process(frames[step % len(frames)], v_ego=8.0)
        durations.append((time.perf_counter() - started) * 1000.0)
        if not (np.isfinite(action.desired_curvature) and np.isfinite(action.desired_acceleration)):
            print(f"non-finite action at step {step}: {action}", file=sys.stderr)
            return 1

    mean = float(np.mean(durations))
    print(f"latency   : {mean:.1f} ms mean, {np.max(durations):.1f} ms max -> {1000.0 / mean:.0f} Hz")
    print(f"last action: curvature {action.desired_curvature:+.5f} 1/m, "
          f"accel {action.desired_acceleration:+.2f} m/s^2, target speed {action.target_speed:.1f} m/s")
    if mean > 50.0:
        print("WARNING: slower than the model's 20 Hz design rate", file=sys.stderr)
    print("OK")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

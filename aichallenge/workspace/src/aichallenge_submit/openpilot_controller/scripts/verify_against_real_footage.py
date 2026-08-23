#!/usr/bin/env python3
"""Check this integration against real comma footage.

A bad warp, a wrong channel order or a mis-sliced output all look the same from
inside AWSIM: the model just reports nothing. Running the same pipeline on a
public comma segment, whose camera and intrinsics openpilot documents, separates
"the integration is broken" from "the track is out of distribution".

A working pipeline reports lane line probabilities around 0.9 on the ego lane
with sub-metre standard deviations. On the AI Challenge kart track the same code
reports about 0.02 with several metres of spread.

    python3 scripts/verify_against_real_footage.py
    python3 scripts/verify_against_real_footage.py --frames 60 --provider cuda

Downloads ~37 MB the first time and caches it next to the model.
"""

import argparse
import os
import subprocess
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from openpilot_controller.supercombo import SupercomboRunner  # noqa: E402
from openpilot_controller.transforms import (  # noqa: E402
    get_warp_matrix, prepare_model_frame, rgb_to_yuv_planes)

# A segment from openpilot's own CI data. The camera is the 'neo' config in
# openpilot's common/transformations/camera.py: 1164x874 at a focal length of 910.
SEGMENT_URL = ("https://commadataci.blob.core.windows.net/openpilotci/"
               "0982d79ebb0de295/2021-01-04--17-13-21/13/fcamera.hevc")
NEO_WIDTH, NEO_HEIGHT, NEO_FOCAL = 1164, 874, 910.0
NEO_FPS = 20
# Ego-lane probability below this means something in the pipeline is wrong,
# not that the scene is unusual.
EXPECTED_EGO_LANE_PROB = 0.5


def fetch(path: str) -> bool:
    if os.path.isfile(path) and os.path.getsize(path) > 1_000_000:
        return True
    print(f"downloading {SEGMENT_URL}")
    try:
        subprocess.run(["curl", "--fail", "--location", "--silent", "--show-error",
                        "--output", path, SEGMENT_URL], check=True)
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"could not download the reference segment: {exc}", file=sys.stderr)
        return False
    return True


def decode(path: str, count: int, stride: int):
    capture = cv2.VideoCapture(path)
    if not capture.isOpened():
        print(f"could not decode {path}; OpenCV needs HEVC support", file=sys.stderr)
        return []
    frames = []
    index = 0
    while len(frames) < count:
        ok, frame = capture.read()
        if not ok:
            break
        if index % stride == 0:
            frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        index += 1
    capture.release()
    return frames


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', default=os.path.join(here, '..', 'models', 'driving_supercombo.onnx'))
    parser.add_argument('--provider', default='cpu', help='auto | cpu | cuda | tensorrt')
    parser.add_argument('--frames', type=int, default=40, help='frames to run after warmup')
    parser.add_argument('--cache', default=os.path.join(here, '..', 'models', 'reference_segment.hevc'))
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        print(f"model not found: {args.model}\nRun `make openpilot-models` first.", file=sys.stderr)
        return 1
    if not fetch(args.cache):
        return 1

    warmup = 15
    frames = decode(args.cache, args.frames + warmup, stride=1)
    if len(frames) < warmup + 5:
        print(f"only decoded {len(frames)} frames", file=sys.stderr)
        return 1
    if frames[0].shape[:2] != (NEO_HEIGHT, NEO_WIDTH):
        print(f"unexpected frame size {frames[0].shape[:2]}, expected "
              f"{(NEO_HEIGHT, NEO_WIDTH)}", file=sys.stderr)
        return 1

    intrinsics = np.array([[NEO_FOCAL, 0.0, NEO_WIDTH / 2],
                           [0.0, NEO_FOCAL, NEO_HEIGHT / 2],
                           [0.0, 0.0, 1.0]])
    runner = SupercomboRunner(args.model, provider=args.provider, expected_sha256='')
    # The segment runs at the rate upstream assumes, so the default skip applies.
    warp = get_warp_matrix((0, 0, 0), intrinsics, False)
    warp_big = get_warp_matrix((0, 0, 0), intrinsics, True)
    desire = np.zeros(8, dtype=np.float32)
    traffic_convention = np.array([1.0, 0.0], dtype=np.float32)  # left-hand drive footage

    probabilities, lane_stds = [], []
    for index, frame in enumerate(frames):
        planes = rgb_to_yuv_planes(frame)
        outputs = runner.run(prepare_model_frame(planes, warp),
                             prepare_model_frame(planes, warp_big),
                             desire, traffic_convention,
                             np.array([0.45, 0.55], dtype=np.float32))
        if index < warmup:
            continue
        probabilities.append(outputs['lane_lines_prob'][0][1::2])
        lane_stds.append(outputs['lane_lines_stds'][0][:, 11, 0])

    probabilities = np.array(probabilities)
    lane_stds = np.array(lane_stds)
    per_line = probabilities.mean(axis=0)
    ego_lanes = float(np.sort(per_line)[-2:].mean())

    print(f"frames evaluated : {len(probabilities)} at {NEO_WIDTH}x{NEO_HEIGHT}, "
          f"provider {runner.active_providers[0]}")
    print(f"lane line probs  : {np.round(per_line, 3)}")
    print(f"ego lane mean    : {ego_lanes:.3f}   (expect >= {EXPECTED_EGO_LANE_PROB})")
    print(f"lane line spread : {lane_stds.mean():.2f} m mean, "
          f"{lane_stds.mean(axis=0).min():.2f} m on the best line")

    if ego_lanes < EXPECTED_EGO_LANE_PROB:
        print("\nFAIL: the model does not find lane lines on footage it was trained for, "
              "so the fault is in this integration rather than in the input.", file=sys.stderr)
        return 1
    print("\nOK: the pipeline reproduces openpilot-grade lane detection on real footage.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

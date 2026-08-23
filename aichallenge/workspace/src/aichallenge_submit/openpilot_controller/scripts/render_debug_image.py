#!/usr/bin/env python3
"""Render /openpilot/debug/image offline, from a still frame.

Useful for tuning `camera.hfov_deg` and `camera.calib_pitch` without running the
simulator: grab a frame from AWSIM (or a rosbag), sweep the angle, and look at
where the horizon lands in the model_input view.

    python3 scripts/render_debug_image.py --image frame.png --out /tmp
    python3 scripts/render_debug_image.py --image frame.png --calib-pitch 0.03
    python3 scripts/render_debug_image.py --image frame.png --source camera
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from openpilot_controller.debug_image import build_debug_image  # noqa: E402
from openpilot_controller.openpilot_controller_core import ControlConfig, OpenPilotCore  # noqa: E402
from openpilot_controller.transforms import focal_from_hfov, intrinsics_from_focal  # noqa: E402


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--image', required=True, help='camera frame to run the model on')
    parser.add_argument('--model', default=os.path.join(here, '..', 'models', 'driving_supercombo.onnx'))
    parser.add_argument('--provider', default='cpu', help='auto | cpu | cuda | tensorrt')
    parser.add_argument('--source', default='model_input', choices=('model_input', 'camera'))
    parser.add_argument('--hfov-deg', type=float, default=60.0)
    parser.add_argument('--calib-roll', type=float, default=0.0)
    parser.add_argument('--calib-pitch', type=float, default=0.0)
    parser.add_argument('--calib-yaw', type=float, default=0.0)
    parser.add_argument('--camera-height', type=float, default=0.7)
    parser.add_argument('--v-ego', type=float, default=8.0)
    parser.add_argument('--steps', type=int, default=25,
                        help='repeats of the frame, to fill the temporal buffers')
    parser.add_argument('--scale', type=int, default=2)
    parser.add_argument('--out', default='.')
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        print(f"model not found: {args.model}\nRun `make openpilot-models` first.", file=sys.stderr)
        return 1
    bgr = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if bgr is None:
        print(f"could not read image: {args.image}", file=sys.stderr)
        return 1
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    height, width = rgb.shape[:2]

    core = OpenPilotCore(args.model, provider=args.provider, expected_sha256='',
                         calib_euler=(args.calib_roll, args.calib_pitch, args.calib_yaw),
                         control=ControlConfig())
    core.set_intrinsics(intrinsics_from_focal(width, height, focal_from_hfov(width, args.hfov_deg)))
    for _ in range(max(args.steps, 1)):
        action = core.process(rgb, v_ego=args.v_ego)

    hud = (
        f'curv {action.desired_curvature:+.4f} 1/m   accel {action.desired_acceleration:+.2f} m/s2',
        f'v_ego {args.v_ego:.1f}   target {action.target_speed:.1f} m/s',
        f'hfov {args.hfov_deg:.0f}deg   pitch {args.calib_pitch:+.3f} rad',
    )
    picture = build_debug_image(action, core.last_model_frame, source=args.source,
                               camera_rgb=rgb, camera_intrinsics=core.intrinsics,
                               device_from_calib=core.device_from_calib,
                               scale=args.scale, hud_lines=hud,
                               path_z_offset=args.camera_height)
    os.makedirs(args.out, exist_ok=True)
    destination = os.path.join(args.out, f'openpilot_debug_{args.source}.png')
    cv2.imwrite(destination, picture)

    print(f"wrote {destination} ({picture.shape[1]}x{picture.shape[0]})")
    print(f"curvature {action.desired_curvature:+.5f} 1/m, "
          f"path reaches {action.path[:, 0].max():.1f} m, "
          f"lane line probs {np.round(action.lane_lines_prob, 2)}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
"""Run openpilot's driver monitoring model over a video.

Unlike the driving model, this one carries its own sanity check: `face_prob`.
If it is high the model found a face, so the preprocessing is right and any
weak sleep/phone readings are about the footage rather than the plumbing.

    python3 run_dms.py --video clip.mp4
    python3 run_dms.py --video clip.mp4 --hfov-deg 60 --annotate out.mp4

openpilot feeds this model a 1440x960 luma crop of a 1928x1208 fisheye at a
focal length of 567 px, and the crop is exactly that: no rescale, no distortion
correction. Footage from a different camera has to be mapped to the same angular
scale, which is what --hfov-deg controls. Sweep it: the driving model turned out
to be very sensitive to the assumed field of view, and this one may be too.
"""

import argparse
import codecs
import os
import pickle
import sys

import cv2
import numpy as np
import onnxruntime as ort

DM_WIDTH, DM_HEIGHT = 1440, 960
DM_FOCAL = 567.0  # openpilot's dmonitoringmodel_fl
# openpilot's crop sits above centre: DM_HEIGHT/2 - (1208 - DM_HEIGHT)/2
DM_CX, DM_CY = DM_WIDTH / 2, DM_HEIGHT / 2 - (1208 - DM_HEIGHT) / 2

PROBS = ('face_prob', 'left_eye_prob', 'right_eye_prob', 'sunglasses_prob',
         'left_blink_prob', 'right_blink_prob', 'using_phone_prob', 'sleep_prob')


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -30, 30)))


def build_warp(width, height, hfov_deg, pitch_shift):
    """Camera pixels -> the model's 1440x960 frame, matching openpilot's crop."""
    focal = (width / 2) / np.tan(np.radians(hfov_deg) / 2)
    camera = np.array([[focal, 0.0, width / 2],
                       [0.0, focal, height / 2],
                       [0.0, 0.0, 1.0]])
    model = np.array([[DM_FOCAL, 0.0, DM_CX],
                      [0.0, DM_FOCAL, DM_CY + pitch_shift],
                      [0.0, 0.0, 1.0]])
    # warpPerspective with WARP_INVERSE_MAP wants model -> camera
    return camera @ np.linalg.inv(model)


class DriverMonitor:
    def __init__(self, model_path, provider='cpu'):
        options = ort.SessionOptions()
        options.log_severity_level = 3
        providers = {'cpu': ['CPUExecutionProvider'],
                     'cuda': ['CUDAExecutionProvider', 'CPUExecutionProvider']}.get(provider)
        if providers is None:
            raise ValueError(f"unknown provider '{provider}'")
        self.session = ort.InferenceSession(model_path, options, providers=providers)
        metadata = self.session.get_modelmeta().custom_metadata_map
        if 'output_slices' not in metadata:
            raise RuntimeError(f"{model_path} has no output_slices metadata")
        self.slices = pickle.loads(codecs.decode(metadata['output_slices'].encode(), 'base64'))
        self.input_dtype = {'tensor(uint8)': np.uint8,
                            'tensor(float)': np.float32}[self.session.get_inputs()[0].type]

    def run(self, luma_1440x960, calib=(0.0, 0.0, 0.0)):
        image = np.ascontiguousarray(luma_1440x960, dtype=self.input_dtype).reshape(1, -1)
        raw = self.session.run(None, {
            'input_img': image,
            'calib': np.asarray(calib, dtype=np.float32).reshape(1, 3),
        })[0][0].astype(np.float32)
        out = {}
        for hand in ('lhd', 'rhd'):
            for key in PROBS:
                out[f'{key}_{hand}'] = float(sigmoid(raw[self.slices[f'{key}_{hand}']])[0])
            descs = raw[self.slices[f'face_descs_{hand}']]
            out[f'face_orientation_{hand}'] = descs[:3]
            out[f'face_position_{hand}'] = descs[3:5]
        out['wheel_on_right'] = float(sigmoid(raw[self.slices['wheel_on_right']])[0])
        return out


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--video', required=True)
    parser.add_argument('--model', default=os.path.join(here, 'models', 'dmonitoring_model.onnx'))
    parser.add_argument('--provider', default='cpu')
    parser.add_argument('--hfov-deg', type=float, default=104.0,
                        help='assumed horizontal field of view of the footage; '
                             '104 matches the model frame, a webcam is nearer 78')
    parser.add_argument('--pitch-shift', type=float, default=0.0,
                        help='shift the crop vertically, in model pixels')
    parser.add_argument('--flip', action='store_true', help='mirror the frame')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--annotate', default='', help='write an annotated mp4 here')
    args = parser.parse_args()

    if not os.path.isfile(args.model):
        print(f"model not found: {args.model}\nRun ./download_model.bash first.", file=sys.stderr)
        return 1
    capture = cv2.VideoCapture(args.video)
    if not capture.isOpened():
        print(f"could not open {args.video}", file=sys.stderr)
        return 1

    monitor = DriverMonitor(args.model, args.provider)
    warp = None
    writer = None
    rows = []
    while True:
        ok, frame = capture.read()
        if not ok or (args.max_frames and len(rows) >= args.max_frames):
            break
        if args.flip:
            frame = frame[:, ::-1]
        if warp is None:
            height, width = frame.shape[:2]
            warp = build_warp(width, height, args.hfov_deg, args.pitch_shift)
            print(f"{width}x{height} footage, assumed hfov {args.hfov_deg:.0f} deg "
                  f"-> {DM_WIDTH}x{DM_HEIGHT} model frame")
        luma = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        warped = cv2.warpPerspective(luma, warp, (DM_WIDTH, DM_HEIGHT),
                                     flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                     borderMode=cv2.BORDER_REPLICATE)
        out = monitor.run(warped)
        rows.append(out)

        if args.annotate:
            canvas = cv2.cvtColor(cv2.resize(warped, (DM_WIDTH // 2, DM_HEIGHT // 2)), cv2.COLOR_GRAY2BGR)
            hand = 'lhd' if out['face_prob_lhd'] >= out['face_prob_rhd'] else 'rhd'
            lines = [f"face {out[f'face_prob_{hand}']:.2f}   eyes "
                     f"{out[f'left_eye_prob_{hand}']:.2f}/{out[f'right_eye_prob_{hand}']:.2f}",
                     f"blink {out[f'left_blink_prob_{hand}']:.2f}/{out[f'right_blink_prob_{hand}']:.2f}"
                     f"   phone {out[f'using_phone_prob_{hand}']:.2f}",
                     f"SLEEP {out[f'sleep_prob_{hand}']:.2f}"]
            for row, text in enumerate(lines):
                origin = (10, 26 + row * 26)
                cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
            if writer is None:
                fps = capture.get(cv2.CAP_PROP_FPS) or 20.0
                writer = cv2.VideoWriter(args.annotate, cv2.VideoWriter_fourcc(*'mp4v'),
                                         fps, (canvas.shape[1], canvas.shape[0]))
            writer.write(canvas)
    capture.release()
    if writer is not None:
        writer.release()
        print(f"wrote {args.annotate}")

    if not rows:
        print("no frames decoded", file=sys.stderr)
        return 1

    hand = 'lhd' if np.mean([r['face_prob_lhd'] for r in rows]) >= \
                    np.mean([r['face_prob_rhd'] for r in rows]) else 'rhd'
    print(f"{len(rows)} frames, reporting the {hand} head (mean face_prob decides)\n")
    print(f"{'signal':18s} {'mean':>7s} {'max':>7s}")
    for key in PROBS:
        values = np.array([r[f'{key}_{hand}'] for r in rows])
        print(f"{key:18s} {values.mean():7.3f} {values.max():7.3f}")

    face = np.array([r[f'face_prob_{hand}'] for r in rows]).mean()
    print()
    if face < 0.3:
        print("face_prob is low: the model is not finding a face at all, so the other "
              "signals mean nothing. Sweep --hfov-deg and --pitch-shift before reading "
              "anything into sleep_prob.")
    else:
        print("face_prob is healthy, so the preprocessing is landing a face in the "
              "model frame and the remaining signals are worth reading.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

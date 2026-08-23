"""Estimate the camera mounting angles from the model's own pose output.

Reduced port of openpilot ``openpilot/selfdrive/locationd/calibrationd.py`` at
commit 084747c7 (MIT, comma.ai). See ``THIRD_PARTY_NOTICES.md``.

While driving straight and fast, the direction of the ego translation the model
predicts should point straight down the camera axis. Whatever angle it comes out
at instead is the mounting error, so the estimate is fed back into the warp.
Roll is assumed to be zero, as upstream does.
"""

import os

import numpy as np
import yaml

from openpilot_controller.transforms import rot_from_euler

BLOCK_SIZE = 100
INPUTS_NEEDED = 5
INPUTS_WANTED = 50
SMOOTH_CYCLES = 10


def euler_from_rot(rot: np.ndarray) -> np.ndarray:
    """Inverse of transforms.rot_from_euler, which builds Rz @ Ry @ Rx."""
    pitch = np.arcsin(np.clip(-rot[2, 0], -1.0, 1.0))
    roll = np.arctan2(rot[2, 1], rot[2, 2])
    yaw = np.arctan2(rot[1, 0], rot[0, 0])
    return np.array([roll, pitch, yaw])


def moving_avg_with_linear_decay(prev_mean, new_val, idx, block_size):
    return (idx * prev_mean + (block_size - idx) * new_val) / block_size


class Calibrator:
    """Blockwise average of per-frame observations, as upstream does."""

    def __init__(self, min_speed: float = 3.0, max_yaw_rate: float = np.radians(2.0),
                 max_vel_angle_std: float = np.radians(0.25),
                 pitch_limits=(-0.15, 0.20), yaw_limits=(-0.10, 0.10)):
        self.min_speed = float(min_speed)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_vel_angle_std = float(max_vel_angle_std)
        self.pitch_limits = tuple(pitch_limits)
        self.yaw_limits = tuple(yaw_limits)
        self.reset()

    def reset(self, rpy_init=(0.0, 0.0, 0.0)) -> None:
        self.rpy = np.array(rpy_init, dtype=np.float64)
        self.rpys = np.tile(self.rpy, (INPUTS_WANTED, 1))
        self.idx = 0
        self.block_idx = 0
        self.valid_blocks = 0
        self.old_rpy = np.array(rpy_init, dtype=np.float64)
        self.old_rpy_weight = 0.0
        self.seen = 0
        self.accepted = 0
        self.fast_frames = 0
        self.straight_frames = 0

    @property
    def calibrated(self) -> bool:
        return self.valid_blocks >= INPUTS_NEEDED

    @property
    def progress(self) -> float:
        return min(1.0, self.valid_blocks / INPUTS_NEEDED)

    def status(self) -> str:
        """Why calibration is or is not progressing, for the periodic log."""
        if self.seen == 0:
            return 'no frames yet'
        pct = lambda n: 100.0 * n / self.seen  # noqa: E731
        return (f'{self.accepted}/{BLOCK_SIZE * INPUTS_NEEDED} usable frames '
                f'({self.valid_blocks}/{INPUTS_NEEDED} blocks); of {self.seen} seen, '
                f'{pct(self.fast_frames):.0f}% fast enough and {pct(self.straight_frames):.0f}% straight enough')

    def sanity_clip(self, rpy: np.ndarray) -> np.ndarray:
        if np.isnan(rpy).any():
            return np.zeros(3)
        return np.array([0.0,
                         float(np.clip(rpy[1], *self.pitch_limits)),
                         float(np.clip(rpy[2], *self.yaw_limits))])

    def get_smooth_rpy(self) -> np.ndarray:
        if self.old_rpy_weight > 0:
            return self.old_rpy_weight * self.old_rpy + (1.0 - self.old_rpy_weight) * self.rpy
        return self.rpy

    def handle_pose(self, trans, rot, trans_std, v_ego: float):
        """One model frame. Returns the updated euler angles, or None if not used."""
        self.old_rpy_weight = max(0.0, self.old_rpy_weight - 1.0 / SMOOTH_CYCLES)
        self.seen += 1

        fast = v_ego > self.min_speed and trans[0] > self.min_speed
        straight = abs(rot[2]) < self.max_yaw_rate
        self.fast_frames += bool(fast)
        self.straight_frames += bool(straight)
        certain = np.arctan2(abs(trans_std[1]), max(trans[0], 1e-3)) < self.max_vel_angle_std
        if not (fast and straight and (certain or not self.calibrated)):
            return None
        self.accepted += 1

        observed_rpy = np.array([0.0,
                                 -np.arctan2(trans[2], trans[0]),
                                 np.arctan2(trans[1], trans[0])])
        new_rpy = euler_from_rot(rot_from_euler(self.get_smooth_rpy()) @ rot_from_euler(observed_rpy))
        new_rpy = self.sanity_clip(new_rpy)

        self.rpys[self.block_idx] = moving_avg_with_linear_decay(
            self.rpys[self.block_idx], new_rpy, self.idx, float(BLOCK_SIZE))

        self.idx = (self.idx + 1) % BLOCK_SIZE
        if self.idx == 0:
            self.block_idx = (self.block_idx + 1) % INPUTS_WANTED
            self.valid_blocks = min(self.valid_blocks + 1, INPUTS_WANTED)

        self.rpy = np.mean(self.rpys[:max(self.valid_blocks, 1)], axis=0)
        return self.rpy


def load_calibration(path: str):
    """Read a saved estimate. Returns (rpy, valid_blocks) or None."""
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path) as handle:
            saved = yaml.safe_load(handle) or {}
        rpy = np.array([float(saved["roll"]), float(saved["pitch"]), float(saved["yaw"])])
        if not np.isfinite(rpy).all():
            return None
        return rpy, int(saved.get("valid_blocks", 0))
    except Exception:
        return None


def save_calibration(path: str, rpy, valid_blocks: int) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w") as handle:
        yaml.safe_dump({"roll": float(rpy[0]), "pitch": float(rpy[1]), "yaw": float(rpy[2]),
                        "valid_blocks": int(valid_blocks)}, handle)
    os.replace(temporary, path)

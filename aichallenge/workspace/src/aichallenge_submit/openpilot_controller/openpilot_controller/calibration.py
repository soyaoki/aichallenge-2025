"""Estimate the camera mounting angles from the model's own pose output.

Reduced port of openpilot ``openpilot/selfdrive/locationd/calibrationd.py`` at
commit 084747c7 (MIT, comma.ai). See ``THIRD_PARTY_NOTICES.md``.

While driving straight and fast, the direction of the ego translation the model
predicts should point straight down the camera axis. Whatever angle it comes out
at instead is the mounting error, so the estimate is fed back into the warp.
Roll is assumed to be zero, as upstream does.
"""

import numpy as np

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

    @property
    def calibrated(self) -> bool:
        return self.valid_blocks >= INPUTS_NEEDED

    @property
    def progress(self) -> float:
        return min(1.0, self.valid_blocks / INPUTS_NEEDED)

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

        straight_and_fast = (v_ego > self.min_speed and trans[0] > self.min_speed
                             and abs(rot[2]) < self.max_yaw_rate)
        certain = np.arctan2(abs(trans_std[1]), max(trans[0], 1e-3)) < self.max_vel_angle_std
        if not (straight_and_fast and (certain or not self.calibrated)):
            return None

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

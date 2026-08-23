"""comma device frame -> AI Challenge base_link.

openpilot's device frame is x->forward, y->right, z->down; Autoware's
``base_link`` is x->forward, y->left, z->up, so the default conversion negates
y and z and flips the sign of yaw. The axis handling is configurable because
the mapping is worth confirming against a real drive rather than trusting.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class FrameConfig:
    swap_xy: bool = False
    invert_x: bool = False
    invert_y: bool = True
    invert_z: bool = True

    @property
    def yaw_sign(self) -> float:
        """Sign applied to yaw when mapping device yaw onto base_link yaw."""
        sign = -1.0 if self.invert_y else 1.0
        return -sign if self.swap_xy else sign


def to_base_link(points: np.ndarray, config: FrameConfig) -> np.ndarray:
    """(..., 3) points in the comma device frame -> base_link."""
    out = np.array(points, dtype=np.float64, copy=True)
    if config.swap_xy:
        out[..., [0, 1]] = out[..., [1, 0]]
    if config.invert_x:
        out[..., 0] *= -1.0
    if config.invert_y:
        out[..., 1] *= -1.0
    if config.invert_z:
        out[..., 2] *= -1.0
    return out


def yaw_to_quaternion(yaw: float):
    """(x, y, z, w) for a rotation about the base_link z axis."""
    half = 0.5 * float(yaw)
    return 0.0, 0.0, float(np.sin(half)), float(np.cos(half))

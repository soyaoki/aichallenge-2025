"""Camera -> model frame transforms for comma.ai's driving_supercombo network.

The intrinsics, ``get_warp_matrix`` and the YUV channel layout are ports of
openpilot ``common/transformations/{camera,model,orientation}.py`` and
``openpilot/selfdrive/modeld/compile_modeld.py`` at commit 084747c7
(MIT, comma.ai). Upstream warps on the GPU with tinygrad; here the same
matrices drive ``cv2.warpPerspective``. See ``THIRD_PARTY_NOTICES.md``.
"""

import cv2
import numpy as np

# MED model (the `img` input)
MEDMODEL_INPUT_SIZE = (512, 256)
MEDMODEL_CY = 47.6
medmodel_fl = 910.0
medmodel_intrinsics = np.array([
    [medmodel_fl, 0.0, 0.5 * MEDMODEL_INPUT_SIZE[0]],
    [0.0, medmodel_fl, MEDMODEL_CY],
    [0.0, 0.0, 1.0]])

# SBIG model (the `big_img` input: big model at the size of the small model)
SBIGMODEL_INPUT_SIZE = (512, 256)
sbigmodel_fl = 455.0
sbigmodel_intrinsics = np.array([
    [sbigmodel_fl, 0.0, 0.5 * SBIGMODEL_INPUT_SIZE[0]],
    [0.0, sbigmodel_fl, 0.5 * (256 + MEDMODEL_CY)],
    [0.0, 0.0, 1.0]])

# device/mesh : x->forward, y->right, z->down
# view        : x->right,   y->down,  z->forward
device_frame_from_view_frame = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0]])
view_frame_from_device_frame = device_frame_from_view_frame.T

calib_from_medmodel = np.linalg.inv(medmodel_intrinsics @ view_frame_from_device_frame)
calib_from_sbigmodel = np.linalg.inv(sbigmodel_intrinsics @ view_frame_from_device_frame)

# UV_SCALE @ M @ UV_SCALE_INV with UV_SCALE = diag(0.5, 0.5, 1), as an elementwise factor.
_UV_WARP_SCALE = np.array([
    [1.0, 1.0, 0.5],
    [1.0, 1.0, 0.5],
    [2.0, 2.0, 1.0]])


def rot_from_euler(euler) -> np.ndarray:
    """Rz(yaw) @ Ry(pitch) @ Rx(roll), matching openpilot's euler2rot."""
    roll, pitch, yaw = (float(v) for v in euler)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def intrinsics_from_focal(width: int, height: int, focal_length: float) -> np.ndarray:
    return np.array([
        [focal_length, 0.0, float(width) / 2.0],
        [0.0, focal_length, float(height) / 2.0],
        [0.0, 0.0, 1.0]])


def focal_from_hfov(width: int, hfov_deg: float) -> float:
    """Pinhole focal length in pixels for a horizontal field of view."""
    return (float(width) / 2.0) / np.tan(np.deg2rad(float(hfov_deg)) / 2.0)


def get_warp_matrix(device_from_calib_euler, intrinsics, bigmodel_frame: bool = False) -> np.ndarray:
    """Model-frame pixel coordinates -> camera pixel coordinates."""
    calib_from_model = calib_from_sbigmodel if bigmodel_frame else calib_from_medmodel
    device_from_calib = rot_from_euler(device_from_calib_euler)
    camera_from_calib = intrinsics @ view_frame_from_device_frame @ device_from_calib
    return camera_from_calib @ calib_from_model


def rgb_to_yuv_planes(rgb: np.ndarray):
    """(H, W, 3) uint8 RGB -> (Y, U, V) planes, U/V at half resolution."""
    height, width = rgb.shape[:2]
    if height % 2 or width % 2:
        rgb = rgb[:height - (height % 2), :width - (width % 2)]
        height, width = rgb.shape[:2]
    i420 = cv2.cvtColor(rgb, cv2.COLOR_RGB2YUV_I420)
    y = i420[:height]
    uv_rows = height // 4
    u = i420[height:height + uv_rows].reshape(height // 2, width // 2)
    v = i420[height + uv_rows:height + 2 * uv_rows].reshape(height // 2, width // 2)
    return y, u, v


def prepare_model_frame(yuv_planes, warp_matrix: np.ndarray,
                        model_width: int = 512, model_height: int = 256) -> np.ndarray:
    """(Y, U, V) planes -> a (6, model_height/2, model_width/2) uint8 model frame.

    Channel order matches upstream ``frames_to_tensor``: the four Y subsamples
    (even/odd rows x even/odd columns) followed by U and V. Nearest-neighbour
    sampling with edge replication mirrors the tinygrad warp kernel.
    """
    y, u, v = yuv_planes
    flags = cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP
    border = cv2.BORDER_REPLICATE
    warped_y = cv2.warpPerspective(y, warp_matrix, (model_width, model_height),
                                   flags=flags, borderMode=border)
    uv_matrix = warp_matrix * _UV_WARP_SCALE
    uv_size = (model_width // 2, model_height // 2)
    warped_u = cv2.warpPerspective(u, uv_matrix, uv_size, flags=flags, borderMode=border)
    warped_v = cv2.warpPerspective(v, uv_matrix, uv_size, flags=flags, borderMode=border)

    frame = np.empty((6, model_height // 2, model_width // 2), dtype=np.uint8)
    frame[0] = warped_y[0::2, 0::2]
    frame[1] = warped_y[1::2, 0::2]
    frame[2] = warped_y[0::2, 1::2]
    frame[3] = warped_y[1::2, 1::2]
    frame[4] = warped_u
    frame[5] = warped_v
    return frame

"""Draw the model's prediction back onto an image.

Kept free of ROS types so it can be rendered and eyeballed offline
(``scripts/render_debug_image.py``) as well as published by the node.
"""

import cv2
import numpy as np

from openpilot_controller.model_constants import ModelConstants
from openpilot_controller.transforms import (
    medmodel_intrinsics,
    model_frame_to_rgb,
    project_calib_points,
)

# BGR, matching the bgr8 encoding the image is published with.
PATH_BGR = (60, 240, 40)
LANE_LINE_BGR = (80, 240, 240)
ROAD_EDGE_BGR = (40, 120, 250)
HUD_BGR = (240, 240, 240)


def cross_section_to_points(values: np.ndarray) -> np.ndarray:
    """(IDX_N, 2) of (y, z) at X_IDXS -> (IDX_N, 3) points in the calibrated frame."""
    points = np.empty((values.shape[0], 3), dtype=np.float64)
    points[:, 0] = ModelConstants.X_IDXS
    points[:, 1] = values[:, 0]
    points[:, 2] = values[:, 1]
    return points




def _draw_polyline(image, points_calib, intrinsics, device_from_calib, color, thickness):
    pixels, valid = project_calib_points(points_calib, intrinsics, device_from_calib)
    height, width = image.shape[:2]
    run = []
    for pixel, ok in zip(pixels, valid):
        inside = ok and -width < pixel[0] < 2 * width and -height < pixel[1] < 2 * height
        if inside:
            run.append((int(round(pixel[0])), int(round(pixel[1]))))
            continue
        if len(run) > 1:
            cv2.polylines(image, [np.array(run, np.int32)], False, color, thickness, cv2.LINE_AA)
        run = []
    if len(run) > 1:
        cv2.polylines(image, [np.array(run, np.int32)], False, color, thickness, cv2.LINE_AA)


def build_debug_image(action, model_frame, source: str = 'model_input',
                      camera_rgb=None, camera_intrinsics=None, device_from_calib=None,
                      scale: int = 2, hud_lines=(), path_z_offset: float = 0.7) -> np.ndarray:
    """An annotated BGR image of what the model saw and what it predicted.

    ``model_input`` shows the warped frame the network was actually fed, which is
    where a wrong camera FOV or mounting pitch shows up first. ``camera`` draws
    the same prediction on the untouched camera image.

    The predicted path is referenced to the camera height, so it is shifted down
    by ``path_z_offset`` (the camera's height above the road) to land on the road
    surface. Lane lines and road edges already carry their own height, matching
    what openpilot's own renderer does.
    """
    if source == 'camera':
        if camera_rgb is None or camera_intrinsics is None:
            raise ValueError("camera source needs camera_rgb and camera_intrinsics")
        image = cv2.cvtColor(camera_rgb, cv2.COLOR_RGB2BGR)
        intrinsics = np.asarray(camera_intrinsics, dtype=np.float64)
        calib_rotation = device_from_calib
    else:
        if model_frame is None:
            raise ValueError("model_input source needs a model frame")
        image = cv2.cvtColor(model_frame_to_rgb(model_frame), cv2.COLOR_RGB2BGR)
        intrinsics = medmodel_intrinsics
        calib_rotation = None

    if scale > 1:
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        intrinsics = intrinsics.copy()
        intrinsics[:2] *= scale
    image = np.ascontiguousarray(image)

    for index, road_edge in enumerate(action.road_edges):
        _draw_polyline(image, cross_section_to_points(road_edge), intrinsics, calib_rotation,
                       ROAD_EDGE_BGR, max(1, scale))
    for index, lane_line in enumerate(action.lane_lines):
        probability = float(np.clip(action.lane_lines_prob[index], 0.0, 1.0))
        if probability < 0.1:
            continue
        colour = tuple(int(c * (0.35 + 0.65 * probability)) for c in LANE_LINE_BGR)
        _draw_polyline(image, cross_section_to_points(lane_line), intrinsics, calib_rotation,
                       colour, max(1, scale))
    path = np.array(action.path, dtype=np.float64, copy=True)
    path[:, 2] += path_z_offset  # device frame z points down
    _draw_polyline(image, path, intrinsics, calib_rotation, PATH_BGR, max(2, scale))

    for row, text in enumerate(hud_lines):
        origin = (6, 16 * scale // 2 + row * 14)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.38, HUD_BGR, 1, cv2.LINE_AA)
    return image

"""RViz markers for what the model predicts.

Publishing these before wiring up control is the cheapest way to tell whether
the model actually sees the track: the predicted path should bend into a corner
before the vehicle reaches it.
"""

import numpy as np
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from openpilot_controller.frames import FrameConfig, to_base_link
from openpilot_controller.debug_image import cross_section_to_points

PATH_COLOR = (0.15, 0.95, 0.35, 0.95)
LANE_LINE_COLOR = (0.98, 0.98, 0.35)
ROAD_EDGE_COLOR = (0.98, 0.45, 0.15)


def _line_strip(namespace: str, marker_id: int, points: np.ndarray, width: float,
                color, stamp, frame_id: str) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale = Vector3(x=width, y=0.0, z=0.0)
    marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
    marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points]
    return marker



def build_markers(action, stamp, frame_config: FrameConfig, frame_id: str = 'base_link',
                  path_width: float = 0.25, line_width: float = 0.1) -> MarkerArray:
    markers = MarkerArray()
    markers.markers.append(
        _line_strip('path', 0, to_base_link(action.path, frame_config),
                    path_width, PATH_COLOR, stamp, frame_id))

    for index, lane_line in enumerate(action.lane_lines):
        alpha = float(np.clip(action.lane_lines_prob[index], 0.0, 1.0))
        points = to_base_link(cross_section_to_points(lane_line), frame_config)
        markers.markers.append(
            _line_strip('lane_lines', index, points, line_width,
                        LANE_LINE_COLOR + (alpha,), stamp, frame_id))

    for index, road_edge in enumerate(action.road_edges):
        points = to_base_link(cross_section_to_points(road_edge), frame_config)
        markers.markers.append(
            _line_strip('road_edges', index, points, line_width,
                        ROAD_EDGE_COLOR + (0.85,), stamp, frame_id))
    return markers

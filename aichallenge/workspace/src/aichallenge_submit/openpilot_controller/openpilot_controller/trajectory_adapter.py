"""Model plan -> autoware_auto_planning_msgs/Trajectory in base_link."""

import numpy as np
from builtin_interfaces.msg import Duration
from autoware_auto_planning_msgs.msg import Trajectory, TrajectoryPoint

from openpilot_controller.frames import FrameConfig, to_base_link, yaw_to_quaternion
from openpilot_controller.model_constants import ModelConstants


def build_trajectory(action, stamp, frame_config: FrameConfig,
                     frame_id: str = 'base_link', max_speed: float = float('inf')) -> Trajectory:
    """Turn a prediction into a trajectory the challenge controllers can follow."""
    trajectory = Trajectory()
    trajectory.header.stamp = stamp
    trajectory.header.frame_id = frame_id

    positions = to_base_link(action.path, frame_config)
    yaws = action.path_yaw * frame_config.yaw_sign
    yaw_rates = action.path_yaw_rate * frame_config.yaw_sign
    speeds = np.clip(action.path_speed, 0.0, max_speed)

    for index, t in enumerate(ModelConstants.T_IDXS):
        point = TrajectoryPoint()
        seconds = int(t)
        point.time_from_start = Duration(sec=seconds, nanosec=int((t - seconds) * 1e9))
        point.pose.position.x = float(positions[index, 0])
        point.pose.position.y = float(positions[index, 1])
        point.pose.position.z = float(positions[index, 2])
        qx, qy, qz, qw = yaw_to_quaternion(yaws[index])
        point.pose.orientation.x = qx
        point.pose.orientation.y = qy
        point.pose.orientation.z = qz
        point.pose.orientation.w = qw
        point.longitudinal_velocity_mps = float(speeds[index])
        point.acceleration_mps2 = float(action.path_accel[index])
        point.heading_rate_rps = float(yaw_rates[index])
        trajectory.points.append(point)
    return trajectory

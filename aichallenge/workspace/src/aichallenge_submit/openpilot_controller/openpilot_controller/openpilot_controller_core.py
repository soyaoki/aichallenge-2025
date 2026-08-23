"""Frame-in / action-out core for the comma.ai supercombo controller.

Mirrors openpilot's ``modeld`` loop: warp the camera frame into the model's
calibrated frame, run driving_supercombo, then turn the predicted plan into a
desired acceleration and curvature via ``get_action_from_model``.
"""

from dataclasses import dataclass, field

import numpy as np

from openpilot_controller.drive_helpers import (
    DT_MDL,
    MAX_CURVATURE,
    MAX_LATERAL_ACCEL_NO_ROLL,
    MAX_LATERAL_JERK,
    clip_curvature,
    get_accel_from_plan,
    get_curvature_from_plan,
    should_stop,
    smooth_value,
)
from openpilot_controller.model_constants import ModelConstants, Plan
from openpilot_controller.supercombo import SupercomboRunner
from openpilot_controller.transforms import (
    focal_from_hfov,
    get_warp_matrix,
    intrinsics_from_focal,
    prepare_model_frame,
    rgb_to_yuv_planes,
    rot_from_euler,
)

# openpilot's modeld defaults
LAT_SMOOTH_SECONDS = 0.0
LONG_SMOOTH_SECONDS = 0.3
MIN_LAT_CONTROL_SPEED = 0.3


@dataclass
class ControlConfig:
    """Tunables that shape the plan -> action conversion."""

    # Actuator delays the model is asked to plan for, in seconds.
    lat_delay: float = 0.2
    long_delay: float = 0.2
    lat_smooth_seconds: float = LAT_SMOOTH_SECONDS
    long_smooth_seconds: float = LONG_SMOOTH_SECONDS
    # Safety envelope applied on top of the model output.
    apply_curvature_limits: bool = True
    max_curvature: float = MAX_CURVATURE
    max_lateral_jerk: float = MAX_LATERAL_JERK
    max_lateral_accel: float = MAX_LATERAL_ACCEL_NO_ROLL
    max_acceleration: float = 2.0
    max_deceleration: float = -4.0
    max_speed: float = 30.0
    # Japan drives on the left, i.e. right-hand drive.
    is_rhd: bool = True


def _zeros(*shape):
    return lambda: np.zeros(shape, dtype=np.float32)


@dataclass
class Action:
    """One prediction: the command to send, plus the plan it was derived from."""

    desired_curvature: float = 0.0
    desired_acceleration: float = 0.0
    target_speed: float = 0.0
    should_stop: bool = False
    curvature_limited: bool = False

    # Plan sampled at ModelConstants.T_IDXS, in the comma device frame
    # (x forward, y right, z down).
    path: np.ndarray = field(default_factory=_zeros(ModelConstants.IDX_N, 3))
    path_speed: np.ndarray = field(default_factory=_zeros(ModelConstants.IDX_N))
    path_accel: np.ndarray = field(default_factory=_zeros(ModelConstants.IDX_N))
    path_yaw: np.ndarray = field(default_factory=_zeros(ModelConstants.IDX_N))
    path_yaw_rate: np.ndarray = field(default_factory=_zeros(ModelConstants.IDX_N))

    # Cross sections of (y, z) at ModelConstants.X_IDXS.
    lane_lines: np.ndarray = field(
        default_factory=_zeros(ModelConstants.NUM_LANE_LINES, ModelConstants.IDX_N, 2))
    lane_lines_prob: np.ndarray = field(default_factory=_zeros(ModelConstants.NUM_LANE_LINES))
    road_edges: np.ndarray = field(
        default_factory=_zeros(ModelConstants.NUM_ROAD_EDGES, ModelConstants.IDX_N, 2))

    # Ego motion in the calibrated frame: translation velocity then rotation rate.
    pose: np.ndarray = field(default_factory=_zeros(ModelConstants.POSE_WIDTH))
    pose_std: np.ndarray = field(default_factory=_zeros(ModelConstants.POSE_WIDTH))


class OpenPilotCore:
    """Stateful wrapper: camera image + ego speed -> desired action."""

    def __init__(self, model_path: str, provider: str = 'auto', expected_sha256: str = '',
                 intra_op_threads: int = 0, calib_euler=(0.0, 0.0, 0.0),
                 control: ControlConfig | None = None):
        self.runner = SupercomboRunner(model_path, provider=provider,
                                       expected_sha256=expected_sha256,
                                       intra_op_threads=intra_op_threads)
        self.control = control or ControlConfig()
        self.calib_euler = np.asarray(calib_euler, dtype=np.float32)
        self.intrinsics = None
        self._warp_main = None
        self._warp_big = None
        self._desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)
        self._traffic_convention = np.zeros(ModelConstants.TRAFFIC_CONVENTION_LEN, dtype=np.float32)
        self._traffic_convention[int(self.control.is_rhd)] = 1.0
        self.prev_action = Action()
        # Last frame handed to the network, kept for the debug image.
        self.last_model_frame = None

    # -- calibration ------------------------------------------------------
    @property
    def calibrated(self) -> bool:
        return self._warp_main is not None

    def set_intrinsics(self, intrinsics) -> None:
        """Set the camera matrix K and rebuild both warp matrices."""
        self.intrinsics = np.asarray(intrinsics, dtype=np.float64).reshape(3, 3)
        self._warp_main = get_warp_matrix(self.calib_euler, self.intrinsics, False)
        self._warp_big = get_warp_matrix(self.calib_euler, self.intrinsics, True)

    def set_intrinsics_from_hfov(self, width: int, height: int, hfov_deg: float) -> None:
        self.set_intrinsics(intrinsics_from_focal(width, height, focal_from_hfov(width, hfov_deg)))

    def set_calibration_euler(self, calib_euler) -> None:
        self.calib_euler = np.asarray(calib_euler, dtype=np.float32)
        if self.intrinsics is not None:
            self.set_intrinsics(self.intrinsics)

    @property
    def device_from_calib(self) -> np.ndarray:
        return rot_from_euler(self.calib_euler)

    def set_frame_skip(self, frame_skip: int) -> None:
        self.runner.set_frame_skip(frame_skip)

    def reset(self) -> None:
        self.runner.reset()
        self.prev_action = Action()
        self.last_model_frame = None

    # -- inference --------------------------------------------------------
    def process(self, rgb: np.ndarray, v_ego: float, dt: float = DT_MDL) -> Action:
        if not self.calibrated:
            raise RuntimeError("camera intrinsics are not set yet")

        yuv = rgb_to_yuv_planes(rgb)
        frame = prepare_model_frame(yuv, self._warp_main)
        big_frame = prepare_model_frame(yuv, self._warp_big)
        self.last_model_frame = frame

        cfg = self.control
        v_ego = max(float(v_ego), 0.0)
        # Compensate for the frame age and for landing mid-way to the next frame,
        # the same way modeld does.
        frame_delay = dt
        action_delay = dt / 2.0
        lat_action_t = cfg.lat_delay + cfg.lat_smooth_seconds + frame_delay + action_delay
        long_action_t = cfg.long_delay + cfg.long_smooth_seconds + frame_delay + action_delay

        outputs = self.runner.run(frame, big_frame, self._desire, self._traffic_convention,
                                  np.array([lat_action_t, long_action_t], dtype=np.float32))
        return self._action_from_outputs(outputs, v_ego, lat_action_t, long_action_t, dt)

    def _action_from_outputs(self, outputs: dict, v_ego: float,
                             lat_action_t: float, long_action_t: float, dt: float) -> Action:
        cfg = self.control
        plan = outputs['plan'][0]
        speeds = plan[:, Plan.VELOCITY][:, 0]
        accels = plan[:, Plan.ACCELERATION][:, 0]

        desired_accel = get_accel_from_plan(speeds, accels, ModelConstants.T_IDXS, action_t=long_action_t)
        desired_curvature = get_curvature_from_plan(plan[:, Plan.T_FROM_CURRENT_EULER][:, 2],
                                                    plan[:, Plan.ORIENTATION_RATE][:, 2],
                                                    ModelConstants.T_IDXS, v_ego, lat_action_t)

        stop = should_stop(v_ego, desired_accel)
        desired_accel = smooth_value(desired_accel, self.prev_action.desired_acceleration,
                                     cfg.long_smooth_seconds, dt=dt)
        if v_ego > MIN_LAT_CONTROL_SPEED:
            desired_curvature = smooth_value(desired_curvature, self.prev_action.desired_curvature,
                                             cfg.lat_smooth_seconds, dt=dt)
        else:
            desired_curvature = self.prev_action.desired_curvature

        limited = False
        if cfg.apply_curvature_limits:
            desired_curvature, limited = clip_curvature(
                v_ego, self.prev_action.desired_curvature, desired_curvature, roll=0.0, dt=dt,
                max_lateral_jerk=cfg.max_lateral_jerk, max_lateral_accel=cfg.max_lateral_accel,
                max_curvature=cfg.max_curvature)

        target_speed = float(np.interp(long_action_t, ModelConstants.T_IDXS, speeds))
        target_speed = float(np.clip(target_speed, 0.0, cfg.max_speed))
        if v_ego >= cfg.max_speed:
            desired_accel = min(desired_accel, 0.0)
        desired_accel = float(np.clip(desired_accel, cfg.max_deceleration, cfg.max_acceleration))

        action = Action(
            desired_curvature=float(desired_curvature),
            desired_acceleration=desired_accel,
            target_speed=target_speed,
            should_stop=bool(stop),
            curvature_limited=bool(limited),
            path=plan[:, Plan.POSITION],
            path_speed=speeds,
            path_accel=accels,
            path_yaw=plan[:, Plan.T_FROM_CURRENT_EULER][:, 2],
            path_yaw_rate=plan[:, Plan.ORIENTATION_RATE][:, 2],
            lane_lines=outputs['lane_lines'][0],
            # Two values per lane line; upstream's laneLineProbs takes the odd ones.
            lane_lines_prob=outputs['lane_lines_prob'][0][1::2],
            road_edges=outputs['road_edges'][0],
            pose=outputs['pose'][0],
            pose_std=outputs['pose_stds'][0],
        )
        self.prev_action = action
        return action

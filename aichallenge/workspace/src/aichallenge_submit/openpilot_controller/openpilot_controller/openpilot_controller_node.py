#!/usr/bin/env python3
"""ROS 2 node running comma.ai's driving_supercombo as an end-to-end controller.

Two output modes:

``direct_action``
    Publish ``AckermannControlCommand`` from the model's desired curvature and
    acceleration. Only actually publishes when ``control.enabled`` is true.

``trajectory``
    Publish the predicted plan as an ``autoware_auto_planning_msgs/Trajectory``
    in ``base_link`` and let one of the challenge controllers follow it.

Either way the predicted path, lane lines and road edges go out as RViz markers,
which is what to look at first: the path should bend into a corner before the
vehicle reaches it. Control commands are published on a fixed timer with a
watchdog, so a stalled camera brakes rather than latching the last steering angle.
"""

import math
import os
import time

import numpy as np
import cv2

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import MarkerArray
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import VelocityReport

from openpilot_controller.calibration import Calibrator, load_calibration, save_calibration
from openpilot_controller.debug_image import build_debug_image
from openpilot_controller.debug_publisher import build_markers
from openpilot_controller.drive_helpers import DT_MDL
from openpilot_controller.frames import FrameConfig
from openpilot_controller.model_constants import ModelConstants
from openpilot_controller.openpilot_controller_core import ControlConfig, OpenPilotCore
from openpilot_controller.trajectory_adapter import build_trajectory

OUTPUT_MODES = ('direct_action', 'trajectory')


class OpenPilotControllerNode(Node):
    def __init__(self):
        super().__init__('openpilot_controller_node')

        self.declare_parameter('model.path', '')
        self.declare_parameter('model.sha256', '')
        self.declare_parameter('model.provider', 'auto')
        self.declare_parameter('model.intra_op_threads', 0)
        # Steps between the two frames the network stacks. 0 measures the camera
        # rate and picks the skip that lands closest to the model's context period.
        self.declare_parameter('model.frame_skip', 0)

        self.declare_parameter('camera.hfov_deg', 60.0)
        self.declare_parameter('camera.use_camera_info', True)
        self.declare_parameter('camera.calib_roll', 0.0)
        self.declare_parameter('camera.calib_pitch', 0.0)
        self.declare_parameter('camera.calib_yaw', 0.0)
        self.declare_parameter('camera.height', 0.7)
        # Estimate the mounting angles while driving, the way openpilot does, instead
        # of relying on the calib_* parameters being right.
        self.declare_parameter('camera.auto_calibration', True)
        self.declare_parameter('camera.calibration_min_speed', 3.0)
        # openpilot uses 2 deg/s, which assumes highway straights. A kart circuit is
        # almost all corners, so nothing qualifies unless this is relaxed - at the
        # cost of biasing the estimate with turning frames.
        self.declare_parameter('camera.calibration_max_yaw_rate_deg', 2.0)
        # Persisted so a calibration run (another controller drives while this node
        # only observes) carries over to the next run.
        self.declare_parameter('camera.calibration_file', '/aichallenge/calibration/openpilot_calibration.yaml')

        self.declare_parameter('output.mode', 'direct_action')
        self.declare_parameter('output.frame_id', 'base_link')
        self.declare_parameter('output.publish_markers', True)
        self.declare_parameter('output.publish_debug_image', True)
        # model_input: the warped frame the network is fed. camera: the raw camera image.
        self.declare_parameter('output.debug_image_source', 'model_input')
        self.declare_parameter('output.debug_image_scale', 2)
        self.declare_parameter('output.debug_image_decimation', 5)

        self.declare_parameter('frame.swap_xy', False)
        self.declare_parameter('frame.invert_x', False)
        self.declare_parameter('frame.invert_y', True)
        self.declare_parameter('frame.invert_z', True)

        self.declare_parameter('control.enabled', False)
        self.declare_parameter('control.publish_rate_hz', 20.0)
        self.declare_parameter('control.command_timeout_sec', 0.5)
        self.declare_parameter('control.lat_delay', 0.2)
        self.declare_parameter('control.long_delay', 0.2)
        self.declare_parameter('control.lat_smooth_seconds', 0.0)
        self.declare_parameter('control.long_smooth_seconds', 0.3)
        self.declare_parameter('control.apply_curvature_limits', True)
        self.declare_parameter('control.max_curvature', 0.44)
        self.declare_parameter('control.max_lateral_jerk', 20.0)
        self.declare_parameter('control.max_lateral_accel', 8.0)
        self.declare_parameter('control.max_acceleration', 2.0)
        self.declare_parameter('control.max_deceleration', -4.0)
        self.declare_parameter('control.max_speed', 30.0)
        self.declare_parameter('control.is_rhd', True)
        self.declare_parameter('control.timeout_deceleration', -2.0)
        # The model is trained for a car whose driver presses the gas to depart. From
        # a standstill it reports shouldStop, which would hold the kart at the line
        # forever, so pull away under our own power until the model has motion to see.
        self.declare_parameter('control.launch_speed', 2.0)
        self.declare_parameter('control.launch_acceleration', 1.0)
        self.declare_parameter('control.launch_rearm_sec', 2.0)

        self.declare_parameter('vehicle.wheel_base', 1.087)
        self.declare_parameter('vehicle.max_steering_tire_angle', 0.442)
        # simple_pure_pursuit uses 1.5 against AWSIM and 1.639 on the real kart, so the
        # bicycle-model angle alone under-steers here by that factor.
        self.declare_parameter('vehicle.steering_tire_angle_gain', 1.5)

        self.declare_parameter('log_interval_sec', 5.0)
        self.declare_parameter('debug', False)

        model_path = self.get_parameter('model.path').value
        if not model_path or not os.path.isfile(model_path):
            raise RuntimeError(
                f"model file not found: '{model_path}'. Run `make openpilot-models` to download it.")

        self.output_mode = str(self.get_parameter('output.mode').value)
        if self.output_mode not in OUTPUT_MODES:
            raise RuntimeError(
                f"unknown output.mode '{self.output_mode}', expected one of: {', '.join(OUTPUT_MODES)}")
        self.output_frame = str(self.get_parameter('output.frame_id').value)
        self.control_enabled = bool(self.get_parameter('control.enabled').value)

        self.frame_config = FrameConfig(
            swap_xy=bool(self.get_parameter('frame.swap_xy').value),
            invert_x=bool(self.get_parameter('frame.invert_x').value),
            invert_y=bool(self.get_parameter('frame.invert_y').value),
            invert_z=bool(self.get_parameter('frame.invert_z').value),
        )

        control = ControlConfig(
            lat_delay=float(self.get_parameter('control.lat_delay').value),
            long_delay=float(self.get_parameter('control.long_delay').value),
            lat_smooth_seconds=float(self.get_parameter('control.lat_smooth_seconds').value),
            long_smooth_seconds=float(self.get_parameter('control.long_smooth_seconds').value),
            apply_curvature_limits=bool(self.get_parameter('control.apply_curvature_limits').value),
            max_curvature=float(self.get_parameter('control.max_curvature').value),
            max_lateral_jerk=float(self.get_parameter('control.max_lateral_jerk').value),
            max_lateral_accel=float(self.get_parameter('control.max_lateral_accel').value),
            max_acceleration=float(self.get_parameter('control.max_acceleration').value),
            max_deceleration=float(self.get_parameter('control.max_deceleration').value),
            max_speed=float(self.get_parameter('control.max_speed').value),
            is_rhd=bool(self.get_parameter('control.is_rhd').value),
        )

        calib_euler = (
            float(self.get_parameter('camera.calib_roll').value),
            float(self.get_parameter('camera.calib_pitch').value),
            float(self.get_parameter('camera.calib_yaw').value),
        )

        self.core = OpenPilotCore(
            model_path,
            provider=self.get_parameter('model.provider').value,
            expected_sha256=self.get_parameter('model.sha256').value,
            intra_op_threads=int(self.get_parameter('model.intra_op_threads').value),
            calib_euler=calib_euler,
            control=control,
        )
        self.get_logger().info(
            f"driving_supercombo loaded (providers={self.core.runner.active_providers}, "
            f"checkpoint={self.core.runner.model_checkpoint})")

        self.use_camera_info = bool(self.get_parameter('camera.use_camera_info').value)
        self.hfov_deg = float(self.get_parameter('camera.hfov_deg').value)
        self.camera_height = float(self.get_parameter('camera.height').value)
        self.calibration_file = str(self.get_parameter('camera.calibration_file').value)
        saved = load_calibration(self.calibration_file)
        if saved is not None:
            calib_euler = tuple(saved[0])
            self.core.set_calibration_euler(calib_euler)
            self.get_logger().info(
                f'loaded calibration from {self.calibration_file}: '
                f'pitch {calib_euler[1]:+.4f} rad ({np.degrees(calib_euler[1]):+.2f} deg), '
                f'yaw {calib_euler[2]:+.4f} rad ({np.degrees(calib_euler[2]):+.2f} deg)')

        self.calibrator = None
        if bool(self.get_parameter('camera.auto_calibration').value):
            self.calibrator = Calibrator(
                min_speed=float(self.get_parameter('camera.calibration_min_speed').value),
                max_yaw_rate=np.radians(
                    float(self.get_parameter('camera.calibration_max_yaw_rate_deg').value)))
            self.calibrator.reset(calib_euler)
            if saved is not None:
                self.calibrator.valid_blocks = saved[1]
            self._calibration_reported = self.calibrator.calibrated
            self._calibration_saves = 0
        self.wheel_base = float(self.get_parameter('vehicle.wheel_base').value)
        self.max_steer = float(self.get_parameter('vehicle.max_steering_tire_angle').value)
        self.steering_gain = float(self.get_parameter('vehicle.steering_tire_angle_gain').value)
        self.command_timeout = float(self.get_parameter('control.command_timeout_sec').value)
        self.timeout_decel = float(self.get_parameter('control.timeout_deceleration').value)
        self.launch_speed = float(self.get_parameter('control.launch_speed').value)
        self.launch_acceleration = float(self.get_parameter('control.launch_acceleration').value)
        self.launch_rearm_sec = float(self.get_parameter('control.launch_rearm_sec').value)
        # Armed at a standstill, disarmed as soon as the kart is rolling, so the
        # model owns the longitudinal command for the rest of the run.
        self._launch_armed = True
        self._stalled_since = None
        self.debug = bool(self.get_parameter('debug').value)
        self.log_interval = float(self.get_parameter('log_interval_sec').value)

        self.v_ego = 0.0
        self._ego_pose = None
        # (action, stamp) written as one tuple so the control timer can never read
        # an action paired with the wrong timestamp.
        self._latest = None
        self.last_image_time = None
        self.frame_skip_param = int(self.get_parameter('model.frame_skip').value)
        if self.frame_skip_param > 0:
            self.core.set_frame_skip(self.frame_skip_param)
        self._frame_periods = []
        self._warmup_frames = 0
        self.inference_times = []
        self.last_log_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Inference runs in its own callback group so it cannot delay the control
        # timer, which is the watchdog that brakes when predictions stop arriving.
        self.inference_group = MutuallyExclusiveCallbackGroup()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, '/image_raw', self.image_callback, sensor_qos,
                                 callback_group=self.inference_group)
        if self.use_camera_info:
            # Not in the inference group: a busy inference callback would starve it
            # and the node would run on the fallback FOV forever.
            self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback,
                                     sensor_qos, callback_group=self.control_group)
        self.create_subscription(VelocityReport, '/vehicle/status/velocity_status',
                                 self.velocity_callback, sensor_qos,
                                 callback_group=self.control_group)
        if self.output_frame != 'base_link':
            self.create_subscription(Odometry, '/localization/kinematic_state',
                                     self.odometry_callback, 1,
                                     callback_group=self.control_group)

        self.pub_markers = None
        if bool(self.get_parameter('output.publish_markers').value):
            self.pub_markers = self.create_publisher(MarkerArray, '/openpilot/debug/markers', 1)

        self.pub_debug_image = None
        self.debug_image_source = str(self.get_parameter('output.debug_image_source').value)
        self.debug_image_scale = max(1, int(self.get_parameter('output.debug_image_scale').value))
        self.debug_image_decimation = max(1, int(self.get_parameter('output.debug_image_decimation').value))
        self._debug_image_counter = 0
        if bool(self.get_parameter('output.publish_debug_image').value):
            if self.debug_image_source not in ('model_input', 'camera'):
                raise RuntimeError(
                    f"unknown output.debug_image_source '{self.debug_image_source}', "
                    "expected model_input or camera")
            self.pub_debug_image = self.create_publisher(
                Image, '/openpilot/debug/image', qos_profile_sensor_data)

        self.pub_trajectory = None
        self.pub_control = None
        if self.output_mode == 'trajectory':
            self.pub_trajectory = self.create_publisher(Trajectory, '/openpilot/trajectory', 1)
            self.get_logger().info(
                'output mode: trajectory -> /openpilot/trajectory in '
                f"'{self.output_frame}'. Remap it onto the trajectory topic of the controller "
                'that should follow it.')
        else:
            self.pub_control = self.create_publisher(
                AckermannControlCommand, '/control/command/control_cmd', 1)
            rate = max(float(self.get_parameter('control.publish_rate_hz').value), 1.0)
            self.create_timer(1.0 / rate, self.publish_control,
                              callback_group=self.control_group)
            if self.control_enabled:
                self.get_logger().info(
                    'output mode: direct_action -> /control/command/control_cmd')
            else:
                self.get_logger().warn(
                    'output mode: direct_action, but control.enabled is false: no control command '
                    'will be published. Watch /openpilot/debug/markers first, then set '
                    'control.enabled:=true to drive.')

        self.get_logger().info('OpenPilotControllerNode is ready.')

    # -- callbacks --------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        intrinsics = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        if intrinsics[0, 0] <= 0.0 or intrinsics[1, 1] <= 0.0:
            self.get_logger().warn('CameraInfo has no usable focal length; ignoring',
                                   throttle_duration_sec=10.0)
            return
        if self.core.intrinsics is not None and np.allclose(self.core.intrinsics, intrinsics):
            return
        self.core.set_intrinsics(intrinsics)
        self.get_logger().info(
            f"camera intrinsics from CameraInfo: fx={intrinsics[0, 0]:.1f} fy={intrinsics[1, 1]:.1f} "
            f"cx={intrinsics[0, 2]:.1f} cy={intrinsics[1, 2]:.1f}")

    def velocity_callback(self, msg: VelocityReport):
        self.v_ego = float(msg.longitudinal_velocity)

    def odometry_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._ego_pose = (position.x, position.y, position.z, yaw)

    def image_callback(self, msg: Image):
        image = self._image_msg_to_rgb(msg)
        if image is None:
            return

        if not self.core.calibrated:
            self.core.set_intrinsics_from_hfov(msg.width, msg.height, self.hfov_deg)
            self.get_logger().warn(
                f"no CameraInfo yet; assuming a {self.hfov_deg:.1f} deg horizontal FOV for "
                f"{msg.width}x{msg.height}. Set camera.hfov_deg to match the simulator camera.")

        now = self.get_clock().now()
        dt = DT_MDL
        if self.last_image_time is not None:
            measured = (now - self.last_image_time).nanoseconds / 1e9
            if 0.0 < measured < 1.0:
                dt = measured
                self._tune_frame_skip(measured)
        self.last_image_time = now

        started = time.monotonic()
        try:
            action = self.core.process(image, self.v_ego, dt=dt)
        except Exception as exc:  # keep the timer publishing a safe command
            self.get_logger().error(f'inference failed: {exc}', throttle_duration_sec=5.0)
            return
        self.inference_times.append((time.monotonic() - started) * 1000.0)

        self._latest = (action, now)
        self._update_calibration(action)

        stamp = now.to_msg()
        if self.pub_markers is not None:
            self.pub_markers.publish(
                build_markers(action, stamp, self.frame_config, frame_id=self.output_frame))
        if self.pub_trajectory is not None:
            if self.output_frame != 'base_link' and self._ego_pose is None:
                self.get_logger().warn(
                    f"no odometry yet; cannot emit the trajectory in '{self.output_frame}'",
                    throttle_duration_sec=5.0)
            else:
                self.pub_trajectory.publish(
                    build_trajectory(action, stamp, self.frame_config, frame_id=self.output_frame,
                                     max_speed=self.core.control.max_speed,
                                     ego_pose=None if self.output_frame == 'base_link' else self._ego_pose))
        self._publish_debug_image(action, image, stamp, msg.header.frame_id)

        self._log_performance_metrics()

    def _update_calibration(self, action):
        """Feed the model's pose to the calibrator and apply what it settles on."""
        if self.calibrator is None:
            return
        rpy = self.calibrator.handle_pose(action.pose[:3], action.pose[3:],
                                          action.pose_std[:3], self.v_ego)
        if not self.calibrator.calibrated:
            # Silence here used to look identical to success; say why it is stalled.
            self.get_logger().info(f'calibration: {self.calibrator.status()}',
                                   throttle_duration_sec=20.0)
        if rpy is None:
            return
        if not np.allclose(rpy, self.core.calib_euler, atol=1e-4):
            self.core.set_calibration_euler(rpy)
        if not self.calibrator.calibrated:
            return
        if not self._calibration_reported:
            self._calibration_reported = True
            self.get_logger().info(
                f'camera calibrated from the model pose: pitch {rpy[1]:+.4f} rad '
                f'({np.degrees(rpy[1]):+.2f} deg), yaw {rpy[2]:+.4f} rad '
                f'({np.degrees(rpy[2]):+.2f} deg)')
        self._calibration_saves += 1
        if self.calibration_file and self._calibration_saves % 100 == 1:
            try:
                save_calibration(self.calibration_file, rpy, self.calibrator.valid_blocks)
            except Exception as exc:
                self.get_logger().warn(f'could not save calibration: {exc}',
                                       throttle_duration_sec=30.0)

    # Frames to run before timing anything: the first CUDA inferences and the rest
    # of the stack coming up are far slower than the steady state, and sampling them
    # would lock in a skip that is much too small.
    FRAME_SKIP_WARMUP = 40
    FRAME_SKIP_SAMPLES = 40

    def _tune_frame_skip(self, period: float):
        """Match the stacked-frame spacing to the rate frames actually get processed.

        The two frames the network stacks are `frame_skip` processed frames apart,
        so the spacing depends on the pipeline rate, not just the camera rate.
        Measured once, after warmup, because retuning later would drop the
        temporal history mid-drive.
        """
        if self.frame_skip_param > 0 or self._frame_periods is None:
            return
        if self._warmup_frames < self.FRAME_SKIP_WARMUP:
            self._warmup_frames += 1
            return
        self._frame_periods.append(period)
        if len(self._frame_periods) < self.FRAME_SKIP_SAMPLES:
            return
        median = float(np.median(self._frame_periods))
        self._frame_periods = None
        context_period = 1.0 / ModelConstants.MODEL_CONTEXT_FREQ
        skip = int(np.clip(round(context_period / median), 1, 8))
        current = self.core.runner.frame_skip
        self.get_logger().info(
            f'processing {1.0 / median:.1f} frames/s; the network wants its two frames '
            f'~{context_period * 1000:.0f} ms apart, so frame_skip {current} -> {skip} '
            f'({skip * median * 1000:.0f} ms)')
        if skip != current:
            self.core.set_frame_skip(skip)

    def _publish_debug_image(self, action, camera_rgb, stamp, frame_id: str):
        if self.pub_debug_image is None:
            return
        self._debug_image_counter += 1
        if self._debug_image_counter % self.debug_image_decimation:
            return
        try:
            picture = build_debug_image(
                action, self.core.last_model_frame,
                source=self.debug_image_source,
                camera_rgb=camera_rgb,
                camera_intrinsics=self.core.intrinsics,
                device_from_calib=self.core.device_from_calib,
                scale=self.debug_image_scale,
                hud_lines=self._hud_lines(action),
                path_z_offset=self.camera_height,
            )
        except Exception as exc:
            self.get_logger().warn(f'debug image failed: {exc}', throttle_duration_sec=5.0)
            return

        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_id or 'camera'
        message.height, message.width = picture.shape[0], picture.shape[1]
        message.encoding = 'bgr8'
        message.is_bigendian = 0
        message.step = picture.shape[1] * 3
        message.data = picture.tobytes()
        self.pub_debug_image.publish(message)

    def _hud_lines(self, action):
        mode = self.output_mode if self.output_mode == 'trajectory' else (
            'direct_action' if self.control_enabled else 'direct_action (control off)')
        inference = f'{np.mean(self.inference_times[-20:]):.0f}ms' if self.inference_times else '--'
        return (
            f'curv {action.desired_curvature:+.4f} 1/m   accel {action.desired_acceleration:+.2f} m/s2',
            f'v_ego {self.v_ego:.1f}   target {action.target_speed:.1f} m/s   {inference}',
            f'{mode}   {self.core.runner.active_providers[0].replace("ExecutionProvider", "")}',
        )

    # -- control ----------------------------------------------------------
    def publish_control(self):
        if self.pub_control is None or not self.control_enabled:
            return

        command = AckermannControlCommand()
        command.stamp = self.get_clock().now().to_msg()

        latest = self._latest
        action = latest[0] if latest is not None else None
        stale = latest is None or (
            (self.get_clock().now() - latest[1]).nanoseconds / 1e9 > self.command_timeout)

        if stale:
            if action is not None:
                self.get_logger().warn('no fresh model output; braking',
                                       throttle_duration_sec=1.0)
            self._publish_safe_stop(command)
            return

        steering = self.steering_gain * math.atan(self.wheel_base * action.desired_curvature)
        acceleration = action.desired_acceleration
        target_speed = action.target_speed

        if self._launch_active():
            # AWSIM tracks longitudinal.speed, so a zero speed request pins the kart
            # in place no matter what acceleration says.
            acceleration = max(acceleration, self.launch_acceleration)
            target_speed = max(target_speed, self.launch_speed)
        elif action.should_stop:
            acceleration = min(acceleration, self.core.control.max_deceleration)
            target_speed = 0.0

        if not (math.isfinite(steering) and math.isfinite(acceleration)):
            self.get_logger().error('model produced a non-finite command; braking',
                                    throttle_duration_sec=1.0)
            self._publish_safe_stop(command)
            return

        command.longitudinal.stamp = command.stamp
        command.longitudinal.speed = float(min(target_speed, self.core.control.max_speed))
        command.longitudinal.acceleration = float(acceleration)
        command.lateral.stamp = command.stamp
        command.lateral.steering_tire_angle = float(
            max(-self.max_steer, min(self.max_steer, steering)))
        self.pub_control.publish(command)

    def _launch_active(self) -> bool:
        """Departure aid: get rolling, then hand the longitudinal command back.

        The model reports shouldStop at a standstill, so without this the kart
        never leaves the line. Left permanently on it would instead drive the
        whole run itself, since the kart rarely exceeds the threshold.
        """
        if self.launch_speed <= 0.0:
            return False
        now = self.get_clock().now()
        if self.v_ego >= self.launch_speed:
            if self._launch_armed:
                self.get_logger().info(
                    f'reached {self.launch_speed:.1f} m/s; the model has the throttle now')
            self._launch_armed = False
            self._stalled_since = None
            return False
        if not self._launch_armed:
            # Re-arm only if the kart is genuinely stuck again, not merely slow.
            if self.v_ego > 0.3:
                self._stalled_since = None
            else:
                if self._stalled_since is None:
                    self._stalled_since = now
                elif (now - self._stalled_since).nanoseconds / 1e9 > self.launch_rearm_sec:
                    self.get_logger().info('stalled; pulling away again',
                                           throttle_duration_sec=5.0)
                    self._launch_armed = True
                    self._stalled_since = None
            return self._launch_armed
        return True

    def _publish_safe_stop(self, command: AckermannControlCommand):
        command.longitudinal.stamp = command.stamp
        command.lateral.stamp = command.stamp
        command.longitudinal.speed = 0.0
        command.longitudinal.acceleration = self.timeout_decel
        command.lateral.steering_tire_angle = 0.0
        self.pub_control.publish(command)

    # -- helpers ----------------------------------------------------------
    def _image_msg_to_rgb(self, msg: Image):
        conversions = {
            'rgb8': None,
            'bgr8': cv2.COLOR_BGR2RGB,
            'bgra8': cv2.COLOR_BGRA2RGB,
            'rgba8': cv2.COLOR_RGBA2RGB,
        }
        if msg.encoding not in conversions:
            self.get_logger().warn(f'unsupported image encoding: {msg.encoding}',
                                   throttle_duration_sec=5.0)
            return None
        try:
            channels = 4 if msg.encoding in ('bgra8', 'rgba8') else 3
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
            conversion = conversions[msg.encoding]
            return image.copy() if conversion is None else cv2.cvtColor(image, conversion)
        except Exception as exc:
            self.get_logger().error(f'image conversion failed: {exc}', throttle_duration_sec=5.0)
            return None

    def _log_performance_metrics(self):
        if not self.debug:
            self.inference_times.clear()
            return
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 <= self.log_interval:
            return
        if self.inference_times and self._latest is not None:
            average = float(np.mean(self.inference_times))
            action = self._latest[0]
            self.get_logger().info(
                f'inference {average:.1f} ms avg ({1000.0 / average:.1f} Hz), '
                f'max {np.max(self.inference_times):.1f} ms | '
                f'curvature {action.desired_curvature:+.4f} 1/m, '
                f'accel {action.desired_acceleration:+.2f} m/s^2, '
                f'v_ego {self.v_ego:.1f} m/s, '
                f'path 3s ({action.path[20, 0]:+.1f}, {action.path[20, 1]:+.1f}) m')
            self.inference_times.clear()
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotControllerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

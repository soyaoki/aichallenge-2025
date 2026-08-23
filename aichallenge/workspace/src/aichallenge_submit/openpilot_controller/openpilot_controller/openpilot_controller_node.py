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
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import MarkerArray
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import VelocityReport

from openpilot_controller.debug_image import build_debug_image
from openpilot_controller.debug_publisher import build_markers
from openpilot_controller.drive_helpers import DT_MDL
from openpilot_controller.frames import FrameConfig
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

        self.declare_parameter('camera.hfov_deg', 60.0)
        self.declare_parameter('camera.use_camera_info', True)
        self.declare_parameter('camera.calib_roll', 0.0)
        self.declare_parameter('camera.calib_pitch', 0.0)
        self.declare_parameter('camera.calib_yaw', 0.0)
        self.declare_parameter('camera.height', 0.7)

        self.declare_parameter('output.mode', 'direct_action')
        self.declare_parameter('output.frame_id', 'base_link')
        self.declare_parameter('output.publish_markers', True)
        self.declare_parameter('output.publish_debug_image', True)
        # model_input: the warped frame the network is fed. camera: the raw camera image.
        self.declare_parameter('output.debug_image_source', 'model_input')
        self.declare_parameter('output.debug_image_scale', 2)
        self.declare_parameter('output.debug_image_decimation', 1)

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
        self.declare_parameter('control.max_curvature', 0.2)
        self.declare_parameter('control.max_lateral_jerk', 5.0)
        self.declare_parameter('control.max_lateral_accel', 3.0)
        self.declare_parameter('control.max_acceleration', 2.0)
        self.declare_parameter('control.max_deceleration', -4.0)
        self.declare_parameter('control.max_speed', 30.0)
        self.declare_parameter('control.is_rhd', True)
        self.declare_parameter('control.timeout_deceleration', -2.0)

        self.declare_parameter('vehicle.wheel_base', 1.087)
        self.declare_parameter('vehicle.max_steering_tire_angle', 0.442)

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
        self.wheel_base = float(self.get_parameter('vehicle.wheel_base').value)
        self.max_steer = float(self.get_parameter('vehicle.max_steering_tire_angle').value)
        self.command_timeout = float(self.get_parameter('control.command_timeout_sec').value)
        self.timeout_decel = float(self.get_parameter('control.timeout_deceleration').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.log_interval = float(self.get_parameter('log_interval_sec').value)

        self.v_ego = 0.0
        # (action, stamp) written as one tuple so the control timer can never read
        # an action paired with the wrong timestamp.
        self._latest = None
        self.last_image_time = None
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
            self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback,
                                     sensor_qos, callback_group=self.inference_group)
        self.create_subscription(VelocityReport, '/vehicle/status/velocity_status',
                                 self.velocity_callback, sensor_qos,
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
        self.last_image_time = now

        started = time.monotonic()
        try:
            action = self.core.process(image, self.v_ego, dt=dt)
        except Exception as exc:  # keep the timer publishing a safe command
            self.get_logger().error(f'inference failed: {exc}', throttle_duration_sec=5.0)
            return
        self.inference_times.append((time.monotonic() - started) * 1000.0)

        self._latest = (action, now)

        stamp = now.to_msg()
        if self.pub_markers is not None:
            self.pub_markers.publish(
                build_markers(action, stamp, self.frame_config, frame_id=self.output_frame))
        if self.pub_trajectory is not None:
            self.pub_trajectory.publish(
                build_trajectory(action, stamp, self.frame_config, frame_id=self.output_frame,
                                 max_speed=self.core.control.max_speed))
        self._publish_debug_image(action, image, stamp, msg.header.frame_id)

        self._log_performance_metrics()

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

        steering = math.atan(self.wheel_base * action.desired_curvature)
        acceleration = action.desired_acceleration
        if action.should_stop:
            acceleration = min(acceleration, self.core.control.max_deceleration)

        if not (math.isfinite(steering) and math.isfinite(acceleration)):
            self.get_logger().error('model produced a non-finite command; braking',
                                    throttle_duration_sec=1.0)
            self._publish_safe_stop(command)
            return

        command.longitudinal.speed = float(action.target_speed)
        command.longitudinal.acceleration = float(acceleration)
        command.lateral.steering_tire_angle = float(
            max(-self.max_steer, min(self.max_steer, steering)))
        self.pub_control.publish(command)

    def _publish_safe_stop(self, command: AckermannControlCommand):
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

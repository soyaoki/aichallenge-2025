#!/usr/bin/env python3
"""ROS 2 node running comma.ai's driving_supercombo as an end-to-end controller.

Subscribes to the front camera and the vehicle velocity, runs the model, and
publishes ``AckermannControlCommand``. The command is published on a fixed
timer with a watchdog so a stalled camera brakes the vehicle instead of
latching the last steering angle.
"""

import math
import os
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport

from openpilot_controller.openpilot_controller_core import ControlConfig, OpenPilotCore
from openpilot_controller.drive_helpers import DT_MDL


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
        self.wheel_base = float(self.get_parameter('vehicle.wheel_base').value)
        self.max_steer = float(self.get_parameter('vehicle.max_steering_tire_angle').value)
        self.command_timeout = float(self.get_parameter('control.command_timeout_sec').value)
        self.timeout_decel = float(self.get_parameter('control.timeout_deceleration').value)
        self.debug = bool(self.get_parameter('debug').value)
        self.log_interval = float(self.get_parameter('log_interval_sec').value)

        self.v_ego = 0.0
        self.last_action = None
        self.last_action_time = None
        self.last_image_time = None
        self.inference_times = []
        self.last_log_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, '/image_raw', self.image_callback, sensor_qos)
        if self.use_camera_info:
            self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, sensor_qos)
        self.create_subscription(VelocityReport, '/vehicle/status/velocity_status',
                                 self.velocity_callback, sensor_qos)
        self.pub_control = self.create_publisher(
            AckermannControlCommand, '/control/command/control_cmd', 1)

        rate = max(float(self.get_parameter('control.publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / rate, self.publish_control)

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

        self.last_action = action
        self.last_action_time = now
        self._log_performance_metrics()

    # -- control ----------------------------------------------------------
    def publish_control(self):
        command = AckermannControlCommand()
        command.stamp = self.get_clock().now().to_msg()

        action = self.last_action
        stale = action is None or self.last_action_time is None or (
            (self.get_clock().now() - self.last_action_time).nanoseconds / 1e9 > self.command_timeout)

        if stale:
            if action is not None:
                self.get_logger().warn('no fresh model output; braking',
                                       throttle_duration_sec=1.0)
            command.longitudinal.speed = 0.0
            command.longitudinal.acceleration = self.timeout_decel
            command.lateral.steering_tire_angle = 0.0
            self.pub_control.publish(command)
            return

        steering = math.atan(self.wheel_base * action.desired_curvature)
        acceleration = action.desired_acceleration
        if action.should_stop:
            acceleration = min(acceleration, self.core.control.max_deceleration)

        if not (math.isfinite(steering) and math.isfinite(acceleration)):
            self.get_logger().error('model produced a non-finite command; braking',
                                    throttle_duration_sec=1.0)
            command.longitudinal.speed = 0.0
            command.longitudinal.acceleration = self.timeout_decel
            command.lateral.steering_tire_angle = 0.0
            self.pub_control.publish(command)
            return

        command.longitudinal.speed = float(action.target_speed)
        command.longitudinal.acceleration = float(acceleration)
        command.lateral.steering_tire_angle = float(
            max(-self.max_steer, min(self.max_steer, steering)))
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
        if self.inference_times:
            average = float(np.mean(self.inference_times))
            self.get_logger().info(
                f'inference {average:.1f} ms avg ({1000.0 / average:.1f} Hz), '
                f'max {np.max(self.inference_times):.1f} ms | '
                f'curvature {self.last_action.desired_curvature:+.4f} 1/m, '
                f'accel {self.last_action.desired_acceleration:+.2f} m/s^2, '
                f'v_ego {self.v_ego:.1f} m/s')
            self.inference_times.clear()
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = OpenPilotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

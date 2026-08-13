#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport


class VisionPilotAdapter(Node):
    """Translate the challenge vehicle API to Vision Pilot's scalar ROS API."""

    def __init__(self):
        super().__init__("vision_pilot_adapter")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("max_acceleration", 2.0)
        self.declare_parameter("max_deceleration", -6.0)
        self.declare_parameter("max_steering_tire_angle", 0.442)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.speed_pub = self.create_publisher(Float64, "/vehicle/speed", sensor_qos)
        self.control_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self._on_velocity, sensor_qos
        )
        self.create_subscription(Float64, "/vehicle/steering_cmd", self._on_steering, 1)
        self.create_subscription(Float64, "/vehicle/throttle_cmd", self._on_acceleration, 1)

        self.steering = 0.0
        self.acceleration = 0.0
        self.last_command_time = None
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._publish_control)

    def _on_velocity(self, msg):
        speed = Float64()
        speed.data = float(msg.longitudinal_velocity)
        self.speed_pub.publish(speed)

    def _on_steering(self, msg):
        self.steering = float(msg.data)
        self.last_command_time = self.get_clock().now()

    def _on_acceleration(self, msg):
        self.acceleration = float(msg.data)
        self.last_command_time = self.get_clock().now()

    def _publish_control(self):
        if self.last_command_time is None:
            return
        age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        timeout = float(self.get_parameter("command_timeout_sec").value)
        if age > timeout:
            acceleration = 0.0
            steering = 0.0
        else:
            acceleration = self.acceleration
            steering = self.steering

        if not math.isfinite(acceleration) or not math.isfinite(steering):
            self.get_logger().error("Vision Pilot produced a non-finite command")
            return

        command = AckermannControlCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.longitudinal.acceleration = max(
            float(self.get_parameter("max_deceleration").value),
            min(float(self.get_parameter("max_acceleration").value), acceleration),
        )
        max_steer = float(self.get_parameter("max_steering_tire_angle").value)
        command.lateral.steering_tire_angle = max(-max_steer, min(max_steer, steering))
        self.control_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = VisionPilotAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

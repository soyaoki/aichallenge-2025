#!/usr/bin/env python3
# trajectory_selector.py
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from autoware_internal_planning_msgs.msg import CandidateTrajectories
from autoware_auto_planning_msgs.msg import Trajectory as OutputTrajectory
from autoware_auto_planning_msgs.msg import TrajectoryPoint as OutputPoint
from sensor_msgs.msg import Image as RosImage

import cv2
from cv_bridge import CvBridge
from PIL import Image

from vlm_selector import VLMSelector, VLMCommand

class VlmTrajectorySelectorNode(Node):
    def __init__(self):
        super().__init__('vlm_trajectory_selector_node')

        self.declare_parameter('input_topic', '/input/trajectory')
        self.declare_parameter('output_topic', '/output/trajectory')
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.vlm_selector = VLMSelector(self.get_logger())

        # ROS 2 Publisher & Subscriber
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.publisher_ = self.create_publisher(OutputTrajectory, output_topic, qos_profile)
        self.candidate_sub = self.create_subscription(CandidateTrajectories, input_topic, self.candidate_callback, qos_profile)
        self.image_sub = self.create_subscription(RosImage, '/sensing/camera/image_raw', self.image_callback, qos_profile)

        # State variables
        self.bridge = CvBridge()
        self.latest_image = None
        # Variables for rate limiting and result caching (real time based)
        self.last_vlm_command = VLMCommand.GO_STRAIGHT
        self.last_known_sector = 1
        self.last_inference_time_sec = time.monotonic() # Save the last inference time in real time
        self.inference_interval_sec = 5.0               # Interval to perform inference (5 seconds in real time)

        self.get_logger().info(f"VLM Trajectory Selector started. Publishing to '{output_topic}'.")

    def image_callback(self, msg: RosImage):
        """Receives camera images and saves the latest image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def candidate_callback(self, msg: CandidateTrajectories):
        """Main process when receiving candidate trajectories."""
        now_sec = time.monotonic() # Get the current real time
        
        # Only perform VLM inference if 4 seconds have passed and an image is available
        if self.latest_image is not None and (now_sec - self.last_inference_time_sec) >= self.inference_interval_sec:
            self.get_logger().info("Interval passed. Performing VLM inference...")
            
            # Decision by VLM
            vlm_command, current_sector = self.vlm_selector.infer(
                self.latest_image, 
                self.last_vlm_command, 
                self.last_known_sector
            )
            
            # Only update result and time if inference is successful
            if vlm_command != VLMCommand.NONE:
                self.last_vlm_command = vlm_command
                self.last_known_sector = current_sector
                self.last_inference_time_sec = now_sec # Update in real time
            else:
                self.get_logger().warn("VLM inference failed. Reusing last known command.")
                
        else:
            # If 4 seconds have not passed, log and reuse the previous result
            if self.latest_image is not None:
                self.get_logger().info(f"Reusing last command: {self.last_vlm_command.name}")

        self.get_logger().info(f"Command: {self.last_vlm_command.name}, Current sector: {self.last_known_sector}")
        # Convert VLM decision result to index
        if self.last_vlm_command == VLMCommand.TURN_RIGHT:
            selected_index = 0
        elif self.last_vlm_command == VLMCommand.TURN_LEFT:
            selected_index = 1
        else: # GO_STRAIGHT
            selected_index = 2
        
        self.get_logger().info(f"Selected trajectory index: {selected_index}")

        if selected_index >= len(msg.candidate_trajectories):
            self.get_logger().error(f"Selected index {selected_index} is out of range.")
            return
        
        # Get the selected candidate trajectory
        selected_candidate = msg.candidate_trajectories[selected_index]
        
        # Copy the data of the selected candidate trajectory to a new message
        output_msg = OutputTrajectory()
        output_msg.header = selected_candidate.header
        selected_candidate = msg.candidate_trajectories[selected_index]
        output_msg.header = selected_candidate.header

        # Loop to copy each point
        for input_point in selected_candidate.points:
            output_point = OutputPoint()
            # Copy all internal fields
            output_point.time_from_start = input_point.time_from_start
            output_point.pose = input_point.pose
            output_point.longitudinal_velocity_mps = input_point.longitudinal_velocity_mps
            output_point.lateral_velocity_mps = input_point.lateral_velocity_mps
            output_point.acceleration_mps2 = input_point.acceleration_mps2
            output_point.heading_rate_rps = input_point.heading_rate_rps
            output_point.front_wheel_angle_rad = input_point.front_wheel_angle_rad
            output_point.rear_wheel_angle_rad = input_point.rear_wheel_angle_rad

            output_msg.points.append(output_point)

        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = VlmTrajectorySelectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as e:
        print(f"Node shutting down: {e}")
    finally:
        if 'node' in locals() and rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

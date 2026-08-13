// Headless AI Challenge integration of Vision Pilot.
#include <chrono>
#include <algorithm>
#include <memory>
#include <sstream>
#include <thread>

#include <common/utils.hpp>
#include <config/vision_pilot_config.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <planning/planning.hpp>

#include <camera_ros2_interface/camera_ros2_interface.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/imgproc.hpp>
#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    Config cfg;
    try {
        cfg = load_vision_pilot_config();
    } catch (const std::exception& e) {
        VP_ERROR("Config: %s", e.what());
        return 1;
    }

    auto camera = std::make_shared<CameraRos2Interface>(cfg.source.input_camera_topic);
    auto vehicle = std::make_shared<VehicleRos2Interface>(
        cfg.vehicle_speed_topic, cfg.vehicle_steering_topic, cfg.vehicle_acceleration_topic);
    auto debug_node = std::make_shared<rclcpp::Node>("vision_pilot_debug");
    auto debug_image_pub = debug_node->create_publisher<sensor_msgs::msg::Image>(
        "/vision_pilot/debug/image", rclcpp::SensorDataQoS());
    auto debug_outputs_pub = debug_node->create_publisher<std_msgs::msg::String>(
        "/vision_pilot/debug/model_outputs", 1);

    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine);
    vm::InferencePipeline pipeline(engine, cfg.inference);
    Planner planner(cfg.speed_limit, cfg.L);

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    cv::Mat frame, warped, resized;
    bool homography_ready = false;
    cv::Mat homography = load_matrix("H.yaml", "H");

    while (rclcpp::ok()) {
        auto [ok, latest] = camera->get_latest_frame();
        if (!ok || latest.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        frame = std::move(latest);
        preprocessor.preprocess(frame, warped, resized, net_size);
        if (!homography_ready) {
            pipeline.set_H_resized(homography, frame.size());
            homography_ready = true;
        }

        const auto result = pipeline.process(warped, resized);
        if (!result) continue;

        const double ego_speed = vehicle->read();
        constexpr double max_cipo_distance = 150.0;
        const bool has_cipo = result->cipo.valid && result->cipo.distance_m < max_cipo_distance;
        const double cipo_speed = has_cipo ? result->cipo.velocity_ms : cfg.speed_limit;
        const Plan plan = planner.compute_plan(
            result->lateral.cte_m,
            result->lateral.yaw_rad,
            result->lateral.curvature,
            ego_speed,
            has_cipo,
            cipo_speed,
            result->cipo.distance_m);

        const auto valid_waypoints = std::count_if(
            result->auto_steer.h_vector.begin(), result->auto_steer.h_vector.end(),
            [](float confidence) { return confidence >= 0.5f; });
        std::ostringstream json;
        json << "{\"frame_id\":" << result->frame_id
             << ",\"latency_ms\":{\"wall\":" << result->wall_ms
             << ",\"pre\":" << result->pre_ms
             << ",\"auto_drive\":" << result->ad_ms
             << ",\"auto_steer\":" << result->as_ms
             << ",\"auto_speed\":" << result->asp_ms << "}"
             << ",\"auto_drive\":{\"valid\":" << std::boolalpha << result->auto_drive.valid
             << ",\"distance_normalized\":" << result->auto_drive.dist_normalized
             << ",\"curvature_raw\":" << result->auto_drive.curvature_raw
             << ",\"cipo_probability\":" << result->auto_drive.flag_prob << "}"
             << ",\"auto_steer\":{\"valid\":" << result->auto_steer.valid
             << ",\"valid_waypoints\":" << valid_waypoints << "}"
             << ",\"auto_speed\":{\"valid\":" << result->auto_speed.valid
             << ",\"detections\":" << result->auto_speed.detections.size() << "}"
             << ",\"fusion\":{\"cte_m\":" << result->lateral.cte_m
             << ",\"yaw_rad\":" << result->lateral.yaw_rad
             << ",\"curvature\":" << result->lateral.curvature
             << ",\"cipo_valid\":" << result->cipo.valid
             << ",\"cipo_distance_m\":" << result->cipo.distance_m << "}"
             << ",\"plan\":{\"steering_rad\":"
             << (plan.steering.empty() ? 0.0 : plan.steering.front())
             << ",\"acceleration_mps2\":" << plan.acceleration << "}}";
        std_msgs::msg::String outputs_msg;
        outputs_msg.data = json.str();
        debug_outputs_pub->publish(outputs_msg);

        cv::Mat debug_image = resized.clone();
        for (std::size_t i = 0; i < result->auto_steer.xp.size(); ++i) {
            if (result->auto_steer.h_vector[i] < 0.5f) continue;
            const int u = std::clamp(
                static_cast<int>(result->auto_steer.xp[i] * debug_image.cols),
                0, debug_image.cols - 1);
            const int v = static_cast<int>(
                i * (debug_image.rows - 1) / (result->auto_steer.xp.size() - 1));
            cv::circle(debug_image, {u, v}, 4, {0, 255, 0}, cv::FILLED);
        }
        for (const auto& detection : result->auto_speed.detections) {
            cv::rectangle(
                debug_image,
                {static_cast<int>(detection.x1), static_cast<int>(detection.y1)},
                {static_cast<int>(detection.x2), static_cast<int>(detection.y2)},
                {0, 0, 255}, 2);
        }
        std::ostringstream hud;
        hud << "AD d=" << result->auto_drive.dist_normalized
            << " curv=" << result->auto_drive.curvature_raw
            << " cipo=" << result->auto_drive.flag_prob
            << " | AS pts=" << valid_waypoints
            << " | ASP det=" << result->auto_speed.detections.size();
        cv::putText(debug_image, hud.str(), {12, 28}, cv::FONT_HERSHEY_SIMPLEX,
                    0.55, {0, 255, 255}, 2, cv::LINE_AA);
        std::ostringstream timing;
        timing << "wall=" << result->wall_ms << "ms AD=" << result->ad_ms
               << " AS=" << result->as_ms << " ASP=" << result->asp_ms
               << " steer=" << (plan.steering.empty() ? 0.0 : plan.steering.front());
        cv::putText(debug_image, timing.str(), {12, 54}, cv::FONT_HERSHEY_SIMPLEX,
                    0.55, {255, 255, 0}, 2, cv::LINE_AA);
        auto image_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg();
        image_msg->header.stamp = debug_node->now();
        image_msg->header.frame_id = "camera_optical_link";
        debug_image_pub->publish(*image_msg);

        vehicle->write(plan.steering.empty() ? 0.0 : plan.steering.front(), plan.acceleration);
    }

    rclcpp::shutdown();
    return 0;
}

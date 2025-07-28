#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <creos_sdk_msgs/msg/state.hpp>
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <creos_sdk_msgs/msg/control_source.hpp>
#include <creos_sdk_msgs/srv/send_command.hpp>

#include <common/logging.hpp>
#include <common/remote_controller_interface.hpp>
#include <common/flight_controller.hpp>

#include "aruco_pose_estimator.hpp"
#include "drone_interface.hpp"
#include "landing_controller.hpp"
#include "landing_planner.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("aruco_landing_main");  

    InitLogging(node);

    // === Controller Setup ===
    std::shared_ptr<RemoteControllerInterface> controller_;
    std::string controller_type = node->declare_parameter<std::string>("controller_type", "herelink");

    if (controller_type == "herelink") {
        controller_ = CreateRemoteController(ControllerType::kHerelink, node->get_logger());
    } else if (controller_type == "jeti") {
        controller_ = CreateRemoteController(ControllerType::kJeti, node->get_logger());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Unknown controller type: %s", controller_type.c_str());
        return 1;
    }

    bool execution_active = false;
    controller_->RegisterActivationButtonCallback(
        [&execution_active, &node]() {
            execution_active = !execution_active;
            RCLCPP_INFO(node->get_logger(), "Execution active: %s", execution_active ? "true" : "false");
        });

    // === Component Initialization ===
    auto drone_interface = std::make_shared<aruco_landing::DroneInterface>(node);
    auto pose_estimator  = std::make_shared<aruco_landing::ArucoPoseEstimator>(node);
    auto landing_controller = std::make_shared<aruco_landing::LandingController>(drone_interface, node);

    // === Controller Input ===
    auto controller_sub = node->create_subscription<sensor_msgs::msg::Joy>(
        "/robot/joy", rclcpp::SensorDataQoS(), controller_->GetControllerStateCallback());

    // === Periodic Timer ===
    auto timer = node->create_wall_timer(100ms, [=]() {
        pose_estimator->update_pose();

        if (!execution_active)
            return;

        if (pose_estimator->is_pose_ready()) {
            auto averaged_pose = pose_estimator->get_latest_pose();
            if (averaged_pose) {
                geometry_msgs::msg::PoseWithCovarianceStamped current_pose;
                current_pose.header.stamp = node->now();
                current_pose.header.frame_id = "world";
                current_pose.pose.pose.position.x = averaged_pose->translation().x();
                current_pose.pose.pose.position.y = averaged_pose->translation().y();
                current_pose.pose.pose.position.z = averaged_pose->translation().z();
                current_pose.pose.pose.orientation.w = 1.0;  // Flat orientation (heading 0)

                auto waypoints = aruco_landing::generate_landing_path(current_pose);
                landing_controller->set_waypoints(waypoints);
                drone_interface->set_autonomous_mode();
            }
        }
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

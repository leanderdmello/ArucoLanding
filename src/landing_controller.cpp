#include "landing_controller.hpp"

namespace aruco_landing {

LandingController::LandingController(
    std::shared_ptr<DroneInterface> drone_interface,
    rclcpp::Node::SharedPtr node)
: drone_interface_(std::move(drone_interface)), node_(std::move(node)) {
    using std::placeholders::_1;

    // Parameter for max speed
    rcl_interfaces::msg::ParameterDescriptor speed_param;
    speed_param.description = "Maximum landing speed of the drone in meters per second.";
    max_speed_ = node_->declare_parameter("max_speed", 0.2, speed_param);

    // Timer for sending landing waypoints
    waypoint_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LandingController::send_next_waypoint, this)
    );
}

void LandingController::set_waypoints(const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> &wps) {
    waypoints_ = wps;
    current_wp_index_ = 0;
    active_ = true;
}

void LandingController::stop() {
    active_ = false;
    current_wp_index_ = 0;
    waypoints_.clear();
}

void LandingController::send_next_waypoint() {
    if (!active_ || current_wp_index_ >= waypoints_.size()) {
        return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_cov = drone_interface_->get_current_pose();
    geometry_msgs::msg::Pose current_pose = current_pose_cov.pose.pose;
    const auto &target_pose = waypoints_[current_wp_index_];

    double dx = target_pose.pose.pose.position.x - current_pose.position.x;
    double dy = target_pose.pose.pose.position.y - current_pose.position.y;
    double dz = target_pose.pose.pose.position.z - current_pose.position.z;

    double xy_error = std::hypot(dx, dy);
    double alignment_margin = 0.1; // 10 cm margin for smoother alignment

    // Realign in x/y if misaligned during descent
    if (xy_error > alignment_margin) {
        RCLCPP_WARN(node_->get_logger(), "Misalignment detected (%.2f m). Realigning to center...", xy_error);

        geometry_msgs::msg::PoseWithCovarianceStamped align_wp = target_pose;
        align_wp.pose.pose.position.x = 0.0;
        align_wp.pose.pose.position.y = 0.0;
        align_wp.pose.pose.position.z = current_pose.position.z;

        // Insert realignment waypoint just before current target
        waypoints_.insert(waypoints_.begin() + current_wp_index_, align_wp);
        return;
    }

    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < 0.05) {
        current_wp_index_++;
        return;
    }

    // Clamp to max_speed
    double scale = std::min(1.0, max_speed_ / (distance + 1e-6));

    geometry_msgs::msg::PoseWithCovarianceStamped limited_pose = target_pose;
    limited_pose.pose.pose.position.x = current_pose.position.x + dx * scale;
    limited_pose.pose.pose.position.y = current_pose.position.y + dy * scale;
    limited_pose.pose.pose.position.z = current_pose.position.z + dz * scale;

    drone_interface_->update_waypoint(limited_pose);
}

}  
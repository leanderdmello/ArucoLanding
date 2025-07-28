#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "drone_interface.hpp"

namespace aruco_landing {

class LandingController {
public:
    LandingController(std::shared_ptr<DroneInterface> drone_interface,
                      rclcpp::Node::SharedPtr node);

    void set_waypoints(const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> &wps);
    void stop();

private:
    void send_next_waypoint();

    std::shared_ptr<DroneInterface> drone_interface_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr waypoint_timer_;

    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> waypoints_;
    size_t current_wp_index_ = 0;
    bool active_ = false;

    double max_speed_;
};

}  
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <creos_sdk_msgs/msg/state.hpp>
#include <creos_sdk_msgs/msg/control_source.hpp>
#include <creos_sdk_msgs/msg/state_reference.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace aruco_landing {

class DroneInterface {
public:
    explicit DroneInterface(rclcpp::Node::SharedPtr node);

    void set_autonomous_mode();
    void set_manual_mode();
    void update_waypoint(const geometry_msgs::msg::PoseWithCovarianceStamped &pose_msg);

    geometry_msgs::msg::PoseWithCovarianceStamped get_global_pose() const;
    creos_sdk_msgs::msg::State get_state() const;

private:
    void state_callback(const creos_sdk_msgs::msg::State::SharedPtr msg);
    void global_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<creos_sdk_msgs::msg::StateReference>::SharedPtr state_ref_pub_;
    rclcpp::Publisher<creos_sdk_msgs::msg::ControlSource>::SharedPtr control_source_pub_;

    rclcpp::Subscription<creos_sdk_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_sub_;

    creos_sdk_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseWithCovarianceStamped global_pose_;
};

} 
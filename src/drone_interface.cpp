#include "drone_interface.hpp"

namespace aruco_landing {

DroneInterface::DroneInterface(rclcpp::Node::SharedPtr node)
: node_(std::move(node)) {
    using std::placeholders::_1;

    // Publishers
    state_ref_pub_ = node_->create_publisher<creos_sdk_msgs::msg::StateReference>("/robot/state_reference", 10);
    control_source_pub_ = node_->create_publisher<creos_sdk_msgs::msg::ControlSource>("/robot/control_source", 10);

    // Subscribers
    state_sub_ = node_->create_subscription<creos_sdk_msgs::msg::State>(
        "/robot/state", rclcpp::QoS(10),
        std::bind(&DroneInterface::state_callback, this, _1)
    );

    global_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/robot/global_pose", rclcpp::QoS(10),
        std::bind(&DroneInterface::global_pose_callback, this, _1)
    );

    // Default mode: manual
    set_manual_mode();
}

void DroneInterface::set_autonomous_mode() {
    creos_sdk_msgs::msg::ControlSource msg;
    msg.source = creos_sdk_msgs::msg::ControlSource::SOURCE_VERTEX;
    control_source_pub_->publish(msg);
}

void DroneInterface::set_manual_mode() {
    creos_sdk_msgs::msg::ControlSource msg;
    msg.source = creos_sdk_msgs::msg::ControlSource::SOURCE_REMOTE;
    control_source_pub_->publish(msg);
}

void DroneInterface::update_waypoint(const geometry_msgs::msg::PoseWithCovarianceStamped &pose_msg) {
    creos_sdk_msgs::msg::StateReference ref_msg;
    ref_msg.header.stamp = node_->now();
    ref_msg.pose = pose_msg.pose.pose;
    ref_msg.translation_mode = creos_sdk_msgs::msg::StateReference::TRANSLATION_MODE_POSITION;
    ref_msg.orientation_mode = creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ATTITUDE;
    state_ref_pub_->publish(ref_msg);
}

void DroneInterface::state_callback(const creos_sdk_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
}

void DroneInterface::global_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    global_pose_ = *msg;
}

geometry_msgs::msg::PoseWithCovarianceStamped DroneInterface::get_global_pose() const {
    return global_pose_;
}

creos_sdk_msgs::msg::State DroneInterface::get_state() const {
    return current_state_;
}

} 

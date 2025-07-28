#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <deque>
#include <geometry_msgs/msg/pose.hpp>

namespace aruco_landing {

class ArucoPoseEstimator {
public:
    explicit ArucoPoseEstimator(rclcpp::Node::SharedPtr node);

    bool estimate(const cv::Mat& image, geometry_msgs::msg::Pose& pose_out);

private:
    rclcpp::Node::SharedPtr node_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::unique_ptr<cv::aruco::ArucoDetector> detector_;

    std::deque<std::pair<rclcpp::Time, geometry_msgs::msg::Pose>> pose_buffer_;
};

}  

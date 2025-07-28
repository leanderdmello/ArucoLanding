#include "aruco_pose_estimator.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/logging.hpp>
#include <chrono>
#include <unordered_map>
#include <deque>
#include <tf2/LinearMath/Quaternion.h>

namespace aruco_landing {

ArucoPoseEstimator::ArucoPoseEstimator(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
    CREOS_INFO(node_->get_logger(), "Initializing ArUco Pose Estimator...");

    // Camera calibration parameters
    camera_matrix_ = (cv::Mat1d(3, 3) <<
        1336.135004356143, 0.0, 956.1451052158596,
        0.0, 1333.886438722237, 540.7159410836905,
        0.0, 0.0, 1.0);

    dist_coeffs_ = (cv::Mat1d(1, 5) <<
        -0.3478036670023198, 0.1341258850882104,
        0.0001174232535739054, 0.00005508984106933928,
        -0.02215131320202772);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    detector_ = std::make_unique<cv::aruco::ArucoDetector>(dictionary_);
}

bool ArucoPoseEstimator::estimate(const cv::Mat& image, geometry_msgs::msg::Pose& pose_out)
{
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_->detectMarkers(image, corners, ids);
    if (ids.empty()) return false;

    std::unordered_map<int, double> marker_lengths = {{24, 0.022}, {122, 0.154}};
    int selected_index = -1;
    double selected_length = 0.0;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] == 24) {
            selected_index = static_cast<int>(i);
            selected_length = marker_lengths[24];
            break;
        } else if (selected_index == -1 && marker_lengths.count(ids[i])) {
            selected_index = static_cast<int>(i);
            selected_length = marker_lengths[ids[i]];
        }
    }
    if (selected_index == -1) return false;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers({corners[selected_index]}, selected_length,
                                         camera_matrix_, dist_coeffs_, rvecs, tvecs);
    if (rvecs.empty() || tvecs.empty()) return false;

    cv::Mat R;
    cv::Rodrigues(rvecs[0], R);
    tf2::Matrix3x3 tf_R(
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    tf2::Quaternion q;
    tf_R.getRotation(q);

    geometry_msgs::msg::Pose current_pose;
    current_pose.position.x = tvecs[0][0];
    current_pose.position.y = tvecs[0][1];
    current_pose.position.z = tvecs[0][2];
    current_pose.orientation.x = q.x();
    current_pose.orientation.y = q.y();
    current_pose.orientation.z = q.z();
    current_pose.orientation.w = q.w();

    // Add to buffer
    rclcpp::Time now = node_->get_clock()->now();
    pose_buffer_.emplace_back(now, current_pose);

    // Remove old entries
    while (!pose_buffer_.empty() && (now - pose_buffer_.front().first).seconds() > 3.0) {
        pose_buffer_.pop_front();
    }

    // Average the buffer
    if (pose_buffer_.empty()) return false;
    double sum_x = 0, sum_y = 0, sum_z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 0;
    for (const auto& [_, p] : pose_buffer_) {
        sum_x += p.position.x;
        sum_y += p.position.y;
        sum_z += p.position.z;
        qx += p.orientation.x;
        qy += p.orientation.y;
        qz += p.orientation.z;
        qw += p.orientation.w;
    }
    size_t n = pose_buffer_.size();
    pose_out.position.x = sum_x / n;
    pose_out.position.y = sum_y / n;
    pose_out.position.z = sum_z / n;
    tf2::Quaternion avg_q(qx / n, qy / n, qz / n, qw / n);
    avg_q.normalize();
    pose_out.orientation.x = avg_q.x();
    pose_out.orientation.y = avg_q.y();
    pose_out.orientation.z = avg_q.z();
    pose_out.orientation.w = avg_q.w();

    return true;
}

} 

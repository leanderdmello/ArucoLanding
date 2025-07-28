#pragma once

#include <vector>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <eigen3/Eigen/Dense>

namespace aruco_landing {

std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> generate_landing_path(
    const geometry_msgs::msg::PoseWithCovarianceStamped &current_pose,
    double descend_step = 0.1,
    double xy_tolerance = 0.02);

} 
#include "landing_planner.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace aruco_landing {

std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> generate_landing_path(
    const geometry_msgs::msg::PoseWithCovarianceStamped &current_pose,
    double descend_step,
    double xy_tolerance)
{
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> waypoints;

    double x = current_pose.pose.pose.position.x;
    double y = current_pose.pose.pose.position.y;
    double z = current_pose.pose.pose.position.z;

    geometry_msgs::msg::PoseWithCovarianceStamped pose = current_pose;

    // Step 1: Move to (0, 0, z)
    pose.pose.pose.position.x = 0.0;
    pose.pose.pose.position.y = 0.0;
    waypoints.push_back(pose);

    // Step 2: Descend in steps, correcting x/y each time if needed
    while (z < -0.05) {
        z = std::min(z + descend_step, 0.05);
        pose.pose.pose.position.z = z;

        if (std::abs(pose.pose.pose.position.x) > xy_tolerance ||
        std::abs(pose.pose.pose.position.y) > xy_tolerance) {

        waypoints.push_back(pose);
        }

    return waypoints;
    }

} 
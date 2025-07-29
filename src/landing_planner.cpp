#include "landing_planner.hpp"
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

namespace aruco_landing {

std::vector<creos_sdk_msgs::msg::StateReference> generate_landing_path(
    const geometry_msgs::msg::Pose &current_pose,
    double descend_step,
    double xy_tolerance)
{
    std::vector<creos_sdk_msgs::msg::StateReference> waypoints;

    double z = current_pose.position.z;

    // Step 1: Move to (0, 0, z)
    creos_sdk_msgs::msg::StateReference ref;
    ref.pose.position.x = 0.0;
    ref.pose.position.y = 0.0;
    ref.pose.position.z = z;

    ref.twist.linear.x = 0.0;
    ref.twist.linear.y = 0.0;
    ref.twist.linear.z = 0.0;

    ref.accel.linear.x = 0.0;
    ref.accel.linear.y = 0.0;
    ref.accel.linear.z = 0.0;

    ref.translation_mode = creos_sdk_msgs::msg::StateReference::TRANSLATION_MODE_POSITION;
    ref.orientation_mode = creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ATTITUDE;

    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0.0);
    ref.pose.orientation.x = quat.x();
    ref.pose.orientation.y = quat.y();
    ref.pose.orientation.z = quat.z();
    ref.pose.orientation.w = quat.w();

    waypoints.push_back(ref);

    // Step 2: Descend to z = 0.05
    while (z < 0.05) {
        z = std::min(z + descend_step, 0.05);
        ref.pose.position.z = z;

        ref.twist.linear.z = 0.2;  
        waypoints.push_back(ref);
    }

    return waypoints;
}

} 

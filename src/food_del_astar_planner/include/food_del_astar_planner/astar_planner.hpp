#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

using namespace std::placeholders;

class AStarPlanner : public rclcpp::Node {

public:
    AStarPlanner();

private:

    /* ================= ROS Subscriber ================= */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;

    /* ================= Callbacks ================= */
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_);

    /* ================= Robot Position ================= */
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool robot_pose_received_ = false;

};

#endif // ASTAR_PLANNER_HPP

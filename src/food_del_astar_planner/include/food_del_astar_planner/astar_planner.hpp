#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <cmath>

using namespace std::placeholders;

class AStarPlanner : public rclcpp::Node {

public:
    AStarPlanner();

private:

    /* ================= ROS Subscriber ================= */
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    /* ================= Callbacks ================= */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg_);

    /* ================= Robot Position ================= */
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool robot_pose_received_ = false;

    /* ================= Map ==================== */
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    bool map_received_ = false;
    double map_origin_x_;
    double map_origin_y_;
    double map_resolution_;

    /* ================= Goal Position ================= */
    double goal_x_;
    double goal_y_;
    bool goal_pose_received_ = false;

    /* ===============Quaternion================= */
    tf2::Quaternion robot_quaternion_;

    /* ================= Structs ================= */
    struct Cell {
        int row;
        int col;

        bool operator==(const Cell & other) const {
            if (row != other.row) {
                return false;
            }
            if (col != other.col) {
                return false;
            }
            return true;
        }
    };

    /* ================= Helper Functions ================= */
    Cell worldToMap(double wx, double wy);
    bool   isValid(const Cell & cell);
    int cellToIndex(const Cell & cell);

    /* =================AStar Algorithm================= */
    void runAStar();

};

#endif // ASTAR_PLANNER_HPP

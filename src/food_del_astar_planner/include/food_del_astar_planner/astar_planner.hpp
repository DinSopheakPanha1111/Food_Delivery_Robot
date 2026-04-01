#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <cmath>
#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>

using namespace std::placeholders;

class AStarPlanner : public rclcpp::Node {

public:
    AStarPlanner();

private:

    /* ================= ROS Subscriber ================= */
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    /* ================= ROS Publisher ================= */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

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
    
    struct AStarNode {
        Cell   cell;
        double g;   // cost from start to this cell
        double h;   // estimated cost from this cell to goal
        double f;   // g + h

        bool operator>(const AStarNode & other) const {
            return f > other.f;
        }
    };
    
    struct CellHash {
        std::size_t operator()(const Cell & c) const {
            return std::hash<int>()(c.row * 10000 + c.col);
        }
    };

    /* ================= Helper Functions ================= */
    Cell worldToMap(double wx, double wy);
    bool isValid(const Cell & cell);
    int  cellToIndex(const Cell & cell);
    bool isObstacle(const Cell & cell);

    /* ================= Path Smoothing ================= */
    bool hasLineOfSight(const Cell & from, const Cell & to);
    std::vector<Cell> smoothPath(const std::vector<Cell> & path);

    /* =================AStar Algorithm================= */
    void runAStar();

    /* ================= Inflation ================= */
    std::vector<int8_t> inflated_map_;
    int inflation_radius_ = 3;  // cells to inflate around obstacles

    void inflateMap();

};

#endif // ASTAR_PLANNER_HPP
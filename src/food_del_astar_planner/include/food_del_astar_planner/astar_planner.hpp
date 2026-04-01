#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
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
    ~AStarPlanner();

    // Expose so main() can add it to the executor (same pattern as DWB)
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> get_global_costmap()
    {
        return global_costmap_;
    }

private:

    /* ================= Global Costmap (owned) ================= */
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_;
    bool costmap_ready_ = false;
    rclcpp::TimerBase::SharedPtr activate_timer_;

    /* ================= ROS Subscribers ================= */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

    /* ================= ROS Publisher ================= */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    /* ================= Callbacks ================= */
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_);
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg_);

    /* ================= Robot Position ================= */
    double robot_x_   = 0.0;
    double robot_y_   = 0.0;
    double robot_yaw_ = 0.0;
    bool   robot_pose_received_ = false;
    tf2::Quaternion robot_quaternion_;

    /* ================= Goal Position ================= */
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    bool   goal_pose_received_ = false;

    /* ================= Structs ================= */
    struct Cell {
        int row;
        int col;

        bool operator==(const Cell & other) const {
            return row == other.row && col == other.col;
        }
    };

    struct AStarNode {
        Cell   cell;
        double g;
        double h;
        double f;

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
    Cell worldToMap(double wx, double wy,
                    nav2_costmap_2d::Costmap2D * costmap);
    int  cellToIndex(const Cell & cell,
                     nav2_costmap_2d::Costmap2D * costmap);
    bool isValid(const Cell & cell,
                 nav2_costmap_2d::Costmap2D * costmap);
    bool isObstacle(const Cell & cell,
                    nav2_costmap_2d::Costmap2D * costmap);

    /* ================= A* Algorithm ================= */
    void runAStar();
};

#endif // ASTAR_PLANNER_HPP
#include "food_del_astar_planner/astar_planner.hpp"

// =============== Constructor ===============

AStarPlanner::AStarPlanner() : Node("astar_planner_node")
{
    // 1. Map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        std::bind(&AStarPlanner::mapCallback, this, _1)
    );

    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&AStarPlanner::amclPoseCallback, this, _1)
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&AStarPlanner::goalPoseCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscriber to /amcl_pose created");
}

// =============== Callbacks ===============

// MAP callback
void AStarPlanner::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_)
{
    map_ = map_msg_;
    map_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Map received:");
    RCLCPP_INFO(this->get_logger(), "  width      : %d cells",  map_msg_->info.width);
    RCLCPP_INFO(this->get_logger(), "  height     : %d cells",  map_msg_->info.height);
    RCLCPP_INFO(this->get_logger(), "  resolution : %.3f m/cell", map_msg_->info.resolution);
    RCLCPP_INFO(this->get_logger(), "  origin x   : %.3f m",    map_msg_->info.origin.position.x);
    RCLCPP_INFO(this->get_logger(), "  origin y   : %.3f m",    map_msg_->info.origin.position.y);
    RCLCPP_INFO(this->get_logger(), "  total cells: %zu",       map_msg_->data.size());

    // print the first 10 cell values so we can see what the data looks like
    RCLCPP_INFO(this->get_logger(), "First 10 cell values:");
    for (int i = 0; i < 10; i++) {
        RCLCPP_INFO(this->get_logger(), "  data[%d] = %d", i, (int)map_msg_->data[i]);
    }
}

// AMCL Pose callback
void AStarPlanner::amclPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_)
{
    // Extract the robot pose from the message
    robot_x_   = pose_msg_->pose.pose.position.x;
    robot_y_   = pose_msg_->pose.pose.position.y;
    robot_quaternion_ = tf2::Quaternion(
        pose_msg_->pose.pose.orientation.x,
        pose_msg_->pose.pose.orientation.y,
        pose_msg_->pose.pose.orientation.z,
        pose_msg_->pose.pose.orientation.w
    );
    robot_yaw_ = tf2::getYaw(robot_quaternion_);
    robot_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
    "Robot pose received — x: %.3f m  y: %.3f m  yaw: %.1f deg",
    robot_x_, robot_y_, robot_yaw_ * (180.0 / M_PI));
}

// Goal Pose Callback
void AStarPlanner::goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg_)
{
    goal_x_ = goal_msg_->pose.position.x;
    goal_y_ = goal_msg_->pose.position.y;
    goal_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
    "Goal pose received — x: %.3f m  y: %.3f m",
    goal_x_, goal_y_);

    // kick off A* algorithm
    runAStar();
}

// =============== Helper Functions ===============
AStarPlanner::Cell AStarPlanner::worldToMap(double wx, double wy)
{
    Cell cell;
    cell.col = static_cast<int>((wx - map_origin_x_) / map_resolution_);
    cell.row = static_cast<int>((wy - map_origin_y_) / map_resolution_);
    return cell;
}

int AStarPlanner::cellToIndex(const Cell & cell)
{
    return cell.row * static_cast<int>(map_->info.width) + cell.col;
}

bool AStarPlanner::isValid(const Cell & cell)
{
    return cell.row >= 0 &&
           cell.col >= 0 &&
           cell.row < static_cast<int>(map_->info.height) &&
           cell.col < static_cast<int>(map_->info.width);
}

// =============== A* Algorithm ===============
void AStarPlanner::runAStar()
{    
    RCLCPP_INFO(this->get_logger(), "Running A* algorithm...");

    // 1 Get the start and goal cells
    Cell start_cell = worldToMap(robot_x_, robot_y_);
    Cell goal_cell = worldToMap(goal_x_, goal_y_);

    RCLCPP_INFO(this->get_logger(), "Start cell: (row: %d, col: %d)", start_cell.row, start_cell.col);
    RCLCPP_INFO(this->get_logger(), "Goal cell: (row: %d, col: %d)", goal_cell.row, goal_cell.col);

}

// ── main ────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}

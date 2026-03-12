#include "food_del_astar_planner/astar_planner.hpp"

// =============== Constructor ===============

AStarPlanner::AStarPlanner() : Node("astar_planner_node")
{
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&AStarPlanner::amclPoseCallback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscriber to /amcl_pose created");
}

// =============== Callbacks ===============
void AStarPlanner::amclPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg_)
{
    // Extract the robot pose from the message
    robot_x_   = pose_msg_->pose.pose.position.x;
    robot_y_   = pose_msg_->pose.pose.position.y;
    robot_yaw_ = tf2::getYaw(pose_msg_->pose.pose.orientation);
    robot_pose_received_ = true;

    RCLCPP_INFO(this->get_logger(),
    "Robot pose received — x: %.3f m  y: %.3f m  yaw: %.1f deg",
    robot_x_, robot_y_, robot_yaw_ * (180.0 / M_PI));
}

// ── main ────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}

// By TE Tikea

// just for relaying /cmd_vel (Twist) to /diff_drive_controller/cmd_vel (TwistStamped)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class CmdVelRelayNode : public rclcpp::Node
{
public:
  CmdVelRelayNode() : Node("cmd_vel_to_twist_stamped")
  {
    // Publisher - using QoS suitable for controllers (usually reliable + transient local)
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_controller/cmd_vel", 10);

    // Subscriber
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelRelayNode::cmdVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "cmd_vel → TwistStamped relay started");
    RCLCPP_INFO(this->get_logger(), "  listening: /cmd_vel");
    RCLCPP_INFO(this->get_logger(), " publishing: /diff_drive_controller/cmd_vel");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    auto stamped = geometry_msgs::msg::TwistStamped();

    // Most controllers expect header.stamp to be filled
    stamped.header.stamp = this->now();
    // You can also set frame_id if your controller expects it
    // stamped.header.frame_id = "base_link";   // ← uncomment if needed

    // Only copying the values diff_drive_controller usually uses
    stamped.twist.linear.x  = msg->linear.x;
    stamped.twist.angular.z = msg->angular.z;

    // You can also copy acceleration fields if you want (most controllers ignore them)
    // stamped.twist.linear.z  = msg->linear.z;     // usually 0
    // stamped.twist.angular.x = msg->angular.x;    // usually 0
    // stamped.twist.angular.y = msg->angular.y;    // usually 0

    pub_->publish(stamped);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelRelayNode>());
  rclcpp::shutdown();
  return 0;
}
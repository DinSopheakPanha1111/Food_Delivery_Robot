#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DynamicTeleop : public rclcpp::Node
{
public:
    DynamicTeleop()
    : Node("dynamic_teleop"),
      speed_(0.5),
      angular_speed_(1.0),
      last_triangle_(0),
      last_x_(0)
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&DynamicTeleop::joyCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10);

        RCLCPP_INFO(this->get_logger(), "Dynamic teleop started");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        /*
        PS4 Controller Mapping (Linux / typical driver):
        axes:
          0 = left stick horizontal (LX)
          1 = left stick vertical   (LY)
          2 = right stick horizontal (RX)  <-- FIXED
          5 = right stick vertical   (RY)
          3 = L2
          4 = R2
        buttons:
          0 = X
          2 = Triangle
        */

        // Speed control with buttons
        if (msg->buttons[2] == 1 && last_triangle_ == 0)
        {
            speed_ += 0.1;
            RCLCPP_INFO(this->get_logger(), "Speed: %.2f", speed_);
        }

        if (msg->buttons[0] == 1 && last_x_ == 0)
        {
            speed_ = std::max(0.0, speed_ - 0.1);
            RCLCPP_INFO(this->get_logger(), "Speed: %.2f", speed_);
        }

        last_triangle_ = msg->buttons[2];
        last_x_ = msg->buttons[0];

        geometry_msgs::msg::Twist twist;

        // LEFT STICK → linear velocity (forward/back)
        double ly = msg->axes[1]; // usually inverted
        twist.linear.x = ly * speed_;

        // RIGHT STICK → angular velocity (rotation)
        double rx = msg->axes[2]; // FIXED axis
        twist.angular.z = rx * angular_speed_;

        cmd_pub_->publish(twist);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    double speed_;
    double angular_speed_;

    int last_triangle_;
    int last_x_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
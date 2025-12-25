#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

#include "zlac8015d_driver.hpp"
#include "can.hpp"
#include "kinematic.hpp"

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode()
    : Node("motor_control_node"),
      can_("can0", 500000),
      driver_(can_, 0x601),
      last_time_(this->now())
    {
        // Robot parameters
        wheel_radius_ = 0.075;   // meters
        wheel_base_   = 0.40;    // meters

        // Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/wheel", 10);

        // Subscriber
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorControlNode::cmdVelCallback, this, std::placeholders::_1)
        );

        // Timer (100 Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::publishOdometry, this)
        );

        // CAN + driver
        can_.enable();
        driver_.set_velocity_mode();
        driver_.enable();

        RCLCPP_INFO(this->get_logger(), "Motor Control + Odometry node started (no tf2)");
    }

private:
    /* ================= MEMBERS ================= */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    CAN can_;
    ZLAC8015DDriver driver_;
    KINEMATIC kinematic_;

    float wheel_radius_;
    float wheel_base_;

    float x_{0.0f};
    float y_{0.0f};
    float theta_{0.0f};

    rclcpp::Time last_time_;

    /* ================= CMD_VEL CALLBACK ================= */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float v  = msg->linear.x;     // m/s
        float wz = msg->angular.z;    // rad/s

        // Convert robot velocity -> wheel RPM
        float left_rpm  = kinematic_.get_left_wheel_rpm(v, wz, wheel_base_, wheel_radius_);
        float right_rpm = kinematic_.get_right_wheel_rpm(v, wz, wheel_base_, wheel_radius_);

        // Send to driver
        driver_.set_sync_left_right_speed(left_rpm, right_rpm);
    }

    /* ================= ODOMETRY ================= */
    void publishOdometry()
    {
        float left_rpm = 0.0f;
        float right_rpm = 0.0f;

        if (!driver_.read_speed_feedback(left_rpm, right_rpm))
            return;

        // RPM -> linear wheel velocity (m/s)
        float v_left  = (left_rpm * 0.1f  * 2.0f * M_PI * wheel_radius_) / 60.0f;
        float v_right = (right_rpm * 0.1f * 2.0f * M_PI * wheel_radius_) / 60.0f;

        // Robot velocities
        float vx = kinematic_.get_forward_velocity(v_right, v_left);
        float wz = kinematic_.get_rotational_velocity(v_right, v_left, wheel_base_);

        // Time
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Integrate pose (2D)
        x_     += vx * std::cos(theta_) * dt;
        y_     += vx * std::sin(theta_) * dt;
        theta_ += wz * dt;

        // Yaw → quaternion (NO tf2)
        double half_yaw = theta_ * 0.5;
        double qz = std::sin(half_yaw);
        double qw = std::cos(half_yaw);

        // Fill odometry
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = wz;

        odom_pub_->publish(odom);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}

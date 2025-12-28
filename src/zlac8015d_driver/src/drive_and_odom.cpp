#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

#include "zlac8015d_driver.hpp"
#include "can.hpp"
#include "kinematic.hpp"
#include "food_del_robot/srv/emergency_stop.hpp"

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("motor_control_node"),
      can_("can0", 500000),
      driver_(can_, 0x01),
      last_time_(this->now())
    {
        wheel_radius_ = 0.075;
        wheel_base_   = 0.40;

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/wheel", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorControlNode::cmdVelCallback, this, std::placeholders::_1));

        emergency_stop_srv_ =
            this->create_service<food_del_robot::srv::EmergencyStop>(
                "emergency_stop",
                std::bind(
                    &MotorControlNode::emergencyStopCallback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::publishOdometry, this));

        can_.enable();
        driver_.set_velocity_mode();
        driver_.enable();

        RCLCPP_INFO(this->get_logger(),
            "Motor Control + Odometry + Emergency Stop ready");
    }

private:
    /* ================= ROS ================= */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<food_del_robot::srv::EmergencyStop>::SharedPtr emergency_stop_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ================= DRIVER ================= */
    CAN can_;
    ZLAC8015DDriver driver_;
    KINEMATIC kinematic_;

    /* ================= STATE ================= */
    float wheel_radius_;
    float wheel_base_;
    float x_{0.0f};
    float y_{0.0f};
    float theta_{0.0f};
    bool emergency_active_{false};

    rclcpp::Time last_time_;

    /* ================= CMD_VEL ================= */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (emergency_active_)
            return;   // HARD BLOCK

        float v  = msg->linear.x;
        float wz = msg->angular.z;

        float left_rpm  =
            kinematic_.get_left_wheel_rpm(v, wz, wheel_base_, wheel_radius_);
        float right_rpm =
            kinematic_.get_right_wheel_rpm(v, wz, wheel_base_, wheel_radius_);

        driver_.set_sync_left_right_speed(left_rpm, right_rpm);
    }

    /* ================= EMERGENCY SERVICE ================= */
    void emergencyStopCallback(
        const std::shared_ptr<food_del_robot::srv::EmergencyStop::Request> req,
        std::shared_ptr<food_del_robot::srv::EmergencyStop::Response> res)
    {
        // invalid request
        if (req->stop == req->release)
        {
            res->success = false;
            return;
        }

        bool ok = false;

        if (req->stop)
        {
            driver_.emergency_stop();
            emergency_active_ = driver_.status_emergency_stop();
            ok = emergency_active_;
        }
        else if (req->release)
        {
            driver_.release_emergency_stop();
            emergency_active_ = driver_.status_release_emergency_stop();
            ok = emergency_active_;
        }

        res->success = ok;
    }

    /* ================= ODOMETRY ================= */
    void publishOdometry()
    {
        float left_rpm = 0.0f;
        float right_rpm = 0.0f;

        if (!driver_.read_speed_feedback(left_rpm, right_rpm))
            return;

        float v_left  =
            (left_rpm  * 2.0f * 0.1f * M_PI * wheel_radius_) / 60.0f;
        float v_right =
            (right_rpm * 2.0f * 0.1f * M_PI * wheel_radius_) / 60.0f;
        float vx =
            kinematic_.get_forward_velocity(v_right, v_left);
        float wz =
            kinematic_.get_rotational_velocity(v_right, v_left, wheel_base_);

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        x_ += vx * std::cos(theta_) * dt;
        y_ += vx * std::sin(theta_) * dt;
        theta_ += wz * dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(theta_ * 0.5);
        odom.pose.pose.orientation.w = std::cos(theta_ * 0.5);

        odom.twist.twist.linear.x = vx;
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

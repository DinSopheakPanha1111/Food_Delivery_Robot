#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include "zlac8015d_driver.hpp"
#include "can.hpp"
#include "kinematic.hpp"
#include "food_del_robot/srv/emergency_stop.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("drive_and_odom"),
      can_("can0", 500000),
      driver_(can_, 0x01),
      last_time_(this->now())
    {
        wheel_radius_ = 0.065;
        wheel_base_   = 0.457;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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
            std::chrono::milliseconds(20),
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

    /* ================= TF2 ================= */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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

        driver_.set_sync_left_right_speed(-left_rpm, right_rpm);
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
            kinematic_.get_forward_velocity(v_right, -v_left);
        float wz =
            kinematic_.get_rotational_velocity(v_right, -v_left, wheel_base_);

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        x_ += vx * std::cos(theta_) * dt;
        y_ += vx * std::sin(theta_) * dt;
        theta_ += wz * dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(theta_ * 0.5);
        odom.pose.pose.orientation.w = std::cos(theta_ * 0.5);

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.angular.z = wz;

        // Adding covariance for pose (position and orientation)
        odom.pose.covariance = {1e6, 0.0, 0.0, 0.0, 0.0, 0.0,   
                                0.0, 1e6, 0.0, 0.0, 0.0, 0.0,   
                                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,   
                                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,  
                                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,  
                                0.0, 0.0, 0.0, 0.0, 0.0, 1e6};  

        // Adding covariance for twist (linear and angular velocities)
        odom.twist.covariance = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0,   
                                 0.0, 1e6, 0.0, 0.0, 0.0, 0.0,   
                                 0.0, 0.0, 1e6, 0.0, 0.0, 0.0,   
                                 0.0, 0.0, 0.0, 1e6, 0.0, 0.0,   
                                 0.0, 0.0, 0.0, 0.0, 1e6, 0.0,   
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.5};  

        odom_pub_->publish(odom);

        // Publish tf for left wheel
        geometry_msgs::msg::TransformStamped left_wheel_tf;
        left_wheel_tf.header.stamp = now;
        left_wheel_tf.header.frame_id = "base_link";
        left_wheel_tf.child_frame_id = "left_wheel_link";
        left_wheel_tf.transform.translation.x = 0.0;
        left_wheel_tf.transform.translation.y = -0.225f;
        left_wheel_tf.transform.translation.z = 0.0555f;

        // Apply rotation around the Y-axis (quaternion for rotation around Y)
        float left_angle = left_rpm * dt * 0.5; // Angle in radians
        left_wheel_tf.transform.rotation.x = 0.0; // No rotation around X-axis
        left_wheel_tf.transform.rotation.y = std::sin(left_angle / 2.0); // Rotation around Y-axis
        left_wheel_tf.transform.rotation.z = 0.0; // No rotation around Z-axis
        left_wheel_tf.transform.rotation.w = std::cos(left_angle / 2.0); // Cosine of half the angle

        // Publish tf for right wheel
        geometry_msgs::msg::TransformStamped right_wheel_tf;
        right_wheel_tf.header.stamp = now;
        right_wheel_tf.header.frame_id = "base_link";
        right_wheel_tf.child_frame_id = "right_wheel_link";
        right_wheel_tf.transform.translation.x = 0.0f;
        right_wheel_tf.transform.translation.y = 0.225f;
        right_wheel_tf.transform.translation.z = 0.0555f;

        // Apply rotation around the Y-axis (quaternion for rotation around Y)
        float right_angle = right_rpm * dt * 0.5; // Angle in radians
        right_wheel_tf.transform.rotation.x = 0.0; // No rotation around X-axis
        right_wheel_tf.transform.rotation.y = std::sin(right_angle / 2.0); // Rotation around Y-axis
        right_wheel_tf.transform.rotation.z = 0.0; // No rotation around Z-axis
        right_wheel_tf.transform.rotation.w = std::cos(right_angle / 2.0); // Cosine of half the angle
                // Publish the transforms
        tf_broadcaster_->sendTransform(left_wheel_tf);
        tf_broadcaster_->sendTransform(right_wheel_tf);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
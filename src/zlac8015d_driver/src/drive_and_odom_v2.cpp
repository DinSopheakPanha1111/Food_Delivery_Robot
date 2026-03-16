#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include "zlac8015d_driver.hpp"
#include "can.hpp"
#include "kinematic.hpp"
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
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/wheel", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorControlNode::cmdVelCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&MotorControlNode::imuCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MotorControlNode::publishOdometry, this));

        can_.enable();
        driver_.set_velocity_mode();
        driver_.enable();

        RCLCPP_INFO(this->get_logger(), "Motor Control + Odometry");
    }

private:
    /* ================= ROS ================= */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ================= DRIVER ================= */
    CAN can_;
    ZLAC8015DDriver driver_;
    KINEMATIC kinematic_;

    /* ================= STATE ================= */
    float wheel_radius_ = 0.065;
    float wheel_base_   = 0.457;
    float x_{0.0f};
    float y_{0.0f};
    float theta_{0.0f};
    float cmd_w_right = 0.0f;
    float cmd_w_left  = 0.0f;
    float w_left      = 0.0f;
    float w_right     = 0.0f;
    float v           = 0.0f;
    float omega       = 0.0f;   // now filled by IMU
    float theta_left  = 0.0f;
    float theta_right = 0.0f;

    rclcpp::Time last_time_;

    /* ================= TF2 ================= */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* ================= IMU ================= */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // angular_velocity.z is yaw rate in rad/s
        omega = static_cast<float>(msg->angular_velocity.z);
    }

    /* ================= CMD_VEL ================= */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float v_cmd  = msg->linear.x;
        float wz_cmd = msg->angular.z;

        kinematic_.inverse_kinematic(v_cmd, wz_cmd, wheel_radius_, wheel_base_, &cmd_w_right, &cmd_w_left);

        driver_.set_sync_left_right_speed(-cmd_w_left, cmd_w_right);
    }

    /* ================= ODOMETRY ================= */
    void publishOdometry()
    {
        if (!driver_.read_speed_feedback(w_left, w_right))
            return;

        // Only take linear velocity (v) from wheels; omega comes from IMU
        float v_wheel_omega_unused = 0.0f;
        kinematic_.forward_kinematic(w_right, -w_left, wheel_radius_, wheel_base_, &v, &v_wheel_omega_unused);

        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        x_     += v * std::cos(theta_) * dt;
        y_     += v * std::sin(theta_) * dt;
        theta_ += omega * dt;   // IMU-based omega

        /* ---- Odometry message ---- */
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_footprint";

        odom.pose.pose.position.x    = x_;
        odom.pose.pose.position.y    = y_;
        odom.pose.pose.orientation.z = std::sin(theta_ * 0.5);
        odom.pose.pose.orientation.w = std::cos(theta_ * 0.5);

        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = omega;

        odom.pose.covariance = {1e6, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1e6};

        odom.twist.covariance = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

        odom_pub_->publish(odom);

        /* ---- TF: odom → base_footprint ---- */
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp    = now;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id  = "base_footprint";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;

        odom_tf.transform.rotation.x = 0.0;
        odom_tf.transform.rotation.y = 0.0;
        odom_tf.transform.rotation.z = std::sin(theta_ * 0.5);
        odom_tf.transform.rotation.w = std::cos(theta_ * 0.5);

        tf_broadcaster_->sendTransform(odom_tf);

        /* ---- TF: base_link → left_wheel_link ---- */
        geometry_msgs::msg::TransformStamped right_wheel_tf;
        right_wheel_tf.header.stamp    = now;
        right_wheel_tf.header.frame_id = "base_link";
        right_wheel_tf.child_frame_id  = "right_wheel_link";
        right_wheel_tf.transform.translation.x = 0.0;
        right_wheel_tf.transform.translation.y = -0.225f;
        right_wheel_tf.transform.translation.z = 0.0555f;

        theta_right = w_right * 0.1f * M_PI * dt / 30.0f;
        right_wheel_tf.transform.rotation.x = 0.0;
        right_wheel_tf.transform.rotation.y = std::sin(theta_right / 2.0);
        right_wheel_tf.transform.rotation.z = 0.0;
        right_wheel_tf.transform.rotation.w = std::cos(theta_right / 2.0);

        /* ---- TF: base_link → right_wheel_link ---- */
        geometry_msgs::msg::TransformStamped left_wheel_tf;
        left_wheel_tf.header.stamp    = now;
        left_wheel_tf.header.frame_id = "base_link";
        left_wheel_tf.child_frame_id  = "left_wheel_link";
        left_wheel_tf.transform.translation.x = 0.0f;
        left_wheel_tf.transform.translation.y = 0.225f;
        left_wheel_tf.transform.translation.z = 0.0555f;

        theta_left += (-w_left) * 0.1f * M_PI * dt / 30.0f;
        left_wheel_tf.transform.rotation.x = 0.0;
        left_wheel_tf.transform.rotation.y = std::sin(theta_left / 2.0);
        left_wheel_tf.transform.rotation.z = 0.0;
        left_wheel_tf.transform.rotation.w = std::cos(theta_left / 2.0);

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
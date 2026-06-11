#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <mutex>                        
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
      can_("can1", 500000),
      driver_(can_, 0x01),
      last_time_(this->now())
    {
        // Reentrant group allows timer and IMU callbacks to run in parallel
        cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/wheel", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorControlNode::cmdVelCallback, this, std::placeholders::_1),
            sub_opts);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&MotorControlNode::imuCallback, this, std::placeholders::_1),
            sub_opts);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorControlNode::publishOdometry, this),
            cb_group_);

        can_.enable();
        driver_.set_velocity_mode();
        driver_.enable();

        RCLCPP_INFO(this->get_logger(), "[drive_and_odom] : Motor Control + Odometry node started");
    }

private:
    /* ================= ROS ================= */
    rclcpp::CallbackGroup::SharedPtr                           cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr      odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    /* ================= DRIVER ================= */
    CAN can_;
    ZLAC8015DDriver driver_;
    KINEMATIC kinematic_;

    /* ================= STATE ================= */
    float wheel_radius_ = 0.065f;
    float wheel_base_   = 0.45f;
    float x_      {0.0f};
    float y_      {0.0f};
    float theta_  {0.0f};
    float cmd_w_right {0.0f};
    float cmd_w_left  {0.0f};
    float w_left      {0.0f};
    float w_right     {0.0f};
    float v           {0.0f};
    float omega_      {0.0f};   // written by IMU thread, read by timer thread
    float theta_left  {0.0f};
    float theta_right {0.0f};

    rclcpp::Time last_time_;
    std::mutex omega_mutex_;    // protects omega_

    /* ================= TF2 ================= */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /* ================= IMU callback ================= */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(omega_mutex_);
        omega_ = static_cast<float>(msg->angular_velocity.z);
    }

    /* ================= CMD_VEL callback ================= */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float v_cmd  = msg->linear.x;
        float wz_cmd = msg->angular.z;

        kinematic_.inverse_kinematic(v_cmd, wz_cmd, wheel_radius_, wheel_base_,
                                     &cmd_w_right, &cmd_w_left);

        driver_.set_sync_left_right_speed(-cmd_w_left, cmd_w_right);
    }

    /* ================= Publish odom msg + odom→base_footprint TF ================= */
    void publishOdomMsg(const rclcpp::Time & now)
    {   
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_footprint";

        odom.pose.pose.position.x    = x_;
        odom.pose.pose.position.y    = y_;
        odom.pose.pose.position.z    = 0.0;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = std::sin(theta_ * 0.5f);
        odom.pose.pose.orientation.w = std::cos(theta_ * 0.5f);

        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = omega_;

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
        odom_tf.transform.rotation.x    = 0.0;
        odom_tf.transform.rotation.y    = 0.0;
        odom_tf.transform.rotation.z    = std::sin(theta_ * 0.5f);
        odom_tf.transform.rotation.w    = std::cos(theta_ * 0.5f);

        tf_broadcaster_->sendTransform(odom_tf);
    }

    /* ================= ODOMETRY timer (50 Hz) ================= */
    void publishOdometry()
    {
        rclcpp::Time now = this->now();

        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt <= 0.0 || dt > 0.5)
            return;

        // Snapshot omega_ safely from the IMU thread
        float omega_snapshot;
        {
            std::lock_guard<std::mutex> lock(omega_mutex_);
            omega_snapshot = omega_;
        }

        // Integrate IMU yaw unconditionally and before CAN read
        theta_ += omega_snapshot * static_cast<float>(dt);

        if (!driver_.read_speed_feedback(w_left, w_right))
        {
            v = 0.0f;
            publishOdomMsg(now);
            return;
        }

        float wheel_omega_unused = 0.0f;
        kinematic_.forward_kinematic(w_right, -w_left, wheel_radius_, wheel_base_,
                                     &v, &wheel_omega_unused);

        x_ += v * std::cos(theta_) * static_cast<float>(dt);
        y_ += v * std::sin(theta_) * static_cast<float>(dt);

        publishOdomMsg(now);

        /* ---- TF: base_link → right_wheel_link ---- */
        geometry_msgs::msg::TransformStamped right_wheel_tf;
        right_wheel_tf.header.stamp    = now;
        right_wheel_tf.header.frame_id = "base_link";
        right_wheel_tf.child_frame_id  = "right_wheel_link";
        right_wheel_tf.transform.translation.x =  0.0f;
        right_wheel_tf.transform.translation.y = -0.225f;
        right_wheel_tf.transform.translation.z =  0.0555f;

        theta_right += w_right * 0.1f * static_cast<float>(M_PI) * static_cast<float>(dt) / 30.0f;
        right_wheel_tf.transform.rotation.x = 0.0;
        right_wheel_tf.transform.rotation.y = std::sin(theta_right / 2.0f);
        right_wheel_tf.transform.rotation.z = 0.0;
        right_wheel_tf.transform.rotation.w = std::cos(theta_right / 2.0f);

        /* ---- TF: base_link → left_wheel_link ---- */
        geometry_msgs::msg::TransformStamped left_wheel_tf;
        left_wheel_tf.header.stamp    = now;
        left_wheel_tf.header.frame_id = "base_link";
        left_wheel_tf.child_frame_id  = "left_wheel_link";
        left_wheel_tf.transform.translation.x =  0.0f;
        left_wheel_tf.transform.translation.y =  0.225f;
        left_wheel_tf.transform.translation.z =  0.0555f;

        theta_left += (-w_left) * 0.1f * static_cast<float>(M_PI) * static_cast<float>(dt) / 30.0f;
        left_wheel_tf.transform.rotation.x = 0.0;
        left_wheel_tf.transform.rotation.y = std::sin(theta_left / 2.0f);
        left_wheel_tf.transform.rotation.z = 0.0;
        left_wheel_tf.transform.rotation.w = std::cos(theta_left / 2.0f);

        tf_broadcaster_->sendTransform(right_wheel_tf);
        tf_broadcaster_->sendTransform(left_wheel_tf);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorControlNode>();

    // 3 threads: one for timer (CAN), one for IMU, one spare
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
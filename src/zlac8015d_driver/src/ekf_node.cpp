#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

class EKFNode : public rclcpp::Node
{
public:
    EKFNode() : Node("ekf_node"), initialized_(false)
    {
        // State: [x, y, theta, v, omega]
        // Measurement: [v_enc, omega_imu]
        // omega_enc is kept for initialization only — IMU is trusted for angular rate

        Ts_ = 0.01; // 100 Hz

        // --- State vector ---
        X_ = VectorXd::Zero(5);

        // --- Initial covariance P0 ---
        P_ = MatrixXd::Zero(5, 5);
        P_(0, 0) = 0.04;
        P_(1, 1) = 0.04;
        P_(2, 2) = 0.006;
        P_(3, 3) = 0.0025;
        P_(4, 4) = 0.0008;

        // --- Process noise Q ---
        // Small values = trust kinematic model more for pose
        // Slightly higher for v, omega since they change faster
        Q_ = MatrixXd::Zero(5, 5);
        Q_(0, 0) = 1e-4;  // x
        Q_(1, 1) = 1e-4;  // y
        Q_(2, 2) = 1e-4;  // theta
        Q_(3, 3) = 5e-3;  // v
        Q_(4, 4) = 5e-3;  // omega

        // --- Measurement noise R (2x2) ---
        // Z = [v_enc, omega_imu] — omega_enc dropped to avoid conflict with IMU
        // Lower value = more trust. IMU is 50x more trusted than encoder for omega.
        R_ = MatrixXd::Zero(2, 2);
        R_(0, 0) = 0.05;   // v_enc    — moderate trust
        R_(1, 1) = 0.001;  // omega_imu — high trust

        // --- Jacobian of observation model H (2x5) ---
        // Z = [v_enc, omega_imu] -> state indices [3, 4]
        H_ = MatrixXd::Zero(2, 5);
        H_(0, 3) = 1.0; // v_enc     -> v
        H_(1, 4) = 1.0; // omega_imu -> omega

        // --- Subscribers ---
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/wheel", 10,
            std::bind(&EKFNode::odomCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&EKFNode::imuCallback, this, std::placeholders::_1));

        // --- Publisher ---
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

        // --- Timer at 100 Hz ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&EKFNode::timerCallback, this));

        // Latest measurements
        v_enc_ = 0.0;
        omega_enc_ = 0.0;
        omega_imu_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "EKF Node started at 100 Hz");
    }

private:
    // -------------------------------------------------------------------------
    // Callbacks
    // -------------------------------------------------------------------------
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        v_enc_     = msg->twist.twist.linear.x;
        omega_enc_ = msg->twist.twist.angular.z;

        if (!initialized_) {
            X_(0) = msg->pose.pose.position.x;
            X_(1) = msg->pose.pose.position.y;
            // Extract yaw from quaternion
            tf2::Quaternion q;
            tf2::fromMsg(msg->pose.pose.orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            X_(2) = yaw;
            X_(3) = v_enc_;
            X_(4) = omega_enc_;
            initialized_ = true;
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        omega_imu_ = msg->angular_velocity.z;
    }

    // -------------------------------------------------------------------------
    // EKF Timer: Predict + Update at 100 Hz
    // -------------------------------------------------------------------------
    void timerCallback()
    {
        if (!initialized_) return;

        // ---- PREDICTION STEP ----
        double x     = X_(0);
        double y     = X_(1);
        double theta = X_(2);
        double v     = X_(3);
        double omega = X_(4);

        // f(X_{k-1})
        VectorXd X_pred(5);
        X_pred(0) = x     + v * Ts_ * std::cos(theta);
        X_pred(1) = y     + v * Ts_ * std::sin(theta);
        X_pred(2) = theta + omega * Ts_;
        X_pred(3) = v;
        X_pred(4) = omega;

        // Jacobian F_{k-1} (5x5)
        MatrixXd F = MatrixXd::Identity(5, 5);
        F(0, 2) = -Ts_ * v * std::sin(theta);
        F(0, 3) =  Ts_ * std::cos(theta);
        F(1, 2) =  Ts_ * v * std::cos(theta);
        F(1, 3) =  Ts_ * std::sin(theta);
        F(2, 4) =  Ts_;
        // F(3,3)=1, F(4,4)=1 already set by Identity

        // Predicted covariance: P_hat = F * P * F^T + Q
        MatrixXd P_pred = F * P_ * F.transpose() + Q_;

        // ---- CORRECTION STEP ----
        // Innovation: z - H * X_pred
        // Z = [v_enc, omega_imu] — omega_enc not used (IMU trusted for angular rate)
        VectorXd z(2);
        z(0) = v_enc_;
        z(1) = omega_imu_;

        VectorXd z_hat = H_ * X_pred; // 2x1
        VectorXd innov = z - z_hat;   // 2x1

        // Kalman gain: K = P_hat * H^T * (H * P_hat * H^T + R)^{-1}
        MatrixXd S = H_ * P_pred * H_.transpose() + R_; // 2x2
        MatrixXd K = P_pred * H_.transpose() * S.inverse(); // 5x2

        // Update state
        X_ = X_pred + K * innov;

        // Normalize theta to [-pi, pi]
        X_(2) = std::atan2(std::sin(X_(2)), std::cos(X_(2)));

        // Update covariance: P = (I - KH) * P_hat * (I - KH)^T + K*R*K^T
        MatrixXd I5 = MatrixXd::Identity(5, 5);
        MatrixXd IKH = I5 - K * H_;
        P_ = IKH * P_pred * IKH.transpose() + K * R_ * K.transpose();

        // ---- PUBLISH ----
        publishOdometry();
    }

    // -------------------------------------------------------------------------
    // Publish filtered odometry
    // -------------------------------------------------------------------------
    void publishOdometry()
    {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp    = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id  = "base_link";

        // Position
        msg.pose.pose.position.x = X_(0);
        msg.pose.pose.position.y = X_(1);
        msg.pose.pose.position.z = 0.0;

        // Orientation (yaw -> quaternion)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, X_(2));
        msg.pose.pose.orientation = tf2::toMsg(q);

        // Velocity
        msg.twist.twist.linear.x  = X_(3);
        msg.twist.twist.angular.z = X_(4);

        // Pose covariance (6x6 row-major, fill x,y,yaw from P)
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0]  = P_(0, 0); // xx
        msg.pose.covariance[1]  = P_(0, 1); // xy
        msg.pose.covariance[5]  = P_(0, 2); // x-yaw
        msg.pose.covariance[6]  = P_(1, 0); // yx
        msg.pose.covariance[7]  = P_(1, 1); // yy
        msg.pose.covariance[11] = P_(1, 2); // y-yaw
        msg.pose.covariance[30] = P_(2, 0); // yaw-x
        msg.pose.covariance[31] = P_(2, 1); // yaw-y
        msg.pose.covariance[35] = P_(2, 2); // yaw-yaw

        // Twist covariance (6x6 row-major, fill v,omega from P)
        msg.twist.covariance.fill(0.0);
        msg.twist.covariance[0]  = P_(3, 3); // vv
        msg.twist.covariance[35] = P_(4, 4); // omega-omega

        odom_pub_->publish(msg);
    }

    // -------------------------------------------------------------------------
    // Members
    // -------------------------------------------------------------------------
    double Ts_;
    bool initialized_;

    // EKF matrices
    VectorXd X_;    // State [x, y, theta, v, omega]
    MatrixXd P_;    // Covariance
    MatrixXd Q_;    // Process noise
    MatrixXd R_;    // Measurement noise
    MatrixXd H_;    // Observation Jacobian

    // Latest sensor readings
    double v_enc_;
    double omega_enc_;
    double omega_imu_;

    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// -------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
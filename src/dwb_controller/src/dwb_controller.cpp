#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <cmath>
#include <limits>
#include <vector>

// ─────────────────────────────────────────────
//  Tuneable DWA parameters
// ─────────────────────────────────────────────
struct DWAParams
{
    // Kinematic limits
    double max_v        = 0.5;   // [m/s]   max translational velocity
    double min_v        = 0.0;   // [m/s]   min (no reversing by default)
    double max_w        = 1.0;   // [rad/s] max rotational velocity
    double max_accel_v  = 0.5;   // [m/s²]
    double max_accel_w  = 1.2;   // [rad/s²]

    // Sampling resolution
    int    v_samples    = 10;    // number of v samples in window
    int    w_samples    = 20;    // number of ω samples in window

    // Simulation
    double sim_time     = 1.5;   // [s]  forward-simulate each trajectory
    double sim_dt       = 0.1;   // [s]  integration step

    // Obstacle detection
    double robot_radius = 0.25;  // [m]  used as clearance threshold

    // Objective weights
    double alpha        = 1.2;   // heading weight
    double beta         = 0.3;   // obstacle clearance weight
    double gamma        = 0.2;   // velocity weight

    // Goal tolerance
    double xy_goal_tol  = 0.15;  // [m]
    double yaw_goal_tol = 0.10;  // [rad]

    // Control cycle
    int    control_hz   = 20;    // → 50 ms
};

// ─────────────────────────────────────────────
//  Simple 2-D pose helper
// ─────────────────────────────────────────────
struct Pose2D { double x, y, yaw; };

// ─────────────────────────────────────────────
//  Extract yaw from a ROS2 quaternion message
//  Pure math — no tf2 headers needed
// ─────────────────────────────────────────────
static inline double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

static inline double normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static inline double euclidean(double dx, double dy)
{
    return std::sqrt(dx * dx + dy * dy);
}

// ─────────────────────────────────────────────
//  Node
// ─────────────────────────────────────────────
class DWBControllerNode : public rclcpp::Node
{
public:
    DWBControllerNode()
    : Node("dwb_controller")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/wheel", 10,
            std::bind(&DWBControllerNode::odomCallback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&DWBControllerNode::pathCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&DWBControllerNode::imuCallback, this, std::placeholders::_1));

        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / p_.control_hz),
            std::bind(&DWBControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "DWB Controller Node started");
    }

private:
    // ── ROS handles ─────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr     imu_sub_;
    rclcpp::TimerBase::SharedPtr                               timer_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── state ────────────────────────────────────
    nav_msgs::msg::Odometry   latest_odom_;
    nav_msgs::msg::Path       latest_path_;
    sensor_msgs::msg::Imu     latest_imu_;
    bool odom_received_ = false;
    bool goal_reached_  = false;

    // ── params ───────────────────────────────────
    DWAParams p_;

    // ─────────────────────────────────────────────
    //  Callbacks
    // ─────────────────────────────────────────────
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_   = *msg;
        odom_received_ = true;
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        latest_path_  = *msg;
        goal_reached_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "Received path with %zu poses", msg->poses.size());
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_ = *msg;
    }

    // ─────────────────────────────────────────────
    //  Extract current robot pose from odom
    // ─────────────────────────────────────────────
    Pose2D getRobotPose() const
    {
        Pose2D p;
        p.x   = latest_odom_.pose.pose.position.x;
        p.y   = latest_odom_.pose.pose.position.y;
        p.yaw = quatToYaw(latest_odom_.pose.pose.orientation);
        return p;
    }

    // ─────────────────────────────────────────────
    //  Find lookahead goal on the path
    // ─────────────────────────────────────────────
    Pose2D getLookaheadGoal(const Pose2D & robot) const
    {
        constexpr double lookahead_dist = 0.5; // [m]

        for (const auto & ps : latest_path_.poses) {
            double dx = ps.pose.position.x - robot.x;
            double dy = ps.pose.position.y - robot.y;
            if (euclidean(dx, dy) >= lookahead_dist) {
                Pose2D g;
                g.x   = ps.pose.position.x;
                g.y   = ps.pose.position.y;
                g.yaw = quatToYaw(ps.pose.orientation);
                return g;
            }
        }

        // Fall back to final pose
        const auto & last = latest_path_.poses.back().pose;
        Pose2D g;
        g.x   = last.position.x;
        g.y   = last.position.y;
        g.yaw = quatToYaw(last.orientation);
        return g;
    }

    // ─────────────────────────────────────────────
    //  Forward-simulate a circular arc
    // ─────────────────────────────────────────────
    Pose2D simulateArc(const Pose2D & start, double v, double w) const
    {
        Pose2D pose = start;
        for (double t = 0.0; t < p_.sim_time; t += p_.sim_dt) {
            pose.x   += v * std::cos(pose.yaw) * p_.sim_dt;
            pose.y   += v * std::sin(pose.yaw) * p_.sim_dt;
            pose.yaw  = normalizeAngle(pose.yaw + w * p_.sim_dt);
        }
        return pose;
    }

    // ─────────────────────────────────────────────
    //  Obstacle clearance along the arc
    //  Replace body with a real costmap/laser query.
    // ─────────────────────────────────────────────
    double arcClearance(const Pose2D & /*start*/,
                        double /*v*/, double /*w*/) const
    {
        return p_.sim_time;   // placeholder → always "clear"
    }

    // ─────────────────────────────────────────────
    //  Scoring helpers
    // ─────────────────────────────────────────────
    double headingScore(const Pose2D & end, const Pose2D & goal) const
    {
        double angle_to_goal = std::atan2(goal.y - end.y, goal.x - end.x);
        double diff = std::fabs(normalizeAngle(angle_to_goal - end.yaw));
        return 180.0 - (diff * 180.0 / M_PI);
    }

    double velocityScore(double v) const
    {
        return (p_.max_v > 0.0) ? (v / p_.max_v) : 0.0;
    }

    // ─────────────────────────────────────────────
    //  DWA core
    // ─────────────────────────────────────────────
    std::pair<double, double> computeDWA(const Pose2D & robot,
                                          const Pose2D & goal,
                                          double cur_v,
                                          double cur_w)
    {
        const double dt = 1.0 / p_.control_hz;

        // Dynamic window bounds
        double v_lo = std::max(p_.min_v, cur_v - p_.max_accel_v * dt);
        double v_hi = std::min(p_.max_v, cur_v + p_.max_accel_v * dt);
        double w_lo = std::max(-p_.max_w, cur_w - p_.max_accel_w * dt);
        double w_hi = std::min( p_.max_w, cur_w + p_.max_accel_w * dt);

        double best_score = -std::numeric_limits<double>::infinity();
        double best_v = 0.0, best_w = 0.0;

        for (int iv = 0; iv <= p_.v_samples; ++iv)
        {
            double v = v_lo + iv * (v_hi - v_lo) / p_.v_samples;

            for (int iw = 0; iw <= p_.w_samples; ++iw)
            {
                double w = w_lo + iw * (w_hi - w_lo) / p_.w_samples;

                // Admissibility: robot must be able to stop before obstacle
                double stop_dist = (p_.max_accel_v > 0.0)
                                   ? (v * v) / (2.0 * p_.max_accel_v)
                                   : 0.0;
                double clearance = arcClearance(robot, v, w);
                if (clearance < stop_dist + p_.robot_radius)
                    continue;

                // Simulate & score
                Pose2D end   = simulateArc(robot, v, w);
                double score = p_.alpha * headingScore(end, goal)
                             + p_.beta  * clearance
                             + p_.gamma * velocityScore(v);

                if (score > best_score) {
                    best_score = score;
                    best_v = v;
                    best_w = w;
                }
            }
        }
        return {best_v, best_w};
    }

    // ─────────────────────────────────────────────
    //  Spin in place to align with goal yaw
    // ─────────────────────────────────────────────
    std::pair<double, double> rotateToYaw(double current_yaw,
                                           double target_yaw) const
    {
        double err = normalizeAngle(target_yaw - current_yaw);
        double w   = std::copysign(
                         std::min(p_.max_w, std::fabs(err) * 2.0), err);
        return {0.0, w};
    }

    // ─────────────────────────────────────────────
    //  Main control loop (20 Hz)
    // ─────────────────────────────────────────────
    void controlLoop()
    {
        if (!odom_received_ || latest_path_.poses.empty())
            return;

        geometry_msgs::msg::Twist cmd;

        Pose2D robot = getRobotPose();
        double cur_v = latest_odom_.twist.twist.linear.x;
        double cur_w = latest_odom_.twist.twist.angular.z;

        // Distance to final goal
        const auto & final_pose = latest_path_.poses.back().pose;
        double dx_f             = final_pose.position.x - robot.x;
        double dy_f             = final_pose.position.y - robot.y;
        double dist_to_goal     = euclidean(dx_f, dy_f);

        // Goal reached?
        if (dist_to_goal < p_.xy_goal_tol)
        {
            double goal_yaw  = quatToYaw(final_pose.orientation);
            double yaw_error = std::fabs(normalizeAngle(goal_yaw - robot.yaw));

            if (yaw_error < p_.yaw_goal_tol) {
                if (!goal_reached_) {
                    RCLCPP_INFO(this->get_logger(), "Goal reached!");
                    goal_reached_ = true;
                }
                cmd_vel_pub_->publish(cmd);   // zero velocity
                return;
            }

            auto [v, w] = rotateToYaw(robot.yaw, goal_yaw);
            cmd.linear.x  = v;
            cmd.angular.z = w;
            cmd_vel_pub_->publish(cmd);
            return;
        }
        goal_reached_ = false;

        // DWA toward lookahead goal
        Pose2D goal = getLookaheadGoal(robot);
        auto [v, w] = computeDWA(robot, goal, cur_v, cur_w);

        cmd.linear.x  = v;
        cmd.angular.z = w;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_DEBUG(this->get_logger(),
                     "DWA cmd: v=%.3f  w=%.3f  dist=%.3f",
                     v, w, dist_to_goal);
    }
};

// ─────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWBControllerNode>());
    rclcpp::shutdown();
    return 0;
}
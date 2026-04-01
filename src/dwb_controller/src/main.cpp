#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include "dwb_controller/obstacle_detector.hpp"
#include "dwb_controller/dwb_controller.hpp"

#include <cmath>
#include <vector>
#include <mutex>

constexpr uint32_t MAX_OBSTACLES = 64;

class DWBControllerNode : public rclcpp::Node
{
public:
    DWBControllerNode()
    : Node("dwb_controller")
    {
        // ── Owned local_costmap ────────────────────────────────────────────
        local_costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "local_costmap",
            std::string{get_namespace()},
            "local_costmap"
        );

        // Configure immediately — safe before TF
        local_costmap_->configure();

        // Activate after 3 seconds to ensure TF (odom→base_link) is ready
        activate_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Activating local_costmap...");
                local_costmap_->activate();
                activate_timer_->cancel();
                costmap_ready_ = true;
                RCLCPP_INFO(this->get_logger(), "local_costmap activated.");
            });

        // ── Callback groups ────────────────────────────────────────────────
        cb_group_costmap_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_control_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions costmap_opts;
        costmap_opts.callback_group = cb_group_costmap_;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&DWBControllerNode::odom_callback, this,
                      std::placeholders::_1),
            costmap_opts);

        rclcpp::SubscriptionOptions control_opts;
        control_opts.callback_group = cb_group_control_;

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&DWBControllerNode::path_callback, this,
                      std::placeholders::_1),
            control_opts);

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DWBControllerNode::control_loop, this),
            cb_group_control_);

        RCLCPP_INFO(this->get_logger(),
            "DWB Controller Node started — local_costmap owned internally. "
            "Waiting for /plan");
    }

    ~DWBControllerNode()
    {
        if (local_costmap_) {
            local_costmap_->deactivate();
            local_costmap_->cleanup();
        }
    }

    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> get_local_costmap()
    {
        return local_costmap_;
    }

private:
    // ── Owned costmap ───────────────────────────────────────────────────────
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_;
    bool costmap_ready_ = false;

    // ── Timers ──────────────────────────────────────────────────────────────
    rclcpp::TimerBase::SharedPtr activate_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── Callback groups ─────────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr cb_group_costmap_;
    rclcpp::CallbackGroup::SharedPtr cb_group_control_;

    // ── Robot state ─────────────────────────────────────────────────────────
    float robot_x_     = 0.0f;
    float robot_y_     = 0.0f;
    float robot_theta_ = 0.0f;
    float robot_v_     = 0.0f;
    float robot_w_     = 0.0f;
    bool  has_odom_    = false;

    // ── Path ────────────────────────────────────────────────────────────────
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_wp_idx_ = 0;
    bool   has_path_       = false;

    // ── Obstacles ───────────────────────────────────────────────────────────
    std::mutex obs_mutex_;
    float    obs_x_  [MAX_OBSTACLES];
    float    obs_y_  [MAX_OBSTACLES];
    float    obs_rad_[MAX_OBSTACLES];
    uint32_t num_obstacles_ = 0;

    // ── ROS handles ─────────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;

    // =========================================================================
    //  Read costmap directly from owned Costmap2DROS
    // =========================================================================

    void update_obstacles_from_costmap()
    {
        if (!has_odom_ || !costmap_ready_) return;

        auto * costmap2d = local_costmap_->getCostmap();
        if (!costmap2d) return;

        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
            *(costmap2d->getMutex()));

        const uint32_t width      = costmap2d->getSizeInCellsX();
        const uint32_t height     = costmap2d->getSizeInCellsY();
        const float    resolution = costmap2d->getResolution();
        const float    origin_x   = costmap2d->getOriginX();
        const float    origin_y   = costmap2d->getOriginY();

        if (width == 0 || height == 0) return;

        std::vector<int8_t> costmap_data(width * height);
        for (uint32_t i = 0; i < width * height; i++) {
            uint8_t raw = costmap2d->getCharMap()[i];
            costmap_data[i] = (raw == nav2_costmap_2d::LETHAL_OBSTACLE) ? 100
                            : (raw == nav2_costmap_2d::NO_INFORMATION)  ? -1
                            : static_cast<int8_t>(raw * 100 / 253);
        }

        lock.unlock();

        float c_robot_x, c_robot_y;
        world_to_cell_conversion(robot_x_, robot_y_,
                                 origin_x, origin_y, resolution,
                                 &c_robot_x, &c_robot_y);

        constexpr float R_metres = 2.5f;
        uint32_t R_cells = static_cast<uint32_t>(floorf(R_metres / resolution));

        uint32_t max_cells = (2 * R_cells + 1) * (2 * R_cells + 1);
        std::vector<float> world_out(max_cells * 2);
        uint32_t lethal_count = 0;

        collect_lethal_cells(costmap_data, width, height,
                             c_robot_x, c_robot_y,
                             origin_x, origin_y, resolution,
                             R_cells, R_metres,
                             world_out.data(), &lethal_count);

        if (lethal_count == 0) {
            std::lock_guard<std::mutex> lk(obs_mutex_);
            num_obstacles_ = 0;
            return;
        }

        std::vector<int> labels(lethal_count);
        int num_clusters = 0;
        cluster_lethal_points(world_out.data(), lethal_count,
                              resolution, labels.data(), &num_clusters);

        float    tmp_x  [MAX_OBSTACLES];
        float    tmp_y  [MAX_OBSTACLES];
        float    tmp_rad[MAX_OBSTACLES];
        uint32_t tmp_count = 0;

        for (int k = 0; k < num_clusters; k++) {
            if (tmp_count >= MAX_OBSTACLES) break;

            std::vector<float> cluster_pts;
            for (uint32_t i = 0; i < lethal_count; i++) {
                if (labels[i] == k) {
                    cluster_pts.push_back(world_out[i * 2]);
                    cluster_pts.push_back(world_out[i * 2 + 1]);
                }
            }
            uint32_t cluster_count = cluster_pts.size() / 2;

            float w_cx = 0.0f, w_cy = 0.0f, obs_radius = 0.0f;
            get_obstacle_center_and_radius(cluster_pts.data(), cluster_count,
                                           resolution,
                                           &w_cx, &w_cy, &obs_radius);

            tmp_x  [tmp_count] = w_cx;
            tmp_y  [tmp_count] = w_cy;
            tmp_rad[tmp_count] = obs_radius + 0.15f;
            tmp_count++;
        }

        {
            std::lock_guard<std::mutex> lk(obs_mutex_);
            memcpy(obs_x_,   tmp_x,   tmp_count * sizeof(float));
            memcpy(obs_y_,   tmp_y,   tmp_count * sizeof(float));
            memcpy(obs_rad_, tmp_rad, tmp_count * sizeof(float));
            num_obstacles_ = tmp_count;
        }
    }

    // =========================================================================
    //  Callbacks
    // =========================================================================

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        const auto & q = msg->pose.pose.orientation;
        robot_theta_ = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                              1.0f - 2.0f * (q.y * q.y + q.z * q.z));
        robot_v_  = msg->twist.twist.linear.x;
        robot_w_  = msg->twist.twist.angular.z;
        has_odom_ = true;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_           = msg->poses;
        current_wp_idx_ = 0;
        has_path_       = !path_.empty();

        if (has_path_) {
            RCLCPP_INFO(this->get_logger(),
                "New path received: %zu waypoints. First wp: (%.2f, %.2f)",
                path_.size(),
                path_.front().pose.position.x,
                path_.front().pose.position.y);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received empty path — stopping.");
        }
    }

    // =========================================================================
    //  DWB control loop (50 Hz)
    // =========================================================================

    void control_loop()
    {
        if (!has_odom_ || !has_path_ || !costmap_ready_) return;

        update_obstacles_from_costmap();

        constexpr float WP_TOLERANCE = 0.35f;

        while (current_wp_idx_ < path_.size()) {
            const float wx = static_cast<float>(path_[current_wp_idx_].pose.position.x);
            const float wy = static_cast<float>(path_[current_wp_idx_].pose.position.y);
            const float dx = robot_x_ - wx;
            const float dy = robot_y_ - wy;
            if (sqrtf(dx * dx + dy * dy) > WP_TOLERANCE) break;
            current_wp_idx_++;
        }

        if (current_wp_idx_ >= path_.size()) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
            RCLCPP_INFO_ONCE(this->get_logger(), "Goal reached — path complete.");
            has_path_ = false;
            return;
        }

        float    snap_x  [MAX_OBSTACLES];
        float    snap_y  [MAX_OBSTACLES];
        float    snap_rad[MAX_OBSTACLES];
        uint32_t snap_count = 0;
        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            snap_count = num_obstacles_;
            memcpy(snap_x,   obs_x_,   snap_count * sizeof(float));
            memcpy(snap_y,   obs_y_,   snap_count * sizeof(float));
            memcpy(snap_rad, obs_rad_, snap_count * sizeof(float));
        }

        size_t target_wp   = current_wp_idx_;
        bool   found_clear = false;

        for (size_t look = current_wp_idx_;
             look < std::min(path_.size(), current_wp_idx_ + 10);
             look++)
        {
            const float wx = static_cast<float>(path_[look].pose.position.x);
            const float wy = static_cast<float>(path_[look].pose.position.y);
            bool blocked = false;
            for (uint32_t o = 0; o < snap_count; o++) {
                const float dx = wx - snap_x[o];
                const float dy = wy - snap_y[o];
                if (sqrtf(dx * dx + dy * dy) < snap_rad[o] + 0.1f) {
                    blocked = true;
                    break;
                }
            }
            if (!blocked) {
                target_wp   = look;
                found_clear = true;
                break;
            }
        }

        const float goal_x = static_cast<float>(path_[target_wp].pose.position.x);
        const float goal_y = static_cast<float>(path_[target_wp].pose.position.y);

        if (!found_clear) {
            size_t last_look = std::min(path_.size(), current_wp_idx_ + 10) - 1;
            const float tx = static_cast<float>(path_[last_look].pose.position.x);
            const float ty = static_cast<float>(path_[last_look].pose.position.y);

            float angle_to_target = atan2f(ty - robot_y_, tx - robot_x_);
            float heading_error   = angle_to_target - robot_theta_;
            while (heading_error >  M_PI) heading_error -= 2.0f * M_PI;
            while (heading_error < -M_PI) heading_error += 2.0f * M_PI;

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = 0.0f;
            cmd.angular.z = (heading_error > 0.0f) ? 0.4f : -0.4f;
            cmd_vel_pub_->publish(cmd);

            RCLCPP_WARN(this->get_logger(),
                "All look-ahead waypoints blocked — turning in place. heading_err=%.2f",
                heading_error);
            return;
        }

        float best_v = 0.0f, best_w = 0.0f;

        rollout_best_control(
            {robot_x_, robot_y_, robot_theta_, robot_v_, robot_w_},
            goal_x, goal_y,
            snap_x, snap_y, snap_rad, snap_count,
            3.0f,   // k_g
            5.0f,   // k_o
            0.3f,   // k_v
            0.05f,  // v_min
            0.3f,   // v_max
            0.5f,   // w_max
            2.0f,   // a_v
            2.0f,   // a_w
            0.1f,   // dt
            2.0f,   // window_time
            0.02f,  // vel_res
            0.05f,  // ang_res
            0.22f,  // r_robot
            0.5f,   // clearance_thresh
            &best_v, &best_w
        );

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = best_v;
        cmd.angular.z = best_w;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(),
            "wp %zu/%zu  target=%zu  goal=(%.2f,%.2f)  cmd: v=%.3f w=%.3f  obs=%u",
            current_wp_idx_ + 1, path_.size(), target_wp + 1,
            goal_x, goal_y, best_v, best_w, snap_count);
    }
};

// =============================================================================
//  main
// =============================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DWBControllerNode>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 2);

    executor.add_node(node);
    executor.add_node(node->get_local_costmap()->get_node_base_interface());

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
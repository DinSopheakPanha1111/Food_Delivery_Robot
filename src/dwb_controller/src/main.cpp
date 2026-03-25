#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
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
        // ── Callback groups ───────────────────────────────────────────────────
        // cb_group_costmap_  : costmap + odom callbacks (mutually exclusive)
        // cb_group_control_  : path callback + control timer (mutually exclusive)
        //   → path_ and current_wp_idx_ are always accessed from the same group,
        //     so NO mutex is needed for those two members.
        cb_group_costmap_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        cb_group_control_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // ── Costmap / odom subscriptions ──────────────────────────────────────
        rclcpp::SubscriptionOptions costmap_opts;
        costmap_opts.callback_group = cb_group_costmap_;

        local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap/costmap", 10,
            std::bind(&DWBControllerNode::local_costmap_callback, this,
                      std::placeholders::_1),
            costmap_opts);

        local_costmap_update_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            "/costmap/costmap_updates", 10,
            std::bind(&DWBControllerNode::local_costmap_update_callback, this,
                      std::placeholders::_1),
            costmap_opts);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&DWBControllerNode::odom_callback, this,
                      std::placeholders::_1),
            costmap_opts);

        // ── Path subscription (same group as control timer) ───────────────────
        rclcpp::SubscriptionOptions control_opts;
        control_opts.callback_group = cb_group_control_;

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&DWBControllerNode::path_callback, this,
                      std::placeholders::_1),
            control_opts);

        // ── Publisher & control timer ──────────────────────────────────────────
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&DWBControllerNode::control_loop, this),
            cb_group_control_);

        RCLCPP_INFO(this->get_logger(), "DWB Controller Node started — waiting for /planned_path");
    }

private:
    // ── Callback groups ───────────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr cb_group_costmap_;
    rclcpp::CallbackGroup::SharedPtr cb_group_control_;

    // ── Costmap state ─────────────────────────────────────────────────────────
    uint32_t            width_      = 0;
    uint32_t            height_     = 0;
    float               resolution_ = 0.0f;
    float               origin_x_   = 0.0f;
    float               origin_y_   = 0.0f;
    std::vector<int8_t> costmap_;

    // ── Odometry state ────────────────────────────────────────────────────────
    float robot_x_     = 0.0f;
    float robot_y_     = 0.0f;
    float robot_theta_ = 0.0f;
    float robot_v_     = 0.0f;
    float robot_w_     = 0.0f;
    bool  has_odom_    = false;

    // ── Path following state ──────────────────────────────────────────────────
    // Accessed only from cb_group_control_ → no mutex needed.
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_wp_idx_ = 0;
    bool   has_path_       = false;

    // ── Obstacle store ────────────────────────────────────────────────────────
    // Written by cb_group_costmap_, read by cb_group_control_ → mutex required.
    std::mutex obs_mutex_;
    float    obs_x_  [MAX_OBSTACLES];
    float    obs_y_  [MAX_OBSTACLES];
    float    obs_rad_[MAX_OBSTACLES];
    uint32_t num_obstacles_ = 0;

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr       local_costmap_sub_;
    rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr local_costmap_update_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr                                        control_timer_;

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

    void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        width_      = msg->info.width;
        height_     = msg->info.height;
        resolution_ = msg->info.resolution;
        origin_x_   = msg->info.origin.position.x;
        origin_y_   = msg->info.origin.position.y;
        costmap_    = msg->data;
        calculate_obstacle_cost();
    }

    void local_costmap_update_callback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg)
    {
        if (costmap_.empty()) return;
        for (uint32_t row = 0; row < msg->height; row++) {
            for (uint32_t col = 0; col < msg->width; col++) {
                int full_index  = (msg->y + row) * width_ + (msg->x + col);
                int patch_index = row * msg->width + col;
                costmap_[full_index] = msg->data[patch_index];
            }
        }
        calculate_obstacle_cost();
    }

    // =========================================================================
    //  Obstacle detection pipeline
    // =========================================================================

    void calculate_obstacle_cost()
    {
        if (!has_odom_ || costmap_.empty()) return;

        const float r   = resolution_;
        const float o_x = origin_x_;
        const float o_y = origin_y_;

        // 1. robot world → cell
        float c_robot_x, c_robot_y;
        world_to_cell_conversion(robot_x_, robot_y_, o_x, o_y, r,
                                 &c_robot_x, &c_robot_y);

        // 2. search radius: v_max * window_time + clearance_thresh = 2.5 m
        constexpr float R_metres = 2.5f;
        uint32_t R_cells = static_cast<uint32_t>(floorf(R_metres / r));

        // 3. collect lethal cells
        uint32_t max_cells = (2 * R_cells + 1) * (2 * R_cells + 1);
        float    world_out[max_cells * 2];
        uint32_t lethal_count = 0;

        collect_lethal_cells(costmap_, width_, height_,
                             c_robot_x, c_robot_y,
                             o_x, o_y, r,
                             R_cells, R_metres,
                             world_out, &lethal_count);

        if (lethal_count == 0) {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            num_obstacles_ = 0;
            return;
        }

        // 4. cluster
        int labels[lethal_count];
        int num_clusters = 0;
        cluster_lethal_points(world_out, lethal_count, r, labels, &num_clusters);

        // 5. build obstacles — heavy work outside mutex
        float    tmp_x  [MAX_OBSTACLES];
        float    tmp_y  [MAX_OBSTACLES];
        float    tmp_rad[MAX_OBSTACLES];
        uint32_t tmp_count = 0;

        for (int k = 0; k < num_clusters; k++) {
            if (tmp_count >= MAX_OBSTACLES) break;

            float    cluster_pts[lethal_count * 2];
            uint32_t cluster_count = 0;

            for (uint32_t i = 0; i < lethal_count; i++) {
                if (labels[i] == k) {
                    cluster_pts[cluster_count * 2]     = world_out[i * 2];
                    cluster_pts[cluster_count * 2 + 1] = world_out[i * 2 + 1];
                    cluster_count++;
                }
            }

            float w_cx = 0.0f, w_cy = 0.0f, obs_radius = 0.0f;
            get_obstacle_center_and_radius(cluster_pts, cluster_count, r,
                                           &w_cx, &w_cy, &obs_radius);

            tmp_x  [tmp_count] = w_cx;
            tmp_y  [tmp_count] = w_cy;
            tmp_rad[tmp_count] = obs_radius + 0.1f;   // small inflation margin
            tmp_count++;
        }

        // 6. swap into shared arrays under lock — fast memcpy only
        {
            std::lock_guard<std::mutex> lock(obs_mutex_);
            memcpy(obs_x_,   tmp_x,   tmp_count * sizeof(float));
            memcpy(obs_y_,   tmp_y,   tmp_count * sizeof(float));
            memcpy(obs_rad_, tmp_rad, tmp_count * sizeof(float));
            num_obstacles_ = tmp_count;
        }

        RCLCPP_DEBUG(this->get_logger(),
            "Lethal: %u  Clusters: %d  R_metres: %.2f  R_cells: %u",
            lethal_count, num_clusters, R_metres, R_cells);
    }

    // =========================================================================
    //  DWB control loop (50 Hz)
    // =========================================================================

    void control_loop()
    {
        if (!has_odom_ || !has_path_) return;

        constexpr float WP_TOLERANCE    = 0.35f;
        constexpr float HEADING_THRESH  = M_PI / 4.0f;  // 45° — turn in place beyond this

        while (current_wp_idx_ < path_.size()) {
            const float goal_x = static_cast<float>(path_[current_wp_idx_].pose.position.x);
            const float goal_y = static_cast<float>(path_[current_wp_idx_].pose.position.y);
            const float dx = robot_x_ - goal_x;
            const float dy = robot_y_ - goal_y;
            if (sqrtf(dx * dx + dy * dy) > WP_TOLERANCE) break;
            current_wp_idx_++;
        }

        if (current_wp_idx_ >= path_.size()) {
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
            RCLCPP_INFO_ONCE(this->get_logger(), "Goal reached — path complete.");
            return;
        }

        const float goal_x = static_cast<float>(path_[current_wp_idx_].pose.position.x);
        const float goal_y = static_cast<float>(path_[current_wp_idx_].pose.position.y);

        // ── Heading check — turn in place if goal is too far off-axis ────────────
        const float angle_to_goal = atan2f(goal_y - robot_y_, goal_x - robot_x_);
        float heading_error = angle_to_goal - robot_theta_;
        // Normalise to [-π, π]
        while (heading_error >  M_PI) heading_error -= 2.0f * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0f * M_PI;

        if (fabsf(heading_error) > HEADING_THRESH) {
            // Turn in place — no forward motion
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = 0.0f;
            cmd.angular.z = (heading_error > 0.0f) ? 0.5f : -0.5f;
            cmd_vel_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(),
                "Turning in place: heading_error=%.2f rad", heading_error);
            return;
        }

        // ── Snapshot obstacles ────────────────────────────────────────────────────
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

        // ── DWB rollout ───────────────────────────────────────────────────────────
        float best_v = 0.0f, best_w = 0.0f;

        rollout_best_control(
            {robot_x_, robot_y_, robot_theta_, robot_v_, robot_w_},
            goal_x, goal_y,
            snap_x, snap_y, snap_rad, snap_count,
            1.0f,   // k_g
            5.0f,   // k_o
            0.1f,   // k_v
            0.05f,  // v_min
            0.1f,   // v_max
            0.5f,   // w_max
            1.0f,   // a_v
            1.0f,   // a_w
            0.1f,   // dt
            3.0f,   // window_time
            0.02f,  // vel_res
            0.05f,  // ang_res
            0.22f,  // r_robot
            2.0f,   // clearance_thresh
            &best_v, &best_w
        );

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = best_v;
        cmd.angular.z = best_w;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO(this->get_logger(),
            "wp %zu/%zu  goal=(%.2f,%.2f)  heading_err=%.2f  cmd: v=%.3f w=%.3f  obs=%u",
            current_wp_idx_ + 1, path_.size(),
            goal_x, goal_y, heading_error,
            best_v, best_w, snap_count);
    }
};

// =============================================================================
//  main — MultiThreadedExecutor with 2 threads
// =============================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DWBControllerNode>();

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions{}, 2);

    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <vector>
#include <random>
#include <array>

namespace my_amcl
{

// ─────────────────────────────────────────────
//  A single particle: pose + weight
// ─────────────────────────────────────────────
struct Particle
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double weight{1.0};
};

// ─────────────────────────────────────────────
//  Likelihood field: precomputed distance-to-
//  nearest-obstacle grid for fast scoring
// ─────────────────────────────────────────────
class LikelihoodField
{
public:
  void build(const nav_msgs::msg::OccupancyGrid & map);

  // Returns the probability of observing a hit at (world_x, world_y)
  double likelihood(double world_x, double world_y) const;

  bool ready() const { return ready_; }

private:
  nav_msgs::msg::OccupancyGrid map_;
  std::vector<float> dist_field_;   // distance (metres) to nearest obstacle
  bool ready_{false};

  // Convert world coords → grid cell
  bool worldToGrid(double wx, double wy, int & cx, int & cy) const;
  int cellIndex(int cx, int cy) const { return cy * map_.info.width + cx; }
};

// ─────────────────────────────────────────────
//  The AMCL node
// ─────────────────────────────────────────────
class MyAmcl : public rclcpp::Node
{
public:
  explicit MyAmcl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ──────────────────────────────
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // ── Particle filter steps ──────────────────
  void initParticles();
  void motionUpdate(double dx, double dy, double dtheta);
  void sensorUpdate(const sensor_msgs::msg::LaserScan & scan);
  void resample();

  // ── Output ─────────────────────────────────
  void publishPose();
  void publishParticles();
  void broadcastMapToOdom();

  // ── Helpers ────────────────────────────────
  static double normalizeAngle(double a);
  double sampleGaussian(double sigma);

  // ── Subscribers / Publishers ───────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── State ──────────────────────────────────
  std::vector<Particle> particles_;
  LikelihoodField likelihood_field_;

  // Last odometry pose (to compute deltas)
  double last_odom_x_{0.0};
  double last_odom_y_{0.0};
  double last_odom_theta_{0.0};
  bool odom_initialized_{false};
  bool map_received_{false};
  bool initialized_{false};

  // Estimated robot pose in map frame
  double est_x_{0.0};
  double est_y_{0.0};
  double est_theta_{0.0};

  // ── Parameters ─────────────────────────────
  int    num_particles_;
  double sigma_hit_;          // sensor model: std dev of Gaussian hit
  double z_hit_;              // sensor model: weight of Gaussian hit
  double z_rand_;             // sensor model: weight of random noise
  double laser_max_range_;    // max usable laser range
  int    laser_subsample_;    // use every Nth beam (speed vs accuracy)
  double alpha1_, alpha2_;    // motion noise: rotation
  double alpha3_, alpha4_;    // motion noise: translation

  // ── RNG ────────────────────────────────────
  std::mt19937 rng_;
};

}  // namespace my_amcl

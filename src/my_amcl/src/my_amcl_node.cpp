#include "my_amcl/my_amcl.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <algorithm>
#include <numeric>
#include <queue>

namespace my_amcl
{

// ═══════════════════════════════════════════════════════════════
//  LikelihoodField
// ═══════════════════════════════════════════════════════════════

void LikelihoodField::build(const nav_msgs::msg::OccupancyGrid & map)
{
  map_ = map;
  const int W = map.info.width;
  const int H = map.info.height;
  const float res = map.info.resolution;
  const int N = W * H;

  // BFS from every obstacle cell to build distance transform
  dist_field_.assign(N, std::numeric_limits<float>::max());

  // BFS queue: (cell_index)
  std::queue<int> q;

  for (int i = 0; i < N; ++i) {
    // nav_msgs occupancy: 0=free, 100=occupied, -1=unknown
    if (map.data[i] > 50) {
      dist_field_[i] = 0.0f;
      q.push(i);
    }
  }

  // 4-connected BFS
  const int dx[] = {1, -1, 0, 0};
  const int dy[] = {0, 0, 1, -1};

  while (!q.empty()) {
    int idx = q.front(); q.pop();
    int cx = idx % W;
    int cy = idx / W;
    float d = dist_field_[idx];

    for (int k = 0; k < 4; ++k) {
      int nx = cx + dx[k];
      int ny = cy + dy[k];
      if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
      int nidx = ny * W + nx;
      float nd = d + res;
      if (nd < dist_field_[nidx]) {
        dist_field_[nidx] = nd;
        q.push(nidx);
      }
    }
  }

  ready_ = true;
}

bool LikelihoodField::worldToGrid(double wx, double wy, int & cx, int & cy) const
{
  const auto & info = map_.info;
  cx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
  cy = static_cast<int>((wy - info.origin.position.y) / info.resolution);
  return cx >= 0 && cx < static_cast<int>(info.width) &&
         cy >= 0 && cy < static_cast<int>(info.height);
}

double LikelihoodField::likelihood(double world_x, double world_y) const
{
  if (!ready_) return 0.0;
  int cx, cy;
  if (!worldToGrid(world_x, world_y, cx, cy)) {
    return 0.0;  // out of map → treat as max range / no info
  }
  return dist_field_[cellIndex(cx, cy)];
}

// ═══════════════════════════════════════════════════════════════
//  MyAmcl  — constructor
// ═══════════════════════════════════════════════════════════════

MyAmcl::MyAmcl(const rclcpp::NodeOptions & options)
: Node("my_amcl", options),
  rng_(std::random_device{}())
{
  // ── Declare & get parameters ──────────────
  num_particles_   = declare_parameter("num_particles",   500);
  sigma_hit_       = declare_parameter("sigma_hit",       0.2);
  z_hit_           = declare_parameter("z_hit",           0.95);
  z_rand_          = declare_parameter("z_rand",          0.05);
  laser_max_range_ = declare_parameter("laser_max_range", 12.0);
  laser_subsample_ = declare_parameter("laser_subsample", 5);   // use every 5th beam
  alpha1_          = declare_parameter("alpha1",          0.2); // rot noise from rot
  alpha2_          = declare_parameter("alpha2",          0.2); // rot noise from trans
  alpha3_          = declare_parameter("alpha3",          0.2); // trans noise from trans
  alpha4_          = declare_parameter("alpha4",          0.2); // trans noise from rot

  // ── Subscribers ───────────────────────────
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local(),
    std::bind(&MyAmcl::mapCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&MyAmcl::odomCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&MyAmcl::scanCallback, this, std::placeholders::_1));

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10,
    std::bind(&MyAmcl::initialPoseCallback, this, std::placeholders::_1));

  // ── Publishers ────────────────────────────
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10);

  particle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
    "/particle_cloud", 10);

  // ── TF ────────────────────────────────────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_      = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "my_amcl started. Waiting for map...");
}

// ═══════════════════════════════════════════════════════════════
//  Callbacks
// ═══════════════════════════════════════════════════════════════

void MyAmcl::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Map received: %d x %d cells @ %.3f m/cell",
    msg->info.width, msg->info.height, msg->info.resolution);

  likelihood_field_.build(*msg);
  map_received_ = true;

  // If we haven't been given an initial pose yet, spread particles over the map
  if (!initialized_) {
    initParticles();
  }
}

void MyAmcl::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Extract yaw from quaternion
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  est_x_     = msg->pose.pose.position.x;
  est_y_     = msg->pose.pose.position.y;
  est_theta_ = yaw;

  // Spread particles around the given pose with small noise
  std::normal_distribution<double> noise_xy(0.0, 0.5);
  std::normal_distribution<double> noise_theta(0.0, 0.2);

  particles_.resize(num_particles_);
  for (auto & p : particles_) {
    p.x      = est_x_     + noise_xy(rng_);
    p.y      = est_y_     + noise_xy(rng_);
    p.theta  = est_theta_ + noise_theta(rng_);
    p.weight = 1.0 / num_particles_;
  }

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "Initial pose set: (%.2f, %.2f, %.2f deg)",
    est_x_, est_y_, est_theta_ * 180.0 / M_PI);
}

void MyAmcl::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x     = msg->pose.pose.position.x;
  double y     = msg->pose.pose.position.y;

  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  if (!odom_initialized_) {
    last_odom_x_     = x;
    last_odom_y_     = y;
    last_odom_theta_ = yaw;
    odom_initialized_ = true;
    return;
  }

  double dx     = x     - last_odom_x_;
  double dy     = y     - last_odom_y_;
  double dtheta = normalizeAngle(yaw - last_odom_theta_);

  // Only update if the robot moved enough (avoids noisy updates when still)
  if (std::abs(dx) < 0.001 && std::abs(dy) < 0.001 && std::abs(dtheta) < 0.001) return;

  last_odom_x_     = x;
  last_odom_y_     = y;
  last_odom_theta_ = yaw;

  if (initialized_) {
    motionUpdate(dx, dy, dtheta);
  }
}

void MyAmcl::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!initialized_ || !map_received_) return;

  sensorUpdate(*msg);
  resample();
  publishPose();
  publishParticles();
  broadcastMapToOdom();
}

// ═══════════════════════════════════════════════════════════════
//  initParticles  — spread randomly over free cells of the map
// ═══════════════════════════════════════════════════════════════

void MyAmcl::initParticles()
{
  // Collect all free cells
  // (only if map is ready, otherwise called again from mapCallback)
  particles_.resize(num_particles_);

  std::uniform_real_distribution<double> angle_dist(-M_PI, M_PI);

  // Use map bounds for uniform spread
  // (a smarter approach would collect free cells only, but this works fine for thesis)
  double map_x_min = 0.0;
  double map_y_min = 0.0;
  // We don't have the map here directly, so just spread and let sensor model kill bad particles
  // In practice the user sets an initial pose via RViz anyway

  std::uniform_real_distribution<double> x_dist(-5.0, 5.0);
  std::uniform_real_distribution<double> y_dist(-5.0, 5.0);

  for (auto & p : particles_) {
    p.x      = x_dist(rng_);
    p.y      = y_dist(rng_);
    p.theta  = angle_dist(rng_);
    p.weight = 1.0 / num_particles_;
  }

  initialized_ = true;
  RCLCPP_INFO(get_logger(), "Particles initialized globally. Set pose in RViz for faster convergence.");
}

// ═══════════════════════════════════════════════════════════════
//  motionUpdate  — odometry motion model (differential drive)
//
//  Based on Probabilistic Robotics (Thrun et al.) Table 5.6
//  Decomposes motion into: rot1 → trans → rot2
// ═══════════════════════════════════════════════════════════════

void MyAmcl::motionUpdate(double dx, double dy, double dtheta)
{
  double trans     = std::sqrt(dx * dx + dy * dy);
  double rot1      = std::atan2(dy, dx);  // direction of translation
  double rot2      = dtheta - rot1;

  for (auto & p : particles_) {
    // Add noise proportional to motion magnitude
    double noisy_rot1  = rot1  + sampleGaussian(alpha1_ * std::abs(rot1)  + alpha2_ * trans);
    double noisy_trans = trans + sampleGaussian(alpha3_ * trans            + alpha4_ * (std::abs(rot1) + std::abs(rot2)));
    double noisy_rot2  = rot2  + sampleGaussian(alpha1_ * std::abs(rot2)  + alpha2_ * trans);

    p.x     += noisy_trans * std::cos(p.theta + noisy_rot1);
    p.y     += noisy_trans * std::sin(p.theta + noisy_rot1);
    p.theta  = normalizeAngle(p.theta + noisy_rot1 + noisy_rot2);
  }
}

// ═══════════════════════════════════════════════════════════════
//  sensorUpdate  — likelihood field sensor model
//
//  For each particle, project a subset of laser beams into the
//  map and look up the precomputed distance-to-obstacle.
//  Score = product of per-beam probabilities.
// ═══════════════════════════════════════════════════════════════

void MyAmcl::sensorUpdate(const sensor_msgs::msg::LaserScan & scan)
{
  const double sigma2 = sigma_hit_ * sigma_hit_;
  const double z_rand_norm = z_rand_ / (2.0 * laser_max_range_);
  double total_weight = 0.0;

  for (auto & p : particles_) {
    double log_weight = 0.0;

    int beam_count = 0;
    for (size_t i = 0; i < scan.ranges.size(); i += laser_subsample_) {
      double range = scan.ranges[i];

      // Skip invalid readings
      if (std::isnan(range) || std::isinf(range)) continue;
      if (range < scan.range_min || range > laser_max_range_) continue;

      // Angle of this beam in the robot frame
      double beam_angle = scan.angle_min + i * scan.angle_increment;

      // Project end point into world frame (from this particle's pose)
      double hit_x = p.x + range * std::cos(p.theta + beam_angle);
      double hit_y = p.y + range * std::sin(p.theta + beam_angle);

      // Lookup nearest obstacle distance from precomputed field
      double dist = likelihood_field_.likelihood(hit_x, hit_y);

      // Gaussian hit probability
      double p_hit  = (1.0 / (std::sqrt(2.0 * M_PI) * sigma_hit_)) *
                       std::exp(-0.5 * dist * dist / sigma2);

      // Combined probability (mixture of Gaussian hit + random)
      double prob = z_hit_ * p_hit + z_rand_norm;
      if (prob < 1e-300) prob = 1e-300;  // clamp to avoid log(0)

      log_weight += std::log(prob);
      beam_count++;
    }

    // Convert back from log space (normalized per beam count to avoid underflow)
    p.weight = (beam_count > 0) ? std::exp(log_weight / beam_count) : 1e-300;
    total_weight += p.weight;
  }

  // Normalize weights
  if (total_weight > 0.0) {
    for (auto & p : particles_) p.weight /= total_weight;
  } else {
    // All weights collapsed — reset uniform (robot is lost)
    RCLCPP_WARN(get_logger(), "All particle weights near zero — robot may be lost.");
    for (auto & p : particles_) p.weight = 1.0 / num_particles_;
  }

  // ── Compute estimated pose as weighted mean ──
  double mean_x = 0.0, mean_y = 0.0;
  double sin_sum = 0.0, cos_sum = 0.0;
  for (const auto & p : particles_) {
    mean_x   += p.weight * p.x;
    mean_y   += p.weight * p.y;
    sin_sum  += p.weight * std::sin(p.theta);
    cos_sum  += p.weight * std::cos(p.theta);
  }
  est_x_     = mean_x;
  est_y_     = mean_y;
  est_theta_ = std::atan2(sin_sum, cos_sum);
}

// ═══════════════════════════════════════════════════════════════
//  resample  — low variance resampling (Thrun et al. Table 4.4)
//
//  This is better than naive multinomial resampling because it
//  uses a single random number and sweeps the wheel uniformly,
//  so it preserves diversity better.
// ═══════════════════════════════════════════════════════════════

void MyAmcl::resample()
{
  std::vector<Particle> new_particles;
  new_particles.reserve(num_particles_);

  std::uniform_real_distribution<double> dist(0.0, 1.0 / num_particles_);
  double r = dist(rng_);
  double c = particles_[0].weight;
  int i = 0;

  for (int m = 0; m < num_particles_; ++m) {
    double U = r + static_cast<double>(m) / num_particles_;
    while (U > c && i < num_particles_ - 1) {
      ++i;
      c += particles_[i].weight;
    }
    new_particles.push_back(particles_[i]);
    new_particles.back().weight = 1.0 / num_particles_;
  }

  particles_ = std::move(new_particles);
}

// ═══════════════════════════════════════════════════════════════
//  publishPose  — /amcl_pose
// ═══════════════════════════════════════════════════════════════

void MyAmcl::publishPose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp    = now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.pose.position.x = est_x_;
  pose_msg.pose.pose.position.y = est_y_;
  pose_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, est_theta_);
  pose_msg.pose.pose.orientation = tf2::toMsg(q);

  // Simple diagonal covariance (you can compute actual covariance from particles)
  pose_msg.pose.covariance[0]  = 0.1;  // xx
  pose_msg.pose.covariance[7]  = 0.1;  // yy
  pose_msg.pose.covariance[35] = 0.05; // theta theta

  pose_pub_->publish(pose_msg);
}

// ═══════════════════════════════════════════════════════════════
//  publishParticles  — /particle_cloud (visible in RViz)
// ═══════════════════════════════════════════════════════════════

void MyAmcl::publishParticles()
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp    = now();
  msg.header.frame_id = "map";
  msg.poses.reserve(particles_.size());

  for (const auto & p : particles_) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, p.theta);
    pose.orientation = tf2::toMsg(q);
    msg.poses.push_back(pose);
  }

  particle_pub_->publish(msg);
}

// ═══════════════════════════════════════════════════════════════
//  broadcastMapToOdom  — TF: map → odom
//
//  The map→odom transform is the "correction" from AMCL.
//  It accounts for the drift between odometry and the true map.
//
//  We know:  map→base_link  (our estimate)
//  We know:  odom→base_link (from /odom)
//  We want:  map→odom = map→base_link * (odom→base_link)^-1
// ═══════════════════════════════════════════════════════════════

void MyAmcl::broadcastMapToOdom()
{
  // 1. Build map→base_link from our estimated pose
  tf2::Transform map_to_base;
  map_to_base.setOrigin(tf2::Vector3(est_x_, est_y_, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, est_theta_);
  map_to_base.setRotation(q);

  // 2. Look up odom→base_link from TF tree
  geometry_msgs::msg::TransformStamped odom_to_base_msg;
  try {
    odom_to_base_msg = tf_buffer_->lookupTransform(
      "odom", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Could not lookup odom->base_link: %s", ex.what());
    return;
  }

  tf2::Transform odom_to_base;
  tf2::fromMsg(odom_to_base_msg.transform, odom_to_base);

  // 3. map→odom = map→base * base→odom = map→base * (odom→base)^-1
  tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

  // 4. Broadcast
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp    = now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id  = "odom";
  tf_msg.transform       = tf2::toMsg(map_to_odom);

  tf_broadcaster_->sendTransform(tf_msg);
}

// ═══════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════

double MyAmcl::normalizeAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double MyAmcl::sampleGaussian(double sigma)
{
  std::normal_distribution<double> d(0.0, sigma);
  return d(rng_);
}

}  // namespace my_amcl

// ─────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_amcl::MyAmcl>());
  rclcpp::shutdown();
  return 0;
}

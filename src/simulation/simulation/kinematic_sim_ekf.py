#!/usr/bin/env python3

import math

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node


class KinematicSim(Node):

    def __init__(self):
        super().__init__('kinematic_sim')

        # ---------------- Robot parameters ----------------
        self.wheel_radius = 0.065
        self.wheel_base   = 0.457

        self.robot_length = 0.40
        self.robot_width  = 0.30

        # ---------------- Subscribers ----------------

        # 1. Wheel odometry  →  used for both trajectories
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/wheel',
            self.odom_callback,
            10
        )

        # 2. IMU  →  omega (angular velocity z) for the fused heading
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # 3. Filtered odometry  →  e.g. from robot_localization EKF
        self.filtered_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.filtered_callback,
            10
        )

        # ---------------- Timers ----------------
        self.integrate_timer = self.create_timer(0.02, self.integrate_fused_pose)
        self.plot_timer      = self.create_timer(0.05, self.update_plot)

        # ----------------------------------------------------------------
        # TRAJECTORY 1 — raw wheel odometry  (from /odom/wheel directly)
        # ----------------------------------------------------------------
        self.odom_x     = 0.0
        self.odom_y     = 0.0
        self.odom_theta = 0.0

        self.odom_x_hist = [0.0]
        self.odom_y_hist = [0.0]

        # ----------------------------------------------------------------
        # TRAJECTORY 2 — fused: wheel linear velocity  +  IMU heading
        #
        #   Step 1 : read wheel odom  →  extract v (linear.x) and
        #            raw omega (angular.z)  [both already in m/s, rad/s]
        #   Step 2 : run forward kinematic to get wheel-based v
        #            (round-trip through inverse + forward keeps the
        #             same wheel-model, same as your original code)
        #   Step 3 : integrate IMU omega  →  imu_theta
        #   Step 4 : propagate x,y using v  +  imu_theta
        # ----------------------------------------------------------------
        self.fused_x     = 0.0
        self.fused_y     = 0.0
        self.fused_theta = 0.0      # driven purely by IMU integration

        self.fused_x_hist = [0.0]
        self.fused_y_hist = [0.0]

        # Latest wheel velocity extracted from /odom/wheel
        self.wheel_v     = 0.0      # linear  (m/s)
        self.wheel_omega = 0.0      # angular (rad/s)  — kept for reference

        # Latest IMU angular velocity (rad/s around z)
        self.imu_omega = 0.0

        # ----------------------------------------------------------------
        # TRAJECTORY 3 — /odometry/filtered  (e.g. EKF from robot_localization)
        # ----------------------------------------------------------------
        self.filtered_x     = 0.0
        self.filtered_y     = 0.0
        self.filtered_theta = 0.0

        self.filtered_x_hist = [0.0]
        self.filtered_y_hist = [0.0]

        # ---------------- Time tracking ----------------
        self.last_integrate_time = self.get_clock().now()

        # ---------------- Matplotlib ----------------
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

        self.line_odom, = self.ax.plot(
            [], [], 'b-', linewidth=1.5, alpha=0.6,
            label='Wheel odom path')

        self.line_fused, = self.ax.plot(
            [], [], 'r-', linewidth=1.5, alpha=0.6,
            label='Fused odom path  (wheel v + IMU theta)')

        # robot rectangle — wheel odom pose (blue solid outline)
        self.robot_odom_plot, = self.ax.plot([], [], 'b-', linewidth=2.5,
                                              label='Odom robot')

        # robot rectangle — fused pose (red dashed outline so both visible)
        self.robot_fused_plot, = self.ax.plot([], [], 'r--', linewidth=2.5,
                                               label='Fused robot')

        # front direction marker — odom robot (filled blue dot)
        self.front_odom_plot, = self.ax.plot([], [], 'bo', markersize=6)

        # front direction marker — fused robot (filled red dot)
        self.front_fused_plot, = self.ax.plot([], [], 'ro', markersize=6)

        # trajectory — filtered odom (green)
        self.line_filtered, = self.ax.plot(
            [], [], 'g-', linewidth=1.5, alpha=0.6,
            label='Filtered odom path  (/odometry/filtered)')

        # robot rectangle — filtered pose (green dotted outline)
        self.robot_filtered_plot, = self.ax.plot([], [], 'g:', linewidth=2.5,
                                                  label='Filtered robot')

        # front direction marker — filtered robot (filled green dot)
        self.front_filtered_plot, = self.ax.plot([], [], 'go', markersize=6)

        self.ax.set_title('Wheel Odom  vs  Fused (wheel v + IMU θ)  vs  Filtered Odom')
        self.ax.set_xlabel('X position (meters)')
        self.ax.set_ylabel('Y position (meters)')
        self.ax.grid(True)
        self.ax.legend()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        plt.show(block=False)

        self.get_logger().info(
            'KinematicSim started — comparing:\n'
            '  [Blue]  Wheel odom       (/odom/wheel)\n'
            '  [Red]   Fused odom       (wheel v + IMU theta)\n'
            '  [Green] Filtered odom    (/odometry/filtered)'
        )

    # =================================================================
    # KINEMATIC HELPERS
    # =================================================================

    def inverse_kinematic(self, v, omega):
        """
        (v m/s, omega rad/s)  →  (w_right, w_left) in 0.1-rpm units
        """
        factor  = (30.0 / math.pi) * 10.0
        w_right = (v / self.wheel_radius) + ((self.wheel_base * omega) / (2.0 * self.wheel_radius))
        w_left  = (v / self.wheel_radius) - ((self.wheel_base * omega) / (2.0 * self.wheel_radius))
        return w_right * factor, w_left * factor

    def forward_kinematic(self, w_right_rpm, w_left_rpm):
        """
        (w_right, w_left) in 0.1-rpm units  →  (v m/s, omega rad/s)
        """
        w_right = w_right_rpm * 0.1 * (math.pi / 30.0)
        w_left  = w_left_rpm  * 0.1 * (math.pi / 30.0)
        v     = self.wheel_radius * ((w_right + w_left) / 2.0)
        omega = self.wheel_radius * ((w_right - w_left) / self.wheel_base)
        return v, omega

    # =================================================================
    # CALLBACKS
    # =================================================================

    def odom_callback(self, msg):
        """
        TRAJECTORY 1 — store wheel odometry pose directly.
        Also extract v and omega from the twist for trajectory 2.
        """
        # --- Trajectory 1: raw wheel odom pose ---
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.odom_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.odom_x_hist.append(self.odom_x)
        self.odom_y_hist.append(self.odom_y)

        # --- Extract wheel velocity for trajectory 2 ---
        # /odom/wheel publishes actual robot velocity in the twist field
        raw_v     = msg.twist.twist.linear.x
        raw_omega = msg.twist.twist.angular.z

        # Round-trip through kinematics to stay consistent with wheel model
        w_right_rpm, w_left_rpm = self.inverse_kinematic(raw_v, raw_omega)
        self.wheel_v, self.wheel_omega = self.forward_kinematic(w_right_rpm, w_left_rpm)

    def imu_callback(self, msg):
        """
        Store IMU angular velocity around z-axis (yaw rate).
        This replaces wheel-odometry heading in trajectory 2.
        """
        self.imu_omega = msg.angular_velocity.z

    def filtered_callback(self, msg):
        """
        TRAJECTORY 3 — store filtered odometry pose directly.
        Typically published by robot_localization EKF/UKF node.
        Pose and heading are read straight from the message.
        """
        self.filtered_x = msg.pose.pose.position.x
        self.filtered_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.filtered_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.filtered_x_hist.append(self.filtered_x)
        self.filtered_y_hist.append(self.filtered_y)

    # =================================================================
    # INTEGRATE FUSED POSE  (runs at 50 Hz via timer)
    # =================================================================

    def integrate_fused_pose(self):
        """
        TRAJECTORY 2:
          theta  →  integrated from IMU angular velocity
          x, y   →  integrated from wheel linear velocity + IMU theta
        """
        now = self.get_clock().now()
        dt  = (now - self.last_integrate_time).nanoseconds * 1e-9
        self.last_integrate_time = now

        if dt <= 0:
            return

        # Step 1: Update heading using IMU omega
        self.fused_theta += self.imu_omega * dt
        self.fused_theta  = self.wrap_angle(self.fused_theta)

        # Step 2: Propagate x, y using wheel v  +  IMU-derived heading
        self.fused_x += self.wheel_v * math.cos(self.fused_theta) * dt
        self.fused_y += self.wheel_v * math.sin(self.fused_theta) * dt

        self.fused_x_hist.append(self.fused_x)
        self.fused_y_hist.append(self.fused_y)

    # =================================================================
    # ROBOT SHAPE
    # =================================================================

    def draw_robot(self, x, y, theta):
        L = self.robot_length
        W = self.robot_width

        corners = [
            [ L/2,  W/2],
            [ L/2, -W/2],
            [-L/2, -W/2],
            [-L/2,  W/2],
            [ L/2,  W/2],   # close rectangle
        ]

        xs, ys = [], []
        for cx, cy in corners:
            xr = x + cx * math.cos(theta) - cy * math.sin(theta)
            yr = y + cx * math.sin(theta) + cy * math.cos(theta)
            xs.append(xr)
            ys.append(yr)

        # Front center point — shows which direction robot is facing
        front_x = x + (L / 2) * math.cos(theta)
        front_y = y + (L / 2) * math.sin(theta)

        return xs, ys, front_x, front_y

    # =================================================================
    # PLOT UPDATE
    # =================================================================

    def update_plot(self):

        if not plt.fignum_exists(self.fig.number):
            rclpy.shutdown()
            return

        # Trajectory 1 — wheel odom (blue)
        self.line_odom.set_data(self.odom_x_hist, self.odom_y_hist)

        # Trajectory 2 — fused (red)
        self.line_fused.set_data(self.fused_x_hist, self.fused_y_hist)

        # Robot 1 — wheel odom pose + odom_theta (blue outline)
        xs, ys, fx, fy = self.draw_robot(self.odom_x, self.odom_y, self.odom_theta)
        self.robot_odom_plot.set_data(xs, ys)
        self.front_odom_plot.set_data([fx], [fy])

        # Robot 2 — fused pose + fused_theta from IMU (red dashed outline)
        #           fused_theta is integrated from imu_omega so it
        #           correctly follows the IMU rotation
        xs, ys, fx, fy = self.draw_robot(self.fused_x, self.fused_y, self.fused_theta)
        self.robot_fused_plot.set_data(xs, ys)
        self.front_fused_plot.set_data([fx], [fy])

        # Trajectory 3 — filtered odom (green)
        self.line_filtered.set_data(self.filtered_x_hist, self.filtered_y_hist)

        # Robot 3 — filtered pose + filtered_theta (green dotted outline)
        xs, ys, fx, fy = self.draw_robot(self.filtered_x, self.filtered_y, self.filtered_theta)
        self.robot_filtered_plot.set_data(xs, ys)
        self.front_filtered_plot.set_data([fx], [fy])

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # =================================================================
    # UTILS
    # =================================================================

    def quaternion_to_yaw(self, x, y, z, w):
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


# =========================================================
# MAIN
# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = KinematicSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
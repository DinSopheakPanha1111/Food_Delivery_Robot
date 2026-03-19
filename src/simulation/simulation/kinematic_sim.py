#!/usr/bin/env python3

import math

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class KinematicSim(Node):

    def __init__(self):
        super().__init__('kinematic_sim')

        # ---------------- Robot parameters ----------------
        self.wheel_radius = 0.065
        self.wheel_base = 0.457

        self.robot_length = 0.40
        self.robot_width = 0.30

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/wheel',
            self.odom_callback,
            10
        )

        # ---------------- Timers ----------------
        self.integrate_timer = self.create_timer(0.02, self.integrate_cmd_pose)
        self.plot_timer = self.create_timer(0.05, self.update_plot)

        # ---------------- cmd_vel state ----------------
        self.cmd_v = 0.0
        self.cmd_omega = 0.0

        # ---------------- Integrated pose ----------------
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_theta = 0.0

        # ---------------- Odom pose ----------------
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # ---------------- Time ----------------
        self.last_integrate_time = self.get_clock().now()

        # ---------------- Trajectory buffers ----------------
        # FIX: use plain lists (no maxlen cap) so history is never erased
        self.cmd_x_hist = [0.0]
        self.cmd_y_hist = [0.0]

        self.odom_x_hist = [0.0]
        self.odom_y_hist = [0.0]

        # ---------------- Matplotlib ----------------
        plt.ion()

        self.fig, self.ax = plt.subplots()

        # trajectories
        self.line_odom, = self.ax.plot([], [], 'b-', linewidth=2,
                                       label="Odom trajectory (meters)")

        self.line_cmd, = self.ax.plot([], [], 'r-', linewidth=2,
                                      label="Cmd_vel predicted trajectory (meters)")

        # robot outline
        self.robot_plot, = self.ax.plot([], [], 'k-', linewidth=2)

        self.ax.set_title("Robot Top View Trajectory")

        self.ax.set_xlabel("X position (meters)")
        self.ax.set_ylabel("Y position (meters)")

        self.ax.grid(True)
        self.ax.legend()

        self.ax.set_aspect('equal')
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        plt.show(block=False)

        self.get_logger().info("Kinematic simulation started")

    # =====================================================
    # KINEMATIC FUNCTIONS
    # =====================================================

    def inverse_kinematic(self, v_cmd, omega_cmd):

        w_right = (v_cmd / self.wheel_radius) + ((self.wheel_base * omega_cmd) / (2.0 * self.wheel_radius))
        w_left  = (v_cmd / self.wheel_radius) - ((self.wheel_base * omega_cmd) / (2.0 * self.wheel_radius))

        # convert rad/s -> 0.1 rpm units
        factor = (30.0 / math.pi) * 10.0

        return w_right * factor, w_left * factor

    def forward_kinematic(self, w_right_rpm, w_left_rpm):

        w_right = w_right_rpm * 0.1 * (math.pi / 30.0)
        w_left  = w_left_rpm  * 0.1 * (math.pi / 30.0)

        v     = self.wheel_radius * ((w_right + w_left) / 2.0)
        omega = self.wheel_radius * ((w_right - w_left) / self.wheel_base)

        return v, omega

    # =====================================================
    # CALLBACKS
    # =====================================================

    def cmd_callback(self, msg):

        self.cmd_v     = msg.linear.x
        self.cmd_omega = msg.angular.z

    def odom_callback(self, msg):

        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        self.odom_theta = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # FIX: always append — list never drops old points
        self.odom_x_hist.append(self.odom_x)
        self.odom_y_hist.append(self.odom_y)

    # =====================================================
    # INTEGRATE CMD_VEL
    # =====================================================

    def integrate_cmd_pose(self):

        now = self.get_clock().now()
        dt  = (now - self.last_integrate_time).nanoseconds * 1e-9
        self.last_integrate_time = now

        if dt <= 0:
            return

        w_right, w_left = self.inverse_kinematic(self.cmd_v, self.cmd_omega)
        v, omega        = self.forward_kinematic(w_right, w_left)

        self.cmd_x     += v * math.cos(self.cmd_theta) * dt
        self.cmd_y     += v * math.sin(self.cmd_theta) * dt
        self.cmd_theta += omega * dt
        self.cmd_theta  = self.wrap_angle(self.cmd_theta)

        # FIX: always append — list never drops old points
        self.cmd_x_hist.append(self.cmd_x)
        self.cmd_y_hist.append(self.cmd_y)

    # =====================================================
    # ROBOT SHAPE
    # =====================================================

    def draw_robot(self, x, y, theta):

        L = self.robot_length
        W = self.robot_width

        corners = [
            [ L/2,  W/2],
            [ L/2, -W/2],
            [-L/2, -W/2],
            [-L/2,  W/2],
            [ L/2,  W/2],   # close the rectangle
        ]

        xs, ys = [], []

        for cx, cy in corners:
            xr = x + cx * math.cos(theta) - cy * math.sin(theta)
            yr = y + cx * math.sin(theta) + cy * math.cos(theta)
            xs.append(xr)
            ys.append(yr)

        return xs, ys

    # =====================================================
    # PLOT UPDATE
    # =====================================================

    def update_plot(self):

        if not plt.fignum_exists(self.fig.number):
            rclpy.shutdown()
            return

        # full trajectory — all points since startup
        self.line_odom.set_data(self.odom_x_hist, self.odom_y_hist)
        self.line_cmd.set_data(self.cmd_x_hist,  self.cmd_y_hist)

        # robot shape drawn at current odom pose
        xs, ys = self.draw_robot(self.odom_x, self.odom_y, self.odom_theta)
        self.robot_plot.set_data(xs, ys)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    # =====================================================
    # UTILS
    # =====================================================

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
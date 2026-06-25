#!/usr/bin/env python3
"""
nav_realtime_plot.py
====================
ROS 2 node: navigate to a fixed waypoint and show four live plots.

Position / orientation source: TF  map -> base_footprint  (not amcl_pose)
Velocity source              : /cmd_vel  (linear.x  and  angular.z)
Path source                  : /plan  (first plan only)

RViz topics (latched, republished at 1 Hz so they persist after goal):
  /viz/planned_path   -- nav_msgs/Path  -- the Nav2 planned path
  /viz/actual_path    -- nav_msgs/Path  -- the robot's actual travelled path

On completion the script auto-increments a test counter and saves:
  test_N_figure.npz   -- reload with --load flag
  test_N_data.txt     -- raw timestamped data for every channel

Re-open a saved figure:
  python3 nav_realtime_plot.py --load test_1_figure.npz
"""

import argparse
import math
import os
import numpy as np
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path

# TF2
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException, ExtrapolationException)

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

matplotlib.rcParams.update({
    "font.family":       "serif",
    "font.serif":        ["Times New Roman", "Times", "DejaVu Serif"],
    "axes.titlesize":    11,
    "axes.labelsize":    10,
    "xtick.labelsize":   9,
    "ytick.labelsize":   9,
    "legend.fontsize":   9,
    "figure.titlesize":  12,
})

# ── Fixed waypoint ───────────────────────────────────────────────────────────
GOAL_X     = 2.4
GOAL_Y     = 6.0
GOAL_THETA =  90.0   # degrees
GOAL_FRAME = "map"

# ── Robot initial pose (where the robot physically starts) ───────────────────
# These must match the robot's real starting position on the map.
# Same values you would set with the RViz "2D Pose Estimate" button.
INIT_X     = 0.0
INIT_Y     = 0.0
INIT_THETA = 0.0   # degrees

# TF frames
TF_TARGET = "map"
TF_SOURCE = "base_footprint"

# ── Output directory ─────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── Colour palette ───────────────────────────────────────────────────────────
ACCENT = "#0072B2"
ORANGE = "#D55E00"
GREEN  = "#009E73"
RED    = "#CC0000"
YELLOW = "#E69F00"
PURPLE = "#7B2D8B"
CYAN   = "#0096C7"
PINK   = "#C2185B"
GRID_C = "#CCCCCC"
BG     = "white"
FG     = "#111111"

# ── Helpers ──────────────────────────────────────────────────────────────────

def yaw_deg_to_quat(yaw_deg: float):
    r = math.radians(yaw_deg)
    return math.sin(r / 2.0), math.cos(r / 2.0)


def quat_to_yaw_deg(qz: float, qw: float) -> float:
    return math.degrees(2.0 * math.atan2(qz, qw))


def next_test_index(directory: str) -> int:
    idx = 1
    while os.path.exists(os.path.join(directory, f"test_{idx}_figure.npz")):
        idx += 1
    return idx


def _autoscale(ax):
    ax.relim()
    ax.autoscale_view()


# ── ROS 2 Node ───────────────────────────────────────────────────────────────

class NavPlotNode(Node):

    def __init__(self):
        super().__init__("nav_realtime_plot")

        # ── Action client ─────────────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── TF2 ───────────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(Path,  "/plan",    self._plan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._vel_cb,  10)

        # ── Initial pose publisher ─────────────────────────────────────────
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        # ── RViz publishers (TRANSIENT_LOCAL = latched) ───────────────────
        latch_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_planned = self.create_publisher(Path, "/viz/planned_path", latch_qos)
        self._pub_actual  = self.create_publisher(Path, "/viz/actual_path",  latch_qos)

        # Republish both paths at 1 Hz
        self.create_timer(1.0, self._republish_paths)

        # ── Thread-safe data stores ───────────────────────────────────────
        self._lock = threading.Lock()

        self.planned_x: list[float] = []
        self.planned_y: list[float] = []

        self.actual_x:  list[float] = []
        self.actual_y:  list[float] = []

        self.t_stamps:  list[float] = []
        self.err_x:     list[float] = []
        self.err_y:     list[float] = []
        self.yaw_data:  list[float] = []

        self.t_vel:     list[float] = []
        self.vel_lin:   list[float] = []
        self.vel_ang:   list[float] = []

        # Control flags
        self._navigating          = False
        self._nav_done            = threading.Event()
        self._plan_set            = False
        self._t_start: float|None = None
        self._nav_succeeded       = False

        # Poll TF at 20 Hz while navigating
        self._tf_timer = self.create_timer(0.05, self._tf_poll)

    # ── Publish initial pose to initialize AMCL ───────────────────────────

    def publish_initial_pose(self):
        """
        Publish /initialpose so AMCL knows where the robot starts.
        This replaces the manual RViz '2D Pose Estimate' step.
        """
        qz, qw = yaw_deg_to_quat(INIT_THETA)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp    = self.get_clock().now().to_msg()

        msg.pose.pose.position.x    = INIT_X
        msg.pose.pose.position.y    = INIT_Y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Standard covariance values matching Nav2 defaults
        msg.pose.covariance[0]  = 0.25   # x variance
        msg.pose.covariance[7]  = 0.25   # y variance
        msg.pose.covariance[35] = 0.07   # yaw variance

        self._init_pose_pub.publish(msg)
        self.get_logger().info(
            f"Initial pose published: x={INIT_X}, y={INIT_Y}, theta={INIT_THETA} deg"
        )

    # ── RViz republish ────────────────────────────────────────────────────

    def _republish_paths(self):
        now = self.get_clock().now().to_msg()

        with self._lock:
            px = list(self.planned_x)
            py = list(self.planned_y)
            ax = list(self.actual_x)
            ay = list(self.actual_y)

        if px:
            planned_msg = Path()
            planned_msg.header.frame_id = GOAL_FRAME
            planned_msg.header.stamp    = now
            for x, y in zip(px, py):
                ps = PoseStamped()
                ps.header.frame_id = GOAL_FRAME
                ps.header.stamp    = now
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                planned_msg.poses.append(ps)
            self._pub_planned.publish(planned_msg)

        if ax:
            actual_msg = Path()
            actual_msg.header.frame_id = GOAL_FRAME
            actual_msg.header.stamp    = now
            for x, y in zip(ax, ay):
                ps = PoseStamped()
                ps.header.frame_id = GOAL_FRAME
                ps.header.stamp    = now
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                actual_msg.poses.append(ps)
            self._pub_actual.publish(actual_msg)

    # ── TF poll ───────────────────────────────────────────────────────────

    def _tf_poll(self):
        if not self._navigating:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                TF_TARGET, TF_SOURCE,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        t   = tf.transform.translation
        r   = tf.transform.rotation
        yaw = quat_to_yaw_deg(r.z, r.w)
        elapsed = time.time() - self._t_start

        with self._lock:
            if self.planned_x:
                dx = np.asarray(self.planned_x) - t.x
                dy = np.asarray(self.planned_y) - t.y
                i  = int(np.argmin(dx * dx + dy * dy))
                ex = self.planned_x[i] - t.x
                ey = self.planned_y[i] - t.y
            else:
                ex = float("nan")
                ey = float("nan")
            self.actual_x.append(t.x)
            self.actual_y.append(t.y)
            self.t_stamps.append(elapsed)
            self.err_x.append(ex)
            self.err_y.append(ey)
            self.yaw_data.append(yaw)

    # ── Plan callback ─────────────────────────────────────────────────────

    def _plan_cb(self, msg: Path):
        if self._plan_set or not msg.poses:
            return
        with self._lock:
            self.planned_x = [p.pose.position.x for p in msg.poses]
            self.planned_y = [p.pose.position.y for p in msg.poses]
            self._plan_set = True
        self.get_logger().info(f"Plan received: {len(msg.poses)} poses")

    # ── Velocity callback ─────────────────────────────────────────────────

    def _vel_cb(self, msg: Twist):
        if not self._navigating:
            return
        elapsed = time.time() - self._t_start
        with self._lock:
            self.t_vel.append(elapsed)
            self.vel_lin.append(msg.linear.x)
            self.vel_ang.append(msg.angular.z)

    # ── Action callbacks ──────────────────────────────────────────────────

    def _goal_response_cb(self, future):
        gh = future.result()
        print(f"[DEBUG] goal_response_cb fired, accepted={gh.accepted}")
        if not gh.accepted:
            self.get_logger().error("Goal rejected!")
            self._navigating = False
            self._nav_done.set()
            return
        self.get_logger().info("Goal accepted -- navigating ...")
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        res = future.result()
        self._nav_succeeded = res.status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            "Navigation SUCCEEDED!" if self._nav_succeeded
            else f"Navigation FAILED (status {res.status})"
        )
        self._navigating = False
        self._nav_done.set()

    # ── Send goal ─────────────────────────────────────────────────────────

    def send_goal(self):
        qz, qw = yaw_deg_to_quat(GOAL_THETA)

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.header.frame_id    = GOAL_FRAME
        goal.pose.pose.position.x    = GOAL_X
        goal.pose.pose.position.y    = GOAL_Y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._plan_set   = False
        self._navigating = True
        self._t_start    = time.time()
        self._nav_done.clear()

        f = self._action_client.send_goal_async(goal)
        f.add_done_callback(self._goal_response_cb)


# ── Build figure ─────────────────────────────────────────────────────────────

def build_figure(test_label: str):
    plt.style.use("default")

    fig = plt.figure(figsize=(16, 9), facecolor=BG)
    fig.suptitle(
        f"Nav2 Monitor -- {test_label}  |  "
        f"Goal: x={GOAL_X} m, y={GOAL_Y} m, theta={GOAL_THETA} deg  |  "
        f"Pose source: TF map -> base",
        color=FG, fontsize=11, fontweight="bold", y=0.98,
    )

    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.30,
                           left=0.07, right=0.97, top=0.93, bottom=0.07)

    def _style(ax, title, xlabel, ylabel):
        ax.set_facecolor(BG)
        ax.set_title(title, color=FG, pad=6)
        ax.set_xlabel(xlabel, color=FG)
        ax.set_ylabel(ylabel, color=FG)
        ax.tick_params(colors=FG)
        ax.spines[:].set_color(GRID_C)
        ax.grid(True, color=GRID_C, linestyle="--", linewidth=0.6)

    ax_map = fig.add_subplot(gs[0, 0])
    _style(ax_map, "Planned Path vs Actual Path", "X (m)", "Y (m)")
    ax_map.set_aspect("equal", adjustable="datalim")
    line_plan,   = ax_map.plot([], [], color=ACCENT, lw=1.8, ls="--", label="Robot desired path")
    line_actual, = ax_map.plot([], [], color=ORANGE, lw=2.0,          label="Robot actual path")
    ax_map.legend(loc="upper left", fontsize=8, facecolor=BG, labelcolor=FG, edgecolor=GRID_C).set_draggable(True)

    ax_err = fig.add_subplot(gs[0, 1])
    _style(ax_err, "Path Tracking Error vs Time", "Time (s)", "Error (m)")
    ax_err.axhline(0, color="#999999", lw=0.8, ls=":")
    line_ex, = ax_err.plot([], [], color=RED,   lw=1.8, label="X error")
    line_ey, = ax_err.plot([], [], color=GREEN, lw=1.8, label="Y error")
    ax_err.legend(loc="upper right", fontsize=8, facecolor=BG, labelcolor=FG, edgecolor=GRID_C).set_draggable(True)

    ax_yaw = fig.add_subplot(gs[1, 0])
    _style(ax_yaw, "Orientation Profile", "Time (s)", "Yaw (deg)")
    line_yaw, = ax_yaw.plot([], [], color=PURPLE, lw=2.0, label="Yaw")
    ax_yaw.legend(loc="upper right", fontsize=8, facecolor=BG, labelcolor=FG, edgecolor=GRID_C).set_draggable(True)

    ax_vel = fig.add_subplot(gs[1, 1])
    _style(ax_vel, "Velocity Profile", "Time (s)", "Velocity (m/s, rad/s)")
    ax_vel.axhline(0, color="#999999", lw=0.8, ls=":")
    line_vlin, = ax_vel.plot([], [], color=CYAN, lw=1.8, label="Linear velocity")
    line_vang, = ax_vel.plot([], [], color=PINK, lw=1.8, label="Angular velocity")
    ax_vel.legend(loc="upper right", fontsize=8, facecolor=BG, labelcolor=FG, edgecolor=GRID_C).set_draggable(True)

    status_txt = fig.text(0.5, 0.003, "Status: waiting ...",
                          ha="center", color="#555555", fontsize=9)

    lines = dict(plan=line_plan, actual=line_actual,
                 ex=line_ex, ey=line_ey,
                 yaw=line_yaw,
                 vlin=line_vlin, vang=line_vang)
    axes  = dict(map=ax_map, err=ax_err, yaw=ax_yaw, vel=ax_vel)
    return fig, axes, lines, status_txt


# ── Live plotting loop ────────────────────────────────────────────────────────

def run_live_plots(node: NavPlotNode, fig, axes, lines, status_txt):
    plt.ion()
    plt.show(block=False)

    while True:
        with node._lock:
            px = list(node.planned_x);  py = list(node.planned_y)
            ax = list(node.actual_x);   ay = list(node.actual_y)
            ts = list(node.t_stamps)
            ex = list(node.err_x);      ey = list(node.err_y)
            yw = list(node.yaw_data)
            tv = list(node.t_vel)
            vl = list(node.vel_lin);    va = list(node.vel_ang)

        if px:
            lines["plan"].set_data(px, py)
        if ax:
            lines["actual"].set_data(ax, ay)
        _autoscale(axes["map"])

        if ts and len(ts) == len(ex) == len(ey):
            lines["ex"].set_data(ts, ex)
            lines["ey"].set_data(ts, ey)
            _autoscale(axes["err"])

        if ts and len(ts) == len(yw):
            lines["yaw"].set_data(ts, yw)
            _autoscale(axes["yaw"])

        if tv and len(tv) == len(vl) == len(va):
            lines["vlin"].set_data(tv, vl)
            lines["vang"].set_data(tv, va)
            _autoscale(axes["vel"])

        nav_done = node._nav_done.is_set()
        if nav_done:
            status_txt.set_text("Status: Navigation COMPLETE -- figure saved")
        else:
            elapsed = time.time() - node._t_start if node._t_start else 0.0
            status_txt.set_text(f"Status: Navigating ...  elapsed: {elapsed:.1f} s")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        if nav_done:
            break

        time.sleep(0.1)


# ── Final value annotations ───────────────────────────────────────────────────

def annotate_final_values(node, axes: dict):
    with node._lock:
        ax_x = list(node.actual_x);  ax_y = list(node.actual_y)
        ts   = list(node.t_stamps)
        ex   = list(node.err_x);     ey   = list(node.err_y)
        yw   = list(node.yaw_data)
        tv   = list(node.t_vel)
        vl   = list(node.vel_lin);   va   = list(node.vel_ang)

    DOT  = dict(marker="o", markersize=6, zorder=6, clip_on=False)
    HLIN = dict(linestyle="--", linewidth=0.9, alpha=0.55)

    _selected = [None]

    def _on_click(event):
        prev = _selected[0]
        if prev is not None:
            try:
                prev.get_bbox_patch().set_edgecolor(prev._orig_ec)
                prev.get_bbox_patch().set_linewidth(1.0)
            except Exception:
                pass
            _selected[0] = None

        from matplotlib.text import Annotation
        for ax in axes.values():
            for artist in ax.get_children():
                if not isinstance(artist, Annotation):
                    continue
                bb = artist.get_window_extent()
                if bb.contains(event.x, event.y):
                    _selected[0] = artist
                    patch = artist.get_bbox_patch()
                    if patch is not None:
                        patch.set_edgecolor("black")
                        patch.set_linewidth(2.5)
                    ax.figure.canvas.draw_idle()
                    return

    def _on_key(event):
        if event.key not in ("delete", "backspace"):
            return
        ann = _selected[0]
        if ann is None:
            return
        try:
            ann.remove()
        except Exception:
            pass
        _selected[0] = None
        for ax in axes.values():
            ax.figure.canvas.draw_idle()

    fig = list(axes.values())[0].figure
    fig.canvas.mpl_connect("button_press_event", _on_click)
    fig.canvas.mpl_connect("key_press_event",    _on_key)

    def _ann(ax, x, y, text, color, offset_x=12, offset_y=0):
        xl, xr = ax.get_xlim()
        yb, yt = ax.get_ylim()
        xspan  = xr - xl if xr != xl else 1.0
        yspan  = yt - yb if yt != yb else 1.0
        tx = x + offset_x * xspan / 300.0
        ty = y + offset_y * yspan / 300.0
        ann = ax.annotate(
            text,
            xy=(x, y),       xycoords="data",
            xytext=(tx, ty), textcoords="data",
            ha="left", va="center",
            fontsize=9, fontweight="bold", color=color,
            bbox=dict(boxstyle="round,pad=0.3", fc="white",
                      ec=color, alpha=0.92),
            arrowprops=dict(arrowstyle="-", color=color, lw=0.8, alpha=0.6),
        )
        ann._orig_ec = ann.get_bbox_patch().get_edgecolor()
        ann.draggable(use_blit=True)
        return ann

    if ax_x and ax_y:
        a      = axes["map"]
        xv, yv = ax_x[-1], ax_y[-1]
        a.plot(xv, yv, color=ORANGE, **DOT)
        _ann(a, xv, yv,
             f"Final\nx = {xv:.3f} m\ny = {yv:.3f} m",
             ORANGE, offset_x=12, offset_y=0)

    if ts and len(ts) == len(ex) == len(ey):
        a     = axes["err"]
        t_end = ts[-1]
        xl, _ = a.get_xlim()
        for yv, color, label, oy in [
            (ex[-1], RED,   f"X error\n{ex[-1]:.4f} m\nt={t_end:.2f} s",   12),
            (ey[-1], GREEN, f"Y error\n{ey[-1]:.4f} m\nt={t_end:.2f} s",  -40),
        ]:
            a.plot(t_end, yv, color=color, **DOT)
            a.plot([xl, t_end], [yv, yv], color=color, **HLIN)
            _ann(a, t_end, yv, label, color, offset_x=8, offset_y=oy)

    if ts and len(ts) == len(yw):
        a     = axes["yaw"]
        xl, _ = a.get_xlim()
        te, yv = ts[-1], yw[-1]
        a.plot(te, yv, color=PURPLE, **DOT)
        a.plot([xl, te], [yv, yv], color=PURPLE, **HLIN)
        _ann(a, te, yv,
             f"Yaw\n{yv:.2f} deg\nt={te:.2f} s",
             PURPLE, offset_x=8, offset_y=0)

    if tv and len(tv) == len(vl) == len(va):
        a     = axes["vel"]
        t_end = tv[-1]
        xl, _ = a.get_xlim()
        for yv, color, label, oy in [
            (vl[-1], CYAN, f"v = {vl[-1]:.3f} m/s\nt={t_end:.2f} s",        12),
            (va[-1], PINK, f"w = {va[-1]:.3f} rad/s\nt={t_end:.2f} s",      -40),
        ]:
            a.plot(t_end, yv, color=color, **DOT)
            a.plot([xl, t_end], [yv, yv], color=color, **HLIN)
            _ann(a, t_end, yv, label, color, offset_x=8, offset_y=oy)


# ── Save figure data ──────────────────────────────────────────────────────────

def save_figure_npz(node, path: str):
    with node._lock:
        data = dict(
            planned_x = np.array(node.planned_x),
            planned_y = np.array(node.planned_y),
            actual_x  = np.array(node.actual_x),
            actual_y  = np.array(node.actual_y),
            t_stamps  = np.array(node.t_stamps),
            err_x     = np.array(node.err_x),
            err_y     = np.array(node.err_y),
            yaw_data  = np.array(node.yaw_data),
            t_vel     = np.array(node.t_vel),
            vel_lin   = np.array(node.vel_lin),
            vel_ang   = np.array(node.vel_ang),
            goal      = np.array([GOAL_X, GOAL_Y, GOAL_THETA]),
            succeeded = np.array([1 if node._nav_succeeded else 0]),
        )
    np.savez(path, **data)
    print(f"  Figure data saved -> {path}.npz")


# ── Save raw data to .txt ─────────────────────────────────────────────────────

def save_data_txt(node: NavPlotNode, path: str, test_label: str, succeeded: bool):
    with node._lock:
        px = list(node.planned_x);  py = list(node.planned_y)
        ax = list(node.actual_x);   ay = list(node.actual_y)
        ts = list(node.t_stamps)
        ex = list(node.err_x);      ey = list(node.err_y)
        yw = list(node.yaw_data)
        tv = list(node.t_vel)
        vl = list(node.vel_lin);    va = list(node.vel_ang)

    with open(path, "w") as f:
        f.write("# Nav2 Real-Time Data Log\n")
        f.write(f"# {test_label}\n")
        f.write(f"# Recorded : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# Goal     : x={GOAL_X} m  y={GOAL_Y} m  theta={GOAL_THETA} deg\n")
        f.write(f"# TF source: {TF_TARGET} -> {TF_SOURCE}\n")
        f.write(f"# Status   : {'SUCCEEDED' if succeeded else 'FAILED'}\n")
        f.write("#\n")

        f.write("# --- PLANNED PATH ---\n")
        f.write("# planned_x(m)  planned_y(m)\n")
        for x, y in zip(px, py):
            f.write(f"{x:.6f}  {y:.6f}\n")
        f.write("#\n")

        f.write("# --- ACTUAL POSE (TF) + TRACKING ERRORS ---\n")
        f.write("# time(s)  actual_x(m)  actual_y(m)  err_x(m)  err_y(m)  yaw(deg)\n")
        for i, t in enumerate(ts):
            f.write(f"{t:.4f}  "
                    f"{ax[i]:.6f}  {ay[i]:.6f}  "
                    f"{ex[i]:.6f}  {ey[i]:.6f}  "
                    f"{yw[i]:.4f}\n")
        f.write("#\n")

        f.write("# --- VELOCITY (cmd_vel) ---\n")
        f.write("# time(s)  linear_v(m/s)  angular_omega(rad/s)\n")
        for i, t in enumerate(tv):
            f.write(f"{t:.4f}  {vl[i]:.6f}  {va[i]:.6f}\n")

    print(f"  Data  saved -> {path}")


# ── Load & show a saved figure ────────────────────────────────────────────────

def load_and_show(npz_path: str):
    if not npz_path.endswith(".npz"):
        npz_path += ".npz"
    print(f"Loading figure data from {npz_path} ...")
    d = np.load(npz_path)

    gx, gy, gt = float(d["goal"][0]), float(d["goal"][1]), float(d["goal"][2])
    ok = bool(d["succeeded"][0])

    global GOAL_X, GOAL_Y, GOAL_THETA
    GOAL_X, GOAL_Y, GOAL_THETA = gx, gy, gt

    fname  = os.path.basename(npz_path)
    parts  = fname.replace("_figure.npz", "").split("_")
    label  = " ".join(p.capitalize() for p in parts)

    fig, axes, lines, status_txt = build_figure(label)

    px  = d["planned_x"];  py  = d["planned_y"]
    ax  = d["actual_x"];   ay  = d["actual_y"]
    ts  = d["t_stamps"]
    ex  = d["err_x"];      ey  = d["err_y"]
    yw  = d["yaw_data"]
    tv  = d["t_vel"]
    vl  = d["vel_lin"];    va  = d["vel_ang"]

    if len(px): lines["plan"].set_data(px, py)
    if len(ax): lines["actual"].set_data(ax, ay)
    _autoscale(axes["map"])

    if len(ts) == len(ex) == len(ey) > 0:
        lines["ex"].set_data(ts, ex)
        lines["ey"].set_data(ts, ey)
        _autoscale(axes["err"])

    if len(ts) == len(yw) > 0:
        lines["yaw"].set_data(ts, yw)
        _autoscale(axes["yaw"])

    if len(tv) == len(vl) == len(va) > 0:
        lines["vlin"].set_data(tv, vl)
        lines["vang"].set_data(tv, va)
        _autoscale(axes["vel"])

    status_txt.set_text(
        f"Loaded: {fname}  |  Status: {'SUCCEEDED' if ok else 'FAILED'}"
    )

    class _FakeNode:
        pass
    n = _FakeNode()
    n.actual_x  = list(ax);  n.actual_y = list(ay)
    n.t_stamps  = list(ts)
    n.err_x     = list(ex);  n.err_y    = list(ey)
    n.yaw_data  = list(yw)
    n.t_vel     = list(tv)
    n.vel_lin   = list(vl);  n.vel_ang  = list(va)
    n._lock     = __import__("threading").Lock()
    annotate_final_values(n, axes)

    plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--load", metavar="NPZ", default=None,
                        help="Re-open a saved figure (.npz) instead of running a new trial")
    args = parser.parse_args()

    if args.load:
        load_and_show(args.load)
        return

    test_idx   = next_test_index(SCRIPT_DIR)
    test_label = f"Test {test_idx}"
    fig_path   = os.path.join(SCRIPT_DIR, f"test_{test_idx}_figure")
    txt_path   = os.path.join(SCRIPT_DIR, f"test_{test_idx}_data.txt")

    print(f"\n{'='*55}")
    print(f"  {test_label}  (files will be saved as test_{test_idx}_*)")
    print(f"{'='*55}")

    rclpy.init()
    node = NavPlotNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\nWaiting 2 s for nodes to connect ...")
    time.sleep(2.0)

    # ── Step 1: publish initial pose so AMCL knows where robot is ─────────
    print(f"\nPublishing initial pose: x={INIT_X}, y={INIT_Y}, theta={INIT_THETA} deg")
    print("(This replaces the RViz '2D Pose Estimate' step)")
    node.publish_initial_pose()

    print("Waiting 3 s for AMCL to initialize ...")
    time.sleep(3.0)

    # Confirm AMCL has updated
    print("Checking AMCL pose after initialization ...")
    time.sleep(1.0)

    # ── Step 2: wait for action server ────────────────────────────────────
    print("\nWaiting for navigate_to_pose action server ...")
    while not node._action_client.wait_for_server(timeout_sec=1.0):
        print("  still waiting ...")
    print("Action server ready.")

    time.sleep(1.0)

    # ── Step 3: send goal ─────────────────────────────────────────────────
    print(f"\nSending goal -> x={GOAL_X}, y={GOAL_Y}, theta={GOAL_THETA} deg\n")
    node.send_goal()

    fig, axes, lines, status_txt = build_figure(test_label)
    run_live_plots(node, fig, axes, lines, status_txt)

    annotate_final_values(node, axes)
    fig.canvas.draw_idle()

    print(f"\nSaving outputs for {test_label} ...")
    save_figure_npz(node, fig_path)
    save_data_txt(node, txt_path, test_label, node._nav_succeeded)
    npz_name = f"test_{test_idx}_figure.npz"
    print(f"\nDone. Close the plot window to exit.")
    print(f"  Reload figure : python3 {os.path.basename(__file__)} --load {npz_name}")

    plt.ioff()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
import time
import threading
import math
from tabulate import tabulate


# ── Fixed waypoints: (x, y, theta_deg) ────────────────────────────────────────
WAYPOINTS = [
    (-8.0, 0.0, 180.0),
    (-8.0, -3.0, 270.0),
    (-8.0, -1.0,   0.0),
    (0.0,  0.0,   0.0),
]
NUM_LOOPS = 3


def yaw_deg_to_quat(yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quat_to_yaw_deg(qz, qw):
    return math.degrees(2.0 * math.atan2(qz, qw)) % 360.0


def euclidean(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def error_percent(measured, reference):
    if reference == 0.0:
        return 0.0
    return abs(measured - reference) / reference * 100.0


class NavGoalSender(Node):
    def __init__(self):
        super().__init__("nav_goal_sender")

        # ── Action client ──────────────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Planned path subscriber ────────────────────────────────────────
        self.path_sub = self.create_subscription(
            Path, "/plan", self.path_callback, 10
        )

        # ── AMCL pose subscriber ───────────────────────────────────────────
        amcl_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            amcl_qos,
        )

        # ── Odometry subscriber ────────────────────────────────────────────
        self.current_odom_pose = None
        self._odom_source = None
        self._odom_lock = threading.Lock()

        self._odom_wheel_sub = self.create_subscription(
            Odometry, "/odom/wheel", self._odom_wheel_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        # ── Visualization publishers ───────────────────────────────────────
        self.full_path_pub = self.create_publisher(Path, "/full_travelled_path", 10)
        self.accumulated_poses = []

        self.marker_pub = self.create_publisher(MarkerArray, "/goal_markers", 10)
        self.goal_markers = []

        self.create_timer(1.0, self._republish_viz)

        # ── State ──────────────────────────────────────────────────────────
        self.current_amcl_pose = None
        self.path_received = threading.Event()
        self.goal_done = threading.Event()
        self.goal_succeeded = False
        self.last_path_poses = 0

        # ── Results stored per loop ────────────────────────────────────────
        # results[loop_idx] = list of goal result dicts
        self.results_by_loop = [[] for _ in range(NUM_LOOPS)]

        self.get_logger().info("NavGoalSender ready.")

    # ── Odom callbacks ─────────────────────────────────────────────────────

    def _odom_wheel_cb(self, msg: Odometry):
        with self._odom_lock:
            if self._odom_source is None:
                self._odom_source = "/odom/wheel"
                self.get_logger().info("Odometry source locked: /odom/wheel")
            if self._odom_source == "/odom/wheel":
                self.current_odom_pose = msg.pose.pose

    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            if self._odom_source is None:
                self._odom_source = "/odom"
                self.get_logger().info("Odometry source locked: /odom")
            if self._odom_source == "/odom":
                self.current_odom_pose = msg.pose.pose

    # ── Other callbacks ────────────────────────────────────────────────────

    def path_callback(self, msg: Path):
        self.last_path_poses = len(msg.poses)
        if msg.poses:
            # Only record the first plan per goal — ignore replans
            if not self.path_received.is_set():
                self.accumulated_poses.extend(msg.poses)
        self.path_received.set()

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_amcl_pose = msg.pose.pose

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            self.goal_succeeded = False
            self.goal_done.set()
            return
        self.get_logger().info("Goal accepted, robot is navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        from action_msgs.msg import GoalStatus
        result = future.result()
        self.goal_succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
        if self.goal_succeeded:
            self.get_logger().info("Goal SUCCEEDED!")
        else:
            self.get_logger().warn(f"Goal failed with status: {result.status}")
        self.goal_done.set()

    # ── Visualization republish ────────────────────────────────────────────

    def _republish_viz(self):
        if self.accumulated_poses:
            full_path = Path()
            full_path.header.frame_id = "map"
            full_path.header.stamp = self.get_clock().now().to_msg()
            full_path.poses = self.accumulated_poses
            self.full_path_pub.publish(full_path)

        if not self.goal_markers:
            return
        marker_array = MarkerArray()
        for (idx, x, y, qz, qw, theta) in self.goal_markers:
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = "goal_arrows"
            arrow.id = idx
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.0
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = 0.4
            arrow.scale.y = 0.08
            arrow.scale.z = 0.08
            arrow.color.r = 1.0
            arrow.color.g = 0.4
            arrow.color.b = 0.0
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = "goal_spheres"
            sphere.id = idx + 1000
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.1
            sphere.scale.y = 0.1
            sphere.scale.z = 0.1
            sphere.color.r = 1.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

        self.marker_pub.publish(marker_array)

    # ── Send goal ──────────────────────────────────────────────────────────

    def _send_goal_once(self, x, y, qz, qw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.goal_done.clear()
        self.goal_succeeded = False

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    # ── Pose helpers ───────────────────────────────────────────────────────

    def _snapshot_pose(self, pose):
        if pose is None:
            return None
        return {
            "x":  pose.position.x,
            "y":  pose.position.y,
            "qz": pose.orientation.z,
            "qw": pose.orientation.w,
        }

    def _pose_str(self, snap):
        if snap is None:
            return "N/A"
        yaw = quat_to_yaw_deg(snap["qz"], snap["qw"])
        return f"({snap['x']:.3f}, {snap['y']:.3f}, {yaw:.1f} deg)"

    # ── Execute one goal ───────────────────────────────────────────────────

    def _execute_goal(self, loop_idx, goal_idx, x, y, theta):
        qz, qw = yaw_deg_to_quat(theta)
        marker_uid = loop_idx * 100 + goal_idx
        self.goal_markers.append((marker_uid, x, y, qz, qw, theta))

        label = f"Goal {goal_idx + 1}"
        print(f"\n  {'─'*50}")
        print(f"  Loop {loop_idx+1} | {label}  →  x={x}, y={y}, theta={theta:.1f}°")
        print(f"  {'─'*50}")

        amcl_start = self._snapshot_pose(self.current_amcl_pose)
        odom_start = self._snapshot_pose(self.current_odom_pose)

        # Send goal and wait for path plan
        self.path_received.clear()
        plan_start = time.time()
        self._send_goal_once(x, y, qz, qw)

        got_path = self.path_received.wait(timeout=10.0)
        plan_time = time.time() - plan_start
        num_poses = self.last_path_poses

        if got_path:
            print(f"  Path planned in {plan_time:.3f}s  |  Poses: {num_poses}")
        else:
            print(f"  No path received within 10s")
            num_poses = 0

        # Wait for navigation to finish
        travel_start = time.time()
        print(f"  Navigating...")
        self.goal_done.wait(timeout=300.0)
        travel_time = time.time() - travel_start

        amcl_end = self._snapshot_pose(self.current_amcl_pose)
        odom_end = self._snapshot_pose(self.current_odom_pose)

        status = "SUCCESS" if self.goal_succeeded else "FAILED"
        print(f"  ► {status}  |  Travel time: {travel_time:.2f}s")

        # If failed, wait for user to press 1 to continue
        if not self.goal_succeeded:
            while True:
                user_in = input("  Goal FAILED. Press '1' to continue to next goal: ").strip()
                if user_in == "1":
                    break

        self.results_by_loop[loop_idx].append({
            "label":       label,
            "goal_x":      x,
            "goal_y":      y,
            "goal_theta":  theta,
            "goal_qz":     qz,
            "goal_qw":     qw,
            "plan_time":   plan_time,
            "num_poses":   num_poses,
            "travel_time": travel_time,
            "status":      status,
            "amcl_start":  amcl_start,
            "amcl_end":    amcl_end,
            "odom_start":  odom_start,
            "odom_end":    odom_end,
        })

    # ── Main sequence ──────────────────────────────────────────────────────

    def run(self):
        print("\nWaiting for navigation action server...")
        self._action_client.wait_for_server()
        print("Action server ready!\n")

        time.sleep(2.0)
        odom_src = self._odom_source if self._odom_source else "not detected yet"
        print(f"Odometry source: {odom_src}")
        print(f"Fixed waypoints: {len(WAYPOINTS)} goals per loop, {NUM_LOOPS} loops total\n")

        for loop_idx in range(NUM_LOOPS):
            print(f"\n{'='*55}")
            print(f"  STARTING LOOP {loop_idx + 1} of {NUM_LOOPS}")
            print(f"{'='*55}")

            for goal_idx, (x, y, theta) in enumerate(WAYPOINTS):
                self._execute_goal(loop_idx, goal_idx, x, y, theta)

            print(f"\n  ✓ Loop {loop_idx + 1} complete.")

        print("\n\nAll loops done. Generating final report...")
        self.print_report()

    # ── Final report ───────────────────────────────────────────────────────

    def print_report(self):
        odom_src = self._odom_source if self._odom_source else "/odom (assumed)"
        sep = "=" * 80

        for loop_idx in range(NUM_LOOPS):
            loop_results = self.results_by_loop[loop_idx]
            if not loop_results:
                continue

            loop_label = f"LOOP {loop_idx + 1}"

            # ── TABLE 1: Path Generation ───────────────────────────────────
            print(f"\n{sep}")
            print(f"  {loop_label} — TABLE 1: PATH GENERATION RESULTS")
            print(sep)

            path_rows = []
            for r in loop_results:
                path_rows.append([
                    r["label"],
                    f"({r['goal_x']:.3f}, {r['goal_y']:.3f}, {r['goal_theta']:.1f} deg)",
                    r["num_poses"],
                    f"{r['plan_time']:.3f}",
                ])

            print(tabulate(
                path_rows,
                headers=["Goal", "Goal Pose (x, y, theta)", "Number of Poses", "Generation Time (s)"],
                tablefmt="fancy_grid",
            ))

            # ── TABLE 2: Pose Comparison & Error ──────────────────────────
            print(f"\n{sep}")
            print(f"  {loop_label} — TABLE 2: POSE COMPARISON & ERROR (Odometry source: {odom_src})")
            print(sep)
            print("  Euclidean distance = sqrt((x_pose - x_goal)^2 + (y_pose - y_goal)^2)")
            print("  Error (%) = |dist_pose - dist_goal| / dist_goal * 100  (vs goal distance)")
            print(sep)

            pose_rows = []
            for r in loop_results:
                gx    = r["goal_x"]
                gy    = r["goal_y"]
                gt    = r["goal_theta"]
                label = r["label"]

                goal_dist = euclidean(gx, gy, 0.0, 0.0)

                # Goal Pose row
                pose_rows.append([
                    label, "Goal Pose",
                    f"{gx:.3f}", f"{gy:.3f}", f"{gt:.1f}",
                    f"{goal_dist:.4f}", "—",
                ])

                # AMCL Pose
                ae = r["amcl_end"]
                if ae is not None:
                    ax     = ae["x"]
                    ay     = ae["y"]
                    at     = quat_to_yaw_deg(ae["qz"], ae["qw"])
                    a_dist = euclidean(ax, ay, 0.0, 0.0)
                    a_edist = euclidean(ax, ay, gx, gy)
                    a_epct = error_percent(a_dist, goal_dist)
                    pose_rows.append([
                        label, "AMCL Pose",
                        f"{ax:.3f}", f"{ay:.3f}", f"{at:.1f}",
                        f"{a_dist:.4f}", f"{a_epct:.2f}%",
                    ])
                    pose_rows.append([
                        label, "AMCL Error vs Goal",
                        f"{abs(ax-gx):.4f}", f"{abs(ay-gy):.4f}", f"{abs(at-gt):.2f}",
                        f"{a_edist:.4f}", f"{a_epct:.2f}%",
                    ])
                else:
                    pose_rows.append([label, "AMCL Pose",          "N/A","N/A","N/A","N/A","N/A"])
                    pose_rows.append([label, "AMCL Error vs Goal",  "N/A","N/A","N/A","N/A","N/A"])

                # Odometry Pose
                oe = r["odom_end"]
                if oe is not None:
                    ox     = oe["x"]
                    oy     = oe["y"]
                    ot     = quat_to_yaw_deg(oe["qz"], oe["qw"])
                    o_dist = euclidean(ox, oy, 0.0, 0.0)
                    o_edist = euclidean(ox, oy, gx, gy)
                    o_epct = error_percent(o_dist, goal_dist)
                    pose_rows.append([
                        label, "Odometry Pose",
                        f"{ox:.3f}", f"{oy:.3f}", f"{ot:.1f}",
                        f"{o_dist:.4f}", f"{o_epct:.2f}%",
                    ])
                    pose_rows.append([
                        label, "Odometry Error vs Goal",
                        f"{abs(ox-gx):.4f}", f"{abs(oy-gy):.4f}", f"{abs(ot-gt):.2f}",
                        f"{o_edist:.4f}", f"{o_epct:.2f}%",
                    ])
                else:
                    pose_rows.append([label, "Odometry Pose",          "N/A","N/A","N/A","N/A","N/A"])
                    pose_rows.append([label, "Odometry Error vs Goal",  "N/A","N/A","N/A","N/A","N/A"])

                # Blank separator row
                pose_rows.append(["", "", "", "", "", "", ""])

            print(tabulate(
                pose_rows,
                headers=["Goal", "Pose Type", "x (m)", "y (m)", "theta (deg)", "Euclidean Dist (m)", "Error (%)"],
                tablefmt="fancy_grid",
            ))

            # ── TABLE 3: Travel Summary ────────────────────────────────────
            print(f"\n{sep}")
            print(f"  {loop_label} — TABLE 3: TRAVEL SUMMARY")
            print(sep)

            travel_rows = []
            for r in loop_results:
                travel_rows.append([
                    r["label"],
                    self._pose_str(r["amcl_start"]),
                    self._pose_str(r["amcl_end"]),
                    f"{r['travel_time']:.2f}",
                    r["status"],
                ])

            print(tabulate(
                travel_rows,
                headers=["Goal", "AMCL Start Pose", "AMCL End Pose", "Travel Time (s)", "Status"],
                tablefmt="fancy_grid",
            ))

            print(f"\n{sep}\n")


def main():
    rclpy.init()
    node = NavGoalSender()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\nWaiting 2s for nodes to connect...")
    time.sleep(2.0)

    try:
        node.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        if any(node.results_by_loop):
            print("Generating partial report...")
            node.print_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
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


def get_goal_input(idx):
    print(f"\n  Enter coordinates for Goal {idx + 1}:")
    print("  (Type 'q' at any prompt to finish and print results)")
    while True:
        try:
            x_in = input("    x (m)         : ").strip().lower()
            if x_in == "q":
                return None
            x = float(x_in)

            y_in = input("    y (m)         : ").strip().lower()
            if y_in == "q":
                return None
            y = float(y_in)

            t_in = input("    theta (0-360°): ").strip().lower()
            if t_in == "q":
                return None
            theta = float(t_in)

            qz, qw = yaw_deg_to_quat(theta)
            print(f"  Goal {idx+1} set → x={x}, y={y}, theta={theta:.1f} deg")
            return x, y, qz, qw, theta
        except ValueError:
            print("  Invalid input, please enter numbers only.")


class NavGoalSender(Node):
    def __init__(self):
        super().__init__("nav_goal_sender")

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.path_sub = self.create_subscription(
            Path, "/plan", self.path_callback, 10
        )

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

        self.current_odom_pose = None
        self._odom_source = None
        self._odom_lock = threading.Lock()

        self._odom_wheel_sub = self.create_subscription(
            Odometry, "/odom/wheel", self._odom_wheel_cb, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        self.full_path_pub = self.create_publisher(Path, "/full_travelled_path", 10)
        self.accumulated_poses = []

        self.marker_pub = self.create_publisher(MarkerArray, "/goal_markers", 10)
        self.goal_markers = []

        self.create_timer(1.0, self._republish_viz)

        self.current_amcl_pose = None
        self.path_received = threading.Event()
        self.goal_done = threading.Event()
        self.goal_succeeded = False
        self.last_path_poses = 0

        # Each entry: {"poses": int, "time_into_travel": float}
        # time_into_travel = seconds since travel started when this plan arrived
        self._current_goal_plans = []
        self._replan_lock = threading.Lock()
        self._travel_start_time = None  # set just before goal_done.wait()

        self.results = []

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

    # ── Path callback — record ALL plans + timestamp ───────────────────────

    def path_callback(self, msg: Path):
        n_poses = len(msg.poses)
        self.last_path_poses = n_poses
        if msg.poses:
            self.accumulated_poses.extend(msg.poses)
            with self._replan_lock:
                # Record elapsed time since travel started (0.0 for first plan)
                elapsed = 0.0
                if self._travel_start_time is not None:
                    elapsed = time.time() - self._travel_start_time
                self._current_goal_plans.append({
                    "poses":            n_poses,
                    "time_into_travel": elapsed,
                })
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
            sphere.id = idx + 100
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
            "x":   pose.position.x,
            "y":   pose.position.y,
            "qz":  pose.orientation.z,
            "qw":  pose.orientation.w,
        }

    def _pose_str(self, snap):
        if snap is None:
            return "N/A"
        yaw = quat_to_yaw_deg(snap["qz"], snap["qw"])
        return f"({snap['x']:.3f}, {snap['y']:.3f}, {yaw:.1f} deg)"

    # ── Main sequence ──────────────────────────────────────────────────────

    def run(self):
        print("\nWaiting for navigation action server...")
        self._action_client.wait_for_server()
        print("Action server ready!\n")

        time.sleep(2.0)
        odom_src = self._odom_source if self._odom_source else "not detected yet"
        print(f"Odometry source: {odom_src}\n")

        idx = 0
        while True:
            goal_data = get_goal_input(idx)

            if goal_data is None:
                print("\nGenerating final report...")
                break

            x, y, qz, qw, theta = goal_data
            point_label = f"Goal {idx + 1}"

            print(f"\n{'='*55}")
            print(f"  {point_label}  ->  x={x}, y={y}, theta={theta:.1f} deg")
            print(f"{'='*55}")

            self.goal_markers.append((idx, x, y, qz, qw, theta))

            # Reset replan tracker and travel timer for this goal
            with self._replan_lock:
                self._current_goal_plans = []
                self._travel_start_time = None

            amcl_start = self._snapshot_pose(self.current_amcl_pose)
            odom_start = self._snapshot_pose(self.current_odom_pose)

            self.path_received.clear()
            plan_start = time.time()
            self._send_goal_once(x, y, qz, qw)

            got_path = self.path_received.wait(timeout=10.0)
            plan_time = time.time() - plan_start
            num_poses = self.last_path_poses

            if got_path:
                print(f"  Path planned in {plan_time:.3f} s  |  Poses: {num_poses}")
            else:
                print(f"  No path received within 10 s")
                num_poses = 0

            # Set travel start time so replans can record elapsed time
            travel_start = time.time()
            with self._replan_lock:
                self._travel_start_time = travel_start

            print(f"  Robot navigating to {point_label}...")
            self.goal_done.wait(timeout=300.0)
            travel_time = time.time() - travel_start

            amcl_end = self._snapshot_pose(self.current_amcl_pose)
            odom_end = self._snapshot_pose(self.current_odom_pose)

            with self._replan_lock:
                goal_plans = list(self._current_goal_plans)

            status = "SUCCESS" if self.goal_succeeded else "FAILED"
            n_replans = max(0, len(goal_plans) - 1)
            print(f"  {status} — Travel time: {travel_time:.2f} s  |  Replans: {n_replans}")

            self.results.append({
                "label":       point_label,
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
                "goal_plans":  goal_plans,
            })

            idx += 1

            cmd = input("\n  Press ENTER for next goal or 'q' to finish: ").strip().lower()
            if cmd == "q":
                print("\nGenerating final report...")
                break

        if self.results:
            self.print_report()
        else:
            print("No goals were executed.")

    # ── Final report ───────────────────────────────────────────────────────

    def print_report(self):
        odom_src = self._odom_source if self._odom_source else "/odom (assumed)"
        sep = "=" * 80

        # ── TABLE 1: Path Generation ───────────────────────────────────────
        print(f"\n{sep}")
        print("  TABLE 1 — PATH GENERATION RESULTS")
        print(sep)

        path_rows = []
        for r in self.results:
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

        # ── TABLE 2: Pose Comparison & Error ──────────────────────────────
        print(f"\n{sep}")
        print(f"  TABLE 2 — POSE COMPARISON & ERROR (Odometry source: {odom_src})")
        print(sep)
        print("  Euclidean distance = sqrt((x_pose - x_goal)^2 + (y_pose - y_goal)^2)")
        print("  Error (%) = |dist_pose - dist_goal| / dist_goal * 100  (vs goal distance)")
        print(sep)

        pose_rows = []
        for r in self.results:
            gx    = r["goal_x"]
            gy    = r["goal_y"]
            gt    = r["goal_theta"]
            label = r["label"]

            goal_dist = euclidean(gx, gy, 0.0, 0.0)

            pose_rows.append([label, "Goal Pose",
                f"{gx:.3f}", f"{gy:.3f}", f"{gt:.1f}", f"{goal_dist:.4f}", "—"])

            ae = r["amcl_end"]
            if ae is not None:
                ax      = ae["x"];  ay = ae["y"]
                at      = quat_to_yaw_deg(ae["qz"], ae["qw"])
                a_dist  = euclidean(ax, ay, 0.0, 0.0)
                a_edist = euclidean(ax, ay, gx, gy)
                a_epct  = error_percent(a_dist, goal_dist)
                pose_rows.append([label, "AMCL Pose",
                    f"{ax:.3f}", f"{ay:.3f}", f"{at:.1f}", f"{a_dist:.4f}", f"{a_epct:.2f}%"])
                pose_rows.append([label, "AMCL Error vs Goal",
                    f"{abs(ax-gx):.4f}", f"{abs(ay-gy):.4f}", f"{abs(at-gt):.2f}",
                    f"{a_edist:.4f}", f"{a_epct:.2f}%"])
            else:
                pose_rows.append([label, "AMCL Pose",         "N/A","N/A","N/A","N/A","N/A"])
                pose_rows.append([label, "AMCL Error vs Goal", "N/A","N/A","N/A","N/A","N/A"])

            oe = r["odom_end"]
            if oe is not None:
                ox      = oe["x"];  oy = oe["y"]
                ot      = quat_to_yaw_deg(oe["qz"], oe["qw"])
                o_dist  = euclidean(ox, oy, 0.0, 0.0)
                o_edist = euclidean(ox, oy, gx, gy)
                o_epct  = error_percent(o_dist, goal_dist)
                pose_rows.append([label, "Odometry Pose",
                    f"{ox:.3f}", f"{oy:.3f}", f"{ot:.1f}", f"{o_dist:.4f}", f"{o_epct:.2f}%"])
                pose_rows.append([label, "Odometry Error vs Goal",
                    f"{abs(ox-gx):.4f}", f"{abs(oy-gy):.4f}", f"{abs(ot-gt):.2f}",
                    f"{o_edist:.4f}", f"{o_epct:.2f}%"])
            else:
                pose_rows.append([label, "Odometry Pose",          "N/A","N/A","N/A","N/A","N/A"])
                pose_rows.append([label, "Odometry Error vs Goal",  "N/A","N/A","N/A","N/A","N/A"])

            pose_rows.append(["", "", "", "", "", "", ""])

        print(tabulate(
            pose_rows,
            headers=["Goal", "Pose Type", "x (m)", "y (m)", "theta (deg)", "Euclidean Dist (m)", "Error (%)"],
            tablefmt="fancy_grid",
        ))

        # ── TABLE 3: Travel Summary ────────────────────────────────────────
        print(f"\n{sep}")
        print("  TABLE 3 — TRAVEL SUMMARY")
        print(sep)

        travel_rows = []
        for r in self.results:
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

        # ── TABLE 4: Replan Summary ────────────────────────────────────────
        print(f"\n{sep}")
        print("  TABLE 4 — REPLAN SUMMARY")
        print(sep)
        print("  Plan #1 = initial plan. Plan #2 onwards = replans triggered during navigation.")
        print("  Time (s) = seconds into travel when this plan was received (0.00 = initial).")
        print(sep)

        replan_rows = []
        for r in self.results:
            plans  = r["goal_plans"]
            label  = r["label"]
            n_replans = max(0, len(plans) - 1)

            if not plans:
                replan_rows.append([label, "N/A", "—", "N/A", "N/A"])
                replan_rows.append(["", "", "", "", ""])
                continue

            # Summary row for this goal
            replan_rows.append([
                label,
                f"{len(plans)} total ({n_replans} replan{'s' if n_replans != 1 else ''})",
                "—", "—", "—",
            ])

            # One row per plan with poses + time
            for i, p in enumerate(plans):
                plan_type  = "Initial Plan" if i == 0 else f"Replan #{i}"
                type_label = "✓ Initial"   if i == 0 else f"↺ Replan #{i}"
                replan_rows.append([
                    "",
                    f"Plan #{i+1} — {plan_type}",
                    f"{p['time_into_travel']:.3f} s",
                    p["poses"],
                    type_label,
                ])

            replan_rows.append(["", "", "", "", ""])

        print(tabulate(
            replan_rows,
            headers=["Goal", "Plan", "Time into Travel (s)", "Poses", "Type"],
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
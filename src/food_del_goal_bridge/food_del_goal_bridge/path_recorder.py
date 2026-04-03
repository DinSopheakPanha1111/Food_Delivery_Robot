#!/usr/bin/env python3
"""
path_recorder.py

Subscribes to /odom, transforms each pose into the map frame via TF,
and publishes the accumulated trajectory as nav_msgs/Path on /actual_path.

Add a Path display in RViz on /actual_path to see where the robot has been.
The path is cleared whenever a new /goal_pose arrives so each run starts fresh.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers transform for PoseStamped


class PathRecorderNode(Node):

    # Minimum distance (m) the robot must move before a new point is recorded.
    # Prevents filling the path with thousands of identical poses when stationary.
    MIN_DIST = 0.05

    def __init__(self):
        super().__init__('path_recorder')

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._path = Path()
        self._path.header.frame_id = 'map'

        self._last_x: float | None = None
        self._last_y: float | None = None

        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)

        # TRANSIENT_LOCAL: any subscriber that joins late (e.g. RViz Path display
        # added after launch) immediately receives the last published message.
        _qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._path_pub = self.create_publisher(Path, '/actual_path', _qos)

        self.get_logger().info(
            'PathRecorder started — publishing actual trajectory to /actual_path. '
            'Add a Path display in RViz on /actual_path.')

    # ─────────────────────────────────────────────────────────────────────────

    def _goal_callback(self, _msg: PoseStamped) -> None:
        """Clear the recorded path whenever a new goal is set."""
        self._path.poses.clear()
        self._last_x = None
        self._last_y = None
        self._path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(self._path)  # push empty path so RViz clears immediately
        self.get_logger().info('New goal received — actual_path cleared.')

    def _odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Skip if the robot hasn't moved enough
        if self._last_x is not None:
            dist = math.hypot(x - self._last_x, y - self._last_y)
            if dist < self.MIN_DIST:
                return

        # Transform odom pose → map frame
        pose_odom = PoseStamped()
        pose_odom.header = msg.header
        pose_odom.pose   = msg.pose.pose

        try:
            pose_map = self._tf_buffer.transform(pose_odom, 'map')
        except Exception as e:
            self.get_logger().debug(f'TF transform failed: {e}')
            return

        self._last_x = x
        self._last_y = y

        now = self.get_clock().now().to_msg()
        pose_map.header.stamp  = now
        self._path.header.stamp = now
        self._path.poses.append(pose_map)

        self._path_pub.publish(self._path)


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PathRecorderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

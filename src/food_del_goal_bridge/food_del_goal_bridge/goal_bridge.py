#!/usr/bin/env python3
"""
goal_bridge.py

Subscribes to /goal_pose (PoseStamped), calls the Nav2 planner_server's
ComputePathToPose action, and publishes the resulting path to /plan so
the custom dwb_controller can consume it unchanged.

Topic flow:
  /goal_pose  ──►  [this node]  ──action──►  planner_server
                       │
                  publishes /plan  ──►  dwb_controller
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose


class GoalBridgeNode(Node):

    def __init__(self):
        super().__init__('goal_bridge')

        self._action_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose')

        self._goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)

        self._path_pub = self.create_publisher(Path, '/plan', 10)

        # Track in-flight goal so we can cancel it when a new /goal_pose arrives
        self._active_goal_handle: ClientGoalHandle | None = None

        self.get_logger().info(
            'GoalBridge started — waiting for /goal_pose and planner_server.')

    # ─────────────────────────────────────────────────────────────────────────
    #  /goal_pose callback
    # ─────────────────────────────────────────────────────────────────────────

    def _goal_callback(self, goal_msg: PoseStamped) -> None:
        # Cancel any previously running plan request
        if self._active_goal_handle is not None:
            self.get_logger().info('Cancelling previous plan request.')
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'planner_server action not available after 5 s — is it running?')
            return

        action_goal = ComputePathToPose.Goal()
        action_goal.goal = goal_msg
        action_goal.planner_id = 'GridBased'
        # use_start=False → planner_server looks up the robot pose via TF itself
        action_goal.use_start = False

        self.get_logger().info(
            f'Requesting path to goal: '
            f'({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})')

        send_future = self._action_client.send_goal_async(action_goal)
        send_future.add_done_callback(self._goal_accepted_callback)

    # ─────────────────────────────────────────────────────────────────────────
    #  Action callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _goal_accepted_callback(self, future) -> None:
        goal_handle: ClientGoalHandle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Plan request rejected by planner_server.')
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info('Plan request accepted — waiting for result...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        self._active_goal_handle = None

        result = future.result().result
        path: Path = result.path

        if not path.poses:
            self.get_logger().warn('Planner returned an empty path.')
            return

        self._path_pub.publish(path)
        self.get_logger().info(
            f'Path published to /plan — {len(path.poses)} waypoints.')


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GoalBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

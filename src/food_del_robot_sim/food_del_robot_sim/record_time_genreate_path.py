import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import time
import math
import numpy as np

class PlannerTimer(Node):
    def __init__(self):
        super().__init__('planner_timer')
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def send_goal(self, x, y, qz, qw):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.z = qz
        goal.goal.pose.orientation.w = qw

        self._client.wait_for_server()
        start = time.time()
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        elapsed = time.time() - start

        result = result_future.result().result
        path = result.path.poses

        planning_time_ms = elapsed * 1000.0

        path_length = 0.0
        for i in range(1, len(path)):
            dx = path[i].pose.position.x - path[i-1].pose.position.x
            dy = path[i].pose.position.y - path[i-1].pose.position.y
            path_length += math.sqrt(dx**2 + dy**2)

        heading_changes = []
        for i in range(1, len(path) - 1):
            dx1 = path[i].pose.position.x - path[i-1].pose.position.x
            dy1 = path[i].pose.position.y - path[i-1].pose.position.y
            dx2 = path[i+1].pose.position.x - path[i].pose.position.x
            dy2 = path[i+1].pose.position.y - path[i].pose.position.y
            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            diff = abs(angle2 - angle1)
            if diff > math.pi:
                diff = 2 * math.pi - diff
            heading_changes.append(diff)
        smoothness = float(np.mean(heading_changes)) if heading_changes else float('nan')

        return {
            'goal_x': x,
            'goal_y': y,
            'planning_time_ms': planning_time_ms,
            'path_length_m': path_length,
            'smoothness_rad': smoothness,
            'num_poses': len(path)
        }


def print_table(results):
    header = f"{'Goal (x,y)':<20} {'Time(ms)':<12} {'Length(m)':<12} {'Smoothness(rad)':<18} {'Poses'}"
    print("\n" + "="*80)
    print("SMAC Planner Evaluation Results")
    print("="*80)
    print(header)
    print("-"*80)
    for r in results:
        goal = f"({r['goal_x']:.2f},{r['goal_y']:.2f})"
        smoothness = f"{r['smoothness_rad']:.4f}" if not math.isnan(r['smoothness_rad']) else "N/A"
        print(f"{goal:<20} {r['planning_time_ms']:<12.2f} {r['path_length_m']:<12.3f} {smoothness:<18} {r['num_poses']}")
    print("="*80)


def main():
    rclpy.init()
    node = PlannerTimer()

    # Goals from RViz logs
    test_goals = [
        (-0.541738, 5.53287,  -0.709948, 0.704254),   # Goal 1
        ( 2.34289, -5.65144,   0.708482, 0.705729),   # Goal 2
        (-3.57568, -5.60489,   0.704196, 0.710005),   # Goal 3
    ]

    results = []
    for i, (x, y, qz, qw) in enumerate(test_goals):
        print(f"Sending goal: ({x:.3f}, {y:.3f})")
        result = node.send_goal(x, y, qz, qw)
        results.append(result)
        if i < len(test_goals) - 1:
            time.sleep(1.0)

    print_table(results)
    rclpy.shutdown()

main()
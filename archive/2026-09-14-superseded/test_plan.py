#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import time


class PlanTest(Node):
    def __init__(self):
        super().__init__('plan_test')
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def test(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            print("compute_path_to_pose server not available")
            return

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 1.554
        goal.pose.position.y = 0.097
        goal.pose.orientation.w = 1.0

        msg = ComputePathToPose.Goal()
        msg.pose = goal

        future = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            print("goal rejected")
            return

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        result = result_future.result()
        print(f"status: {result.status}")
        path = result.result.path
        print(f"path has {len(path.poses)} poses")
        if len(path.poses) > 0:
            first = path.poses[0].pose.position
            last = path.poses[-1].pose.position
            print(f"first: ({first.x:.3f}, {first.y:.3f})")
            print(f"last:  ({last.x:.3f}, {last.y:.3f})")


def main():
    rclpy.init()
    node = PlanTest()
    node.test()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import time


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait_for_tf(self, timeout=10.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
                return self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        return None

    def send(self, forward_m):
        tf = self.wait_for_tf()
        if tf is None:
            print("Could not get odom->base_link transform")
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        gx = x + forward_m * math.cos(yaw)
        gy = y + forward_m * math.sin(yaw)
        print(f"current pose (odom): x={x:.3f} y={y:.3f} yaw={yaw:.3f}")
        print(f"goal pose   (odom): x={gx:.3f} y={gy:.3f} yaw={yaw:.3f}")

        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation = q

        if not self.client.wait_for_server(timeout_sec=10.0):
            print("navigate_to_pose action server not available")
            return

        msg = NavigateToPose.Goal()
        msg.pose = goal
        future = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            print("Goal rejected or no response")
            return
        print("Goal accepted, navigating...")

        result_future = handle.get_result_async()
        start = time.time()
        while time.time() - start < 40.0:
            rclpy.spin_once(self, timeout_sec=0.5)
            if result_future.done():
                break
        if result_future.done():
            result = result_future.result()
            print(f"Result status: {result.status}")
        else:
            print("Timed out waiting for result (still may be navigating)")


def main():
    import sys
    dist = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    rclpy.init()
    node = GoalSender()
    node.send(dist)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

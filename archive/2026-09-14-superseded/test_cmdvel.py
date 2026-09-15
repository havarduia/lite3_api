#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class Test(Node):
    def __init__(self):
        super().__init__('cmdvel_test')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose = None
        self.create_subscription(Odometry, 'leg_odom2', self.odom_cb, 10)

    def odom_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = Test()
    node.spin_for(1.0)
    start_pose = node.pose
    print(f"start pose: {start_pose}")

    # 2 seconds of slow forward velocity
    twist = Twist()
    twist.linear.x = 0.1
    end = time.time() + 2.0
    while time.time() < end:
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)

    # explicit stop, published several times to be sure
    stop = Twist()
    for _ in range(10):
        node.pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.spin_for(1.0)
    end_pose = node.pose
    print(f"end pose: {end_pose}")
    if start_pose and end_pose:
        dist = math.hypot(end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        print(f"distance moved: {dist:.3f} m")

    rclpy.shutdown()


if __name__ == '__main__':
    main()

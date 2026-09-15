#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import time

FIELDS = ["basic", "gait", "policy", "motion", "task", "need_move", "zero_pos"]

class Watch(Node):
    def __init__(self):
        super().__init__('watch_state')
        self.last = None
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Int32MultiArray, '/robot_state_debug', self.cb, 10)

    def cb(self, msg):
        data = list(msg.data)
        if data != self.last:
            print(f"[{time.time():.2f}] state changed: {dict(zip(FIELDS, data))}")
            self.last = data

def main():
    rclpy.init()
    node = Watch()
    print("--- watching for 3s at rest ---")
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("--- sending cmd_vel 0.3 m/s for 3s ---")
    twist = Twist()
    twist.linear.x = 0.3
    end = time.time() + 3
    while time.time() < end:
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)

    print("--- stopping, watching for 3s more ---")
    stop = Twist()
    for _ in range(10):
        node.pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    end = time.time() + 3
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

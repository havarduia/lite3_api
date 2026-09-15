#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
import time

class Check(Node):
    def __init__(self):
        super().__init__('camera_check')
        self.pc_count = 0
        self.alive_count = 0
        self.last_alive = None
        self.create_subscription(PointCloud2, '/camera/depth/color/points', self.pc_cb, 10)
        self.create_subscription(Bool, '/sensor_status/realsense_isalive', self.alive_cb, 10)

    def pc_cb(self, msg):
        self.pc_count += 1

    def alive_cb(self, msg):
        self.alive_count += 1
        self.last_alive = msg.data

def main():
    rclpy.init()
    node = Check()
    start = time.time()
    while time.time() - start < 20:
        rclpy.spin_once(node, timeout_sec=0.5)
    print(f"pointcloud messages received: {node.pc_count}")
    print(f"isalive messages received: {node.alive_count}, last value: {node.last_alive}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

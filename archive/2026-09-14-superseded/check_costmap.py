#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time

class Check(Node):
    def __init__(self):
        super().__init__('costmap_check')
        self.msg = None
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.cb, 10)

    def cb(self, msg):
        self.msg = msg

def main():
    rclpy.init()
    node = Check()
    start = time.time()
    while time.time() - start < 15:
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.msg is None:
        print("no costmap message received")
    else:
        data = node.msg.data
        occupied = sum(1 for v in data if v > 50)
        unknown = sum(1 for v in data if v < 0)
        free = len(data) - occupied - unknown
        print(f"cells: {len(data)} total, {occupied} occupied(>50), {free} free, {unknown} unknown")
        print(f"resolution: {node.msg.info.resolution}, size: {node.msg.info.width}x{node.msg.info.height}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

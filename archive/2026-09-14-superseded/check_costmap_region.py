#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time

class Check(Node):
    def __init__(self):
        super().__init__('costmap_region_check')
        self.msg = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.cb, 10)

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
        rclpy.shutdown()
        return
    info = node.msg.info
    data = node.msg.data
    # Robot is roughly at (0.55, 0.04) in odom, costmap origin at info.origin
    for x in [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5]:
        for y in [-0.1, 0.0, 0.1]:
            gx = int((x - info.origin.position.x) / info.resolution)
            gy = int((y - info.origin.position.y) / info.resolution)
            if 0 <= gx < info.width and 0 <= gy < info.height:
                val = data[gy * info.width + gx]
                print(f"world ({x:.1f},{y:.1f}) -> cell ({gx},{gy}) = {val}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Why did NavFn fail? Look at the global costmap around the robot and along
the straight line to the goal.

Prints the costmap extent, the robot's own cell cost (a lethal robot cell makes
planning impossible), and a profile of costs from the robot toward the goal.
"""
import os
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


class P(Node):
    def __init__(self):
        super().__init__('costmap_probe')
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.RELIABLE
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.grid = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._cb, q)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)

    def _cb(self, m):
        self.grid = m


def main():
    rclpy.init()
    n = P()
    t0 = time.time()
    while (n.grid is None) and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.grid is None:
        print('no global costmap received')
        rclpy.shutdown()
        return

    g = n.grid
    info = g.info
    print(f'costmap {info.width} x {info.height} cells @ {info.resolution:.3f} m '
          f'= {info.width*info.resolution:.1f} x {info.height*info.resolution:.1f} m')
    print(f'origin: ({info.origin.position.x:.2f}, {info.origin.position.y:.2f})')

    # robot pose
    rx = ry = None
    t0 = time.time()
    while time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf = n.buf.lookup_transform('odom', 'base_link', rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
            break
        except Exception:
            continue
    if rx is None:
        print('no odom->base_link')
        rclpy.shutdown()
        return
    print(f'robot at ({rx:.3f}, {ry:.3f})')

    def cost_at(x, y):
        cx = int((x - info.origin.position.x) / info.resolution)
        cy = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= cx < info.width and 0 <= cy < info.height):
            return None
        return g.data[cy * info.width + cx]

    occ = sum(1 for v in g.data if v > 50)
    unk = sum(1 for v in g.data if v < 0)
    print(f'cells: {len(g.data)} total, {occ} occupied(>50), {unk} unknown')

    rc = cost_at(rx, ry)
    print(f'\nrobot cell cost: {rc}  '
          f'{"<-- LETHAL, planning impossible" if rc is not None and rc >= 99 else ""}')

    print('\ncost along +x from robot toward goal:')
    for d in [i * 0.25 for i in range(0, 15)]:
        c = cost_at(rx + d, ry)
        if c is None:
            print(f'  +{d:4.2f} m  OUTSIDE COSTMAP')
        else:
            mark = ' LETHAL' if c >= 99 else (' inflated' if c > 50 else '')
            print(f'  +{d:4.2f} m  cost {c:4d}{mark}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

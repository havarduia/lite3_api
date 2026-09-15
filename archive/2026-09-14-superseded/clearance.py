#!/usr/bin/env python3
"""Report the nearest obstacle ahead from the depth cloud, before commanding motion.

Camera frame convention for the D435i: +z forward, +x right, +y down.
Ignores the floor and points outside a narrow corridor in front of the robot.
"""
import os
import struct
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

# Real mounting, from voa_launch.py base2frontcamera_broadcaster:
#   xyz = 0.25489, 0, 0.07249   rpy = 0, 0.34907, 0   (base_link -> camera_link)
# i.e. the camera is pitched 20 degrees NOSE DOWN. Without accounting for that
# the floor fills the view and reads as a wall.
CAM_PITCH = 0.34907          # rad, nose down
CAM_X = 0.25489              # m forward of base_link
CAM_Z = 0.07249              # m above base_link
STAND_HEIGHT = 0.33          # m, base_link above ground (conf/deeprcs.json)
FLOOR_MARGIN = 0.08          # m above the floor before it counts as an obstacle

CORRIDOR_HALF_WIDTH = 0.25   # m either side of centre
MIN_VALID = 0.15             # m, ignore closer than this (noise)


COS_P = math.cos(CAM_PITCH)
SIN_P = math.sin(CAM_PITCH)


class C(Node):
    def __init__(self):
        super().__init__('clearance')
        q = QoSProfile(depth=2)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.msg = None
        self.create_subscription(PointCloud2, '/camera/depth/color/points',
                                 self._cb, q)

    def _cb(self, m):
        self.msg = m


def analyse(m):
    fields = {f.name: (f.offset, f.datatype) for f in m.fields}
    if not {'x', 'y', 'z'} <= set(fields):
        return None
    ox = fields['x'][0]
    oy = fields['y'][0]
    oz = fields['z'][0]
    step = m.point_step
    data = m.data
    n = m.width * m.height

    nearest = None
    bins = {'left': None, 'centre': None, 'right': None}
    for i in range(0, n, 3):          # subsample for speed
        base = i * step
        if base + step > len(data):
            break
        xo = struct.unpack_from('<f', data, base + ox)[0]
        yo = struct.unpack_from('<f', data, base + oy)[0]
        zo = struct.unpack_from('<f', data, base + oz)[0]
        if not (zo == zo) or zo < MIN_VALID or zo > 4.0:
            continue
        # optical frame (x right, y down, z fwd) -> camera_link (x fwd, y left, z up)
        xc, yc, zc = zo, -xo, -yo
        # apply the 20 deg nose-down pitch, then the mount offset
        xb = xc * COS_P + zc * SIN_P + CAM_X
        yb = yc
        zb = -xc * SIN_P + zc * COS_P + CAM_Z
        height = zb + STAND_HEIGHT          # height above the ground plane
        if height < FLOOR_MARGIN:           # floor
            continue
        if xb < MIN_VALID:
            continue
        if abs(yb) <= CORRIDOR_HALF_WIDTH:
            if nearest is None or xb < nearest:
                nearest = xb
        key = 'left' if yb > CORRIDOR_HALF_WIDTH else \
              'right' if yb < -CORRIDOR_HALF_WIDTH else 'centre'
        if bins[key] is None or xb < bins[key]:
            bins[key] = xb
    return nearest, bins


def main():
    rclpy.init()
    n = C()
    t0 = time.time()
    while n.msg is None and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.msg is None:
        print('no point cloud - is realsense running?')
        rclpy.shutdown()
        return

    res = analyse(n.msg)
    if res is None:
        print('cloud has no xyz fields')
        rclpy.shutdown()
        return
    nearest, bins = res
    print(f'cloud: {n.msg.width * n.msg.height} points')
    print(f'nearest in {CORRIDOR_HALF_WIDTH*2:.1f} m corridor ahead: '
          f'{"none within 4 m" if nearest is None else f"{nearest:.2f} m"}')
    for k in ('left', 'centre', 'right'):
        v = bins[k]
        print(f'  {k:<7} {"clear" if v is None else f"{v:.2f} m"}')
    print()
    if nearest is None or nearest > 1.5:
        print('>>> PATH LOOKS CLEAR')
    else:
        print(f'>>> OBSTACLE at {nearest:.2f} m ahead')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

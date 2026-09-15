#!/usr/bin/env python3
"""Angular free-space profile from the depth cloud, in the robot's base frame.

clearance.py reports only the nearest point in three coarse bins, which cannot
tell a chair leg with open space behind it from a solid wall. This bins the
nearest range by BEARING so gaps are visible.

Camera mounting from voa/launch/voa_launch.py base2frontcamera_broadcaster:
  xyz 0.25489 0 0.07249, rpy 0 0.34907 0  (base_link -> camera_link)
i.e. pitched 20 degrees nose down.
"""
import math
import os
import struct
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

CAM_PITCH = 0.34907
CAM_X = 0.25489
CAM_Z = 0.07249
STAND_HEIGHT = 0.33
FLOOR_MARGIN = 0.08
CEILING = 0.60          # ignore anything above the robot's back
MIN_VALID = 0.15
MAX_RANGE = 4.0

BIN_DEG = 5
FOV_DEG = 45            # +/- from straight ahead

COS_P = math.cos(CAM_PITCH)
SIN_P = math.sin(CAM_PITCH)


class S(Node):
    def __init__(self):
        super().__init__('scan')
        q = QoSProfile(depth=2)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.msg = None
        self.create_subscription(PointCloud2, '/camera/depth/color/points',
                                 self._cb, q)

    def _cb(self, m):
        self.msg = m


def profile(m):
    f = {fd.name: fd.offset for fd in m.fields}
    if not {'x', 'y', 'z'} <= set(f):
        return None
    ox, oy, oz = f['x'], f['y'], f['z']
    step, data = m.point_step, m.data
    n = m.width * m.height

    nbins = (2 * FOV_DEG) // BIN_DEG
    bins = [None] * nbins
    for i in range(0, n, 2):
        base = i * step
        if base + step > len(data):
            break
        xo = struct.unpack_from('<f', data, base + ox)[0]
        yo = struct.unpack_from('<f', data, base + oy)[0]
        zo = struct.unpack_from('<f', data, base + oz)[0]
        if not (zo == zo) or zo < MIN_VALID or zo > MAX_RANGE:
            continue
        xc, yc, zc = zo, -xo, -yo
        xb = xc * COS_P + zc * SIN_P + CAM_X
        yb = yc
        zb = -xc * SIN_P + zc * COS_P + CAM_Z
        h = zb + STAND_HEIGHT
        if h < FLOOR_MARGIN or h > CEILING:
            continue
        if xb < MIN_VALID:
            continue
        bearing = math.degrees(math.atan2(yb, xb))   # +ve = left
        if abs(bearing) > FOV_DEG:
            continue
        idx = int((bearing + FOV_DEG) // BIN_DEG)
        idx = min(max(idx, 0), nbins - 1)
        r = math.hypot(xb, yb)
        if bins[idx] is None or r < bins[idx]:
            bins[idx] = r
    return bins


def main():
    rclpy.init()
    n = S()
    t0 = time.time()
    while n.msg is None and time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.msg is None:
        print('no point cloud')
        rclpy.shutdown()
        return

    bins = profile(n.msg)
    if bins is None:
        print('cloud has no xyz fields')
        rclpy.shutdown()
        return

    nbins = len(bins)
    print(f'cloud {n.msg.width * n.msg.height} points   '
          f'bearing +ve = LEFT, bars scale to {MAX_RANGE} m\n')
    for i in range(nbins - 1, -1, -1):        # left at top
        lo = -FOV_DEG + i * BIN_DEG
        r = bins[i]
        if r is None:
            bar = '=' * 40
            txt = 'clear'
        else:
            bar = '#' * max(1, int(40 * r / MAX_RANGE))
            txt = f'{r:.2f} m'
        side = 'L' if lo >= 0 else 'R'
        print(f'  {lo:+4d}..{lo+BIN_DEG:+4d} {side}  {bar:<40} {txt}')

    free = [i for i, r in enumerate(bins) if r is None or r > 1.5]
    print()
    if not free:
        print('>>> no gap wider than 1.5 m anywhere in the FOV')
    else:
        left = [i for i in free if -FOV_DEG + i * BIN_DEG >= 0]
        right = [i for i in free if -FOV_DEG + i * BIN_DEG < 0]
        print(f'>>> free bins: {len(left)} on the LEFT, {len(right)} on the RIGHT')
        if len(right) > len(left):
            print('    more free space to the RIGHT')
        elif len(left) > len(right):
            print('    more free space to the LEFT')
        else:
            print('    balanced')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

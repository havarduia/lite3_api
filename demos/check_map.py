"""check_map - one live lidar scan drawn on a floor map, and how well it fits.

    python3 demos/check_map.py <floor> <x> <y> <yaw_deg> [picture.ppm] [topic]

Run on the robot, standing (the relay only passes scans on then), with the
pose of base_link on the map as well as you know it. Prints the share of
the scan that lands on the map's walls at that pose, then looks half a
metre and 15 degrees either way for the pose that fits best. If even the
best fit is poor the map and the scan do not show the same walls (height
slice, scale, mirror), and AMCL has no chance either.

The picture: the map, the scan at the pose given in red, at the best pose
in green, the robot in blue. The scan is the one localization uses
(/scan_walls); name /scan as the topic to see the low one instead.
"""
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import locate, places      # noqa: E402

ZOOM = 6
REACH_M, STEP_M = 0.5, 0.05
REACH_DEG, STEP_DEG = 15, 1


def one_scan(topic, timeout=8.0):
    import time
    import rclpy
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node('check_map')
    got = []
    node.create_subscription(LaserScan, topic, got.append, 1)
    end = time.monotonic() + timeout
    while not got and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        sys.exit('no %s within %.0f s: is the robot standing, and lio_relay running?' % (topic, timeout))
    return got[0].angle_min, got[0].angle_increment, np.array(got[0].ranges, dtype=np.float32)


def best_near(floor_map, pose, scan):
    best = (floor_map.fit(pose, *scan) or 0.0, pose)
    for dx in np.arange(-REACH_M, REACH_M + 1e-6, STEP_M):
        for dy in np.arange(-REACH_M, REACH_M + 1e-6, STEP_M):
            for dyaw in range(-REACH_DEG, REACH_DEG + 1, STEP_DEG):
                p = (pose[0] + dx, pose[1] + dy, pose[2] + math.radians(dyaw))
                best = max(best, (floor_map.fit(p, *scan) or 0.0, p))
    return best


def draw(yaml_path, floor_map, scan, poses, out):
    with open(yaml_path) as f:
        picture = locate.read_pgm(os.path.join(os.path.dirname(yaml_path), yaml.safe_load(f)['image']))
    rgb = np.repeat(np.repeat(np.stack([picture] * 3, axis=-1), ZOOM, axis=0), ZOOM, axis=1).copy()
    h = rgb.shape[0]

    def dots(x, y, colour, size):
        px = ((x - floor_map.x0) / floor_map.res * ZOOM).astype(int)
        py = h - 1 - ((y - floor_map.y0) / floor_map.res * ZOOM).astype(int)    # the picture's top is the far edge
        for ox in range(-size, size + 1):
            for oy in range(-size, size + 1):
                ok = (px + ox >= 0) & (px + ox < rgb.shape[1]) & (py + oy >= 0) & (py + oy < h)
                rgb[py[ok] + oy, px[ok] + ox] = colour

    angle_min, increment, ranges = scan
    seen = np.isfinite(ranges) & (ranges < locate.FIT_RANGE_M)
    angles = angle_min + increment * np.arange(len(ranges))
    for (x, y, yaw), colour in poses:
        dots(x + ranges[seen] * np.cos(yaw + angles[seen]), y + ranges[seen] * np.sin(yaw + angles[seen]), colour, 1)
        dots(np.array([x]), np.array([y]), (0, 0, 255), 3)
    with open(out, 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (rgb.shape[1], h) + rgb.tobytes())


if __name__ == '__main__':
    if len(sys.argv) not in (5, 6, 7):
        sys.exit(__doc__)
    yaml_path = os.path.join(places.floor_dir(sys.argv[1]), 'map.yaml')
    floor_map = locate.Map(yaml_path)
    pose = (float(sys.argv[2]), float(sys.argv[3]), math.radians(float(sys.argv[4])))
    scan = one_scan(sys.argv[6] if len(sys.argv) == 7 else locate.SCAN_TOPIC)
    seen = int((np.isfinite(scan[2]) & (scan[2] < locate.FIT_RANGE_M)).sum())
    fit = floor_map.fit(pose, *scan)
    print('%d of %d beams returned within %.0f m' % (seen, len(scan[2]), locate.FIT_RANGE_M))
    print('fit at the pose given: %s  (localized needs %.2f)' % ('none' if fit is None else '%.2f' % fit, locate.FIT_MIN))
    score, (x, y, yaw) = best_near(floor_map, pose, scan)
    print('best fit nearby: %.2f at (%.2f, %.2f, %.0f deg)' % (score, x, y, math.degrees(yaw)))
    out = sys.argv[5] if len(sys.argv) >= 6 else '/tmp/check_map.ppm'
    draw(yaml_path, floor_map, scan, [(pose, (255, 0, 0)), ((x, y, yaw), (0, 160, 0))], out)
    print('picture:', out)

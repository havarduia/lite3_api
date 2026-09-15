#!/usr/bin/env python3
"""Does the robot stop if velocity commands just STOP, with no zero sent?

Starts walk_forward.py, waits until the robot is genuinely moving, then kills it
abruptly so its finally-block never runs and no zero Twist is ever published.
Then watches odometry to see whether the robot halts on its own.

Force-stops at the end regardless of the outcome.
"""
import os
import subprocess
import sys
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

WATCH_AFTER_KILL = 6.0     # s to observe before forcing a stop
MOVE_THRESHOLD = 0.08      # m; "genuinely walking"


class Watch(Node):
    def __init__(self):
        super().__init__('failsafe_watch')
        self.pose = None
        self.create_subscription(Odometry, 'leg_odom2', self._cb, 10)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)

    def _cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_pose(self, timeout=5.0):
        end = time.time() + timeout
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.pose

    def force_stop(self):
        for _ in range(25):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def main():
    rclpy.init()
    w = Watch()
    proc = None
    try:
        if w.wait_pose() is None:
            sys.exit('no /leg_odom2')

        print('launching walk_forward.py --stand (distance 3 m, ended early)')
        proc = subprocess.Popen(
            ['./walk_forward.py', '--stand', '--speed', '0.15', '--distance', '3.0'],
            cwd=os.path.expanduser('~'),
            stdout=open('/tmp/walk_sub.log', 'w'),
            stderr=subprocess.STDOUT)

        # Wait until it is actually walking, not merely standing up.
        origin = w.pose
        launched = time.time()
        while time.time() - launched < 45:
            w.pump(0.1)
            if dist(w.pose, origin) > MOVE_THRESHOLD:
                break
        else:
            print('robot never started moving. walk_forward.py said:')
            try:
                print(open('/tmp/walk_sub.log').read())
            except OSError:
                pass
            sys.exit('aborting test')

        moving_at = w.pose
        print(f'robot is walking (moved {dist(moving_at, origin):.3f} m)')
        w.pump(1.5)

        kill_pose = w.pose
        proc.kill()
        print(f'\n*** publisher killed abruptly at x={kill_pose[0]:.3f} '
              f'y={kill_pose[1]:.3f} - no zero Twist was published ***\n')

        t0 = time.time()
        last = kill_pose
        while time.time() - t0 < WATCH_AFTER_KILL:
            w.pump(0.5)
            d = dist(w.pose, kill_pose)
            step = dist(w.pose, last)
            print(f'  t+{time.time()-t0:4.1f}s  moved {d:5.3f} m since kill '
                  f'({"MOVING" if step > 0.005 else "stopped"})')
            last = w.pose

        drift = dist(w.pose, kill_pose)
        print()
        if drift < 0.05:
            print(f'>>> STOPPED ON ITS OWN ({drift:.3f} m after kill). '
                  f'There is a velocity timeout.')
        else:
            print(f'>>> KEPT GOING: {drift:.3f} m after the kill. '
                  f'NO failsafe - a dead process leaves the robot walking.')
    finally:
        if proc and proc.poll() is None:
            proc.kill()
        print('\nforcing stop...')
        w.force_stop()
        w.pump(1.0)
        print(f'final pose: {w.pose}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()

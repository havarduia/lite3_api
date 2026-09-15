#!/usr/bin/env python3
"""Does VOA actually STOP the robot for an obstacle, or only throttle it?

Commands a steady forward velocity toward an obstacle roughly 0.5 m ahead and
watches whether cmd_vel_corrected collapses to zero and the robot halts short.

Safety: a hard odometry cap stops the run well before the obstacle even if VOA
does nothing at all. Zero velocity is always published on the way out.
"""
import os
import sys
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

SPEED = 0.15
HARD_CAP_M = float(sys.argv[1]) if len(sys.argv) > 1 else 0.35
MAX_SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0


class T(Node):
    def __init__(self):
        super().__init__('obstacle_test')
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(Twist, 'cmd_vel', q)
        self.pose = None
        self.corrected = []
        self.create_subscription(
            Twist, 'cmd_vel_corrected',
            lambda m: self.corrected.append(
                (m.linear.x, m.linear.y, m.angular.z)), q)
        self.create_subscription(Odometry, 'leg_odom2', self._odom, 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y)

    def pump(self, s):
        e = time.time() + s
        while time.time() < e:
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        for _ in range(25):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def d(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def main():
    rclpy.init()
    n = T()
    reason = 'time limit'
    try:
        n.pump(2.0)
        if n.pose is None:
            print('no odom')
            return
        start = n.pose
        print(f'start pose {start[0]:.3f},{start[1]:.3f}   '
              f'commanding {SPEED} m/s toward obstacle ~0.5 m ahead')
        print(f'hard cap {HARD_CAP_M} m of travel regardless of VOA\n')

        t = Twist()
        t.linear.x = SPEED
        t0 = time.time()
        last_report = 0.0
        while time.time() - t0 < MAX_SECONDS:
            n.pub.publish(t)
            rclpy.spin_once(n, timeout_sec=0.02)
            moved = d(n.pose, start)
            el = time.time() - t0
            if el - last_report >= 1.0:
                recent = n.corrected[-40:] or [(0.0, 0.0, 0.0)]
                mx = max(v[0] for v in recent)
                my = max(v[1] for v in recent)
                mny = min(v[1] for v in recent)
                maz = max(v[2] for v in recent)
                mnz = min(v[2] for v in recent)
                print(f'  t+{el:4.1f}s  moved {moved:5.3f} m   '
                      f'x max {mx:+.3f}   y [{mny:+.3f},{my:+.3f}]   '
                      f'yaw [{mnz:+.3f},{maz:+.3f}]')
                last_report = el
            if moved >= HARD_CAP_M:
                reason = 'HARD CAP hit - VOA did not stop it'
                break
            # If VOA has been commanding zero for a sustained window, it vetoed.
            recent = [v[0] for v in n.corrected[-60:]]
            if len(recent) == 60 and max(recent) <= 0.001 and el > 2.0:
                reason = 'VOA VETO - corrected held zero'
                break

        moved = d(n.pose, start)
        print(f'\nstopped after {moved:.3f} m -> {reason}')
        allc = n.corrected or [(0.0, 0.0, 0.0)]
        nz = sum(1 for v in allc if v[0] > 0.001)
        turn = sum(1 for v in allc if abs(v[2]) > 0.001)
        lat = sum(1 for v in allc if abs(v[1]) > 0.001)
        print(f'corrected total {len(allc)}')
        print(f'  forward non-zero : {nz} ({100*nz/len(allc):.0f}%)')
        print(f'  yaw   non-zero   : {turn} ({100*turn/len(allc):.0f}%)  '
              f'range [{min(v[2] for v in allc):+.3f},{max(v[2] for v in allc):+.3f}]')
        print(f'  lateral non-zero : {lat} ({100*lat/len(allc):.0f}%)  '
              f'range [{min(v[1] for v in allc):+.3f},{max(v[1] for v in allc):+.3f}]')
        if reason.startswith('VOA VETO'):
            print('\n>>> VOA STOPS for obstacles. Avoidance is authoritative.')
        elif moved >= HARD_CAP_M:
            print('\n>>> VOA did NOT stop the robot within the safety cap.')
            print('    It throttles but does not veto - do not rely on it.')
    finally:
        n.stop()
        n.pump(1.0)
        print(f'final pose {n.pose}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()

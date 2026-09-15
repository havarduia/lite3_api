#!/usr/bin/env python3
"""With VOA running and the robot STANDING, does obstacle avoidance pass velocity through?

Lying down, the camera stares at the floor and a correct avoidance node outputs
zero - which is indistinguishable from a broken one. Standing with clear space
ahead is the only condition that separates the two.

Safe by construction: with the Jetson2Motion fix, raw cmd_vel is suppressed while
corrections arrive, so if VOA says zero the robot simply does not move.
"""
import os
import socket
import struct
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import Int32MultiArray  # noqa: E402

MOTION = ('192.168.1.120', 43893)
STAND = 0x21010202
AUTO = 0x21010C03
STANDING = 6
LYING = (1, 8)


def send(code):
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(
        struct.pack('<III', code, 0, 0), MOTION)


class T(Node):
    def __init__(self):
        super().__init__('voa_test')
        q = QoSProfile(depth=1)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(Twist, 'cmd_vel', q)
        self.corrected = []
        self.pose = None
        self.state = None
        self.create_subscription(Twist, 'cmd_vel_corrected',
                                 lambda m: self.corrected.append(m.linear.x), q)
        self.create_subscription(Odometry, 'leg_odom2', self._odom, 10)
        self.create_subscription(Int32MultiArray, '/robot_state_debug',
                                 self._st, 10)

    def _odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y)

    def _st(self, m):
        if m.data:
            self.state = m.data[0]

    def pump(self, s):
        e = time.time() + s
        while time.time() < e:
            rclpy.spin_once(self, timeout_sec=0.01)

    def stop(self):
        for _ in range(25):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init()
    n = T()
    try:
        n.pump(2.0)
        print(f'state={n.state}  pose={n.pose}')
        send(AUTO)
        n.pump(0.5)

        if n.state in LYING:
            print('standing the robot...')
            send(STAND)
            e = time.time() + 20
            while time.time() < e and n.state != STANDING:
                n.pump(0.2)
        if n.state != STANDING:
            print(f'could not stand (state={n.state}); aborting')
            return
        print('standing')
        n.pump(1.0)

        start = n.pose
        n.corrected.clear()
        print('\npublishing cmd_vel linear.x=0.15 for 5 s with VOA active...')
        t0 = time.time()
        t = Twist()
        t.linear.x = 0.15
        while time.time() - t0 < 5.0:
            n.pub.publish(t)
            rclpy.spin_once(n, timeout_sec=0.02)

        c = n.corrected
        nz = [v for v in c if v > 0.001]
        moved = ((n.pose[0] - start[0]) ** 2 + (n.pose[1] - start[1]) ** 2) ** 0.5
        print(f'\ncorrected msgs   : {len(c)}')
        print(f'non-zero         : {len(nz)}  ({100*len(nz)/max(len(c),1):.0f}%)')
        if c:
            print(f'max linear.x     : {max(c):.4f}')
        print(f'robot moved      : {moved:.3f} m')
        print()
        if moved > 0.05:
            print('>>> VOA PASSES VELOCITY THROUGH. Avoidance is live and not blocking.')
        else:
            print('>>> VOA still outputs zero while standing with clear space.')
            print('    Not a floor-detection artefact - something else is wrong.')
    finally:
        n.stop()
        n.pump(1.0)
        print(f'stopped. final pose={n.pose}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()

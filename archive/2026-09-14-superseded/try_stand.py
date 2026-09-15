#!/usr/bin/env python3
"""Send 0x31010C05 ("all joints back to zero") to jy_exe and watch what it does.

Hypothesis: the SDK only ever sends this immediately before seizing joint control,
so jy_exe never gets to act on it. Sent alone, it may run the app's [Stand]
sequence - "reset to zero and then automatically stand up" (User Manual 3.5) -
which would move jy_exe's own state machine to standing.

Sends nothing else, so SDK control is never taken. Robot must be in Ready Posture.
"""
import os
import socket
import struct
import sys
import time

os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Int32MultiArray  # noqa: E402

MOTION_IP = '192.168.1.120'
MOTION_PORT = 43893
ALL_JOINT_BACK_ZERO = 0x31010C05

FIELDS = ['basic', 'gait', 'policy', 'motion', 'task', 'need_move', 'zero_flag']


class StateWatch(Node):
    def __init__(self):
        super().__init__('try_stand_watch')
        self.state = None
        self.create_subscription(
            Int32MultiArray, '/robot_state_debug', self._cb, 10)

    def _cb(self, msg):
        self.state = list(msg.data)

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_state(self, timeout=5.0):
        end = time.time() + timeout
        while self.state is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.state


def show(label, state):
    if state is None:
        print(f'{label}: <no state>')
        return
    print(f'{label}: ' + '  '.join(
        f'{n}={v}' for n, v in zip(FIELDS, state)))


def main():
    rclpy.init()
    node = StateWatch()
    try:
        before = node.wait_for_state()
        if before is None:
            sys.exit('No /robot_state_debug - is transfer_ros2.service up?')
        show('before', before)
        if before[0] != 8:
            print(f'\nWARNING: basic_state is {before[0]}, expected 8 (prone).')
            print('The manual says to run [Stand] from Ready Posture only.')
            if '--force' not in sys.argv:
                sys.exit('Refusing to send. Pass --force to override.')

        # EthCommand: uint32 code, uint32 value, uint32 (type:8 | count:24)
        packet = struct.pack('<III', ALL_JOINT_BACK_ZERO, 0, 0)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(packet, (MOTION_IP, MOTION_PORT))
        print(f'\nsent 0x{ALL_JOINT_BACK_ZERO:08X} ({len(packet)} bytes) '
              f'-> {MOTION_IP}:{MOTION_PORT}')
        print('sending nothing further, so SDK control is never taken.\n')

        last = before
        start = time.time()
        while time.time() - start < 20:
            node.pump(0.5)
            if node.state != last:
                show(f'  t+{time.time()-start:4.1f}s', node.state)
                last = node.state

        print()
        show('after', node.state)
        if node.state and node.state[0] == 6:
            print('\n>>> basic_state reached 6 (standing) - jy_exe stood it up.')
        elif node.state == before:
            print('\n>>> no state change; jy_exe ignored the command.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

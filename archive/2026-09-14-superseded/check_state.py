#!/usr/bin/env python3
import os
import time

# The transfer stack runs CycloneDDS. Without this a default FastRTPS shell
# discovers nothing and this script silently reports "messages: 0".
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

FIELDS = [
    'robot_basic_state',
    'robot_gait_state',
    'robot_policy_state',
    'robot_motion_state',
    'task_state',
    'is_robot_need_move',
    'zero_position_flag',
    'battery_percent',
]

# Observed empirically; see project notes.
BASIC_STATE = {
    1: 'ready/lying',
    4: 'standing up',
    5: 'standing up',
    6: 'STANDING',
    7: 'lying down',
    8: 'not ready',
    9: 'not ready',
    17: 'zeroing',
    98: 'post power-on',
}


class Check(Node):
    def __init__(self):
        super().__init__('state_check')
        self.last = None
        self.count = 0
        self.create_subscription(Int32MultiArray, '/robot_state_debug', self.cb, 10)

    def cb(self, msg):
        self.last = list(msg.data)
        self.count += 1


def main():
    rclpy.init()
    node = Check()
    start = time.time()
    while time.time() - start < 10:
        rclpy.spin_once(node, timeout_sec=0.2)

    print(f"messages: {node.count}")
    if not node.last:
        print("no data - is transfer_ros2.service running?")
        rclpy.shutdown()
        return

    for i, value in enumerate(node.last):
        name = FIELDS[i] if i < len(FIELDS) else f'field_{i}'
        note = ''
        if name == 'robot_basic_state':
            note = f"  ({BASIC_STATE.get(value, 'unknown')})"
        elif name == 'battery_percent':
            note = '  (LOW - robot may refuse to stand)' if value < 20 else \
                   '  (charge soon)' if value < 30 else ''
        print(f"  {name:<20} {value}{note}")

    if len(node.last) < len(FIELDS):
        print(f"\nnote: only {len(node.last)} fields - battery needs the "
              f"rebuilt transfer package")
    print(f"\nraw: {node.last}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()

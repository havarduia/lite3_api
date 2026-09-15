#!/usr/bin/env python3
"""Walk the Lite3 forward using the onboard locomotion controller.

Publishes /cmd_vel, which jetson2motion translates to UDP cmd_code 320 on
192.168.1.120:43893. Requires transfer_ros2.service to be running, and is
mutually exclusive with Lite3_MotionSDK (Lite_motion seizes SDK joint control).

    ./walk_forward.py --speed 0.15 --distance 0.5
    ./walk_forward.py --speed 0.2 --duration 3

A zero Twist is always published on the way out: normal finish, Ctrl-C, or
unhandled exception.
"""
import argparse
import math
import os
import socket
import struct
import sys
import time

# The transfer stack runs CycloneDDS (start_transfer.sh). A shell defaulting to
# FastRTPS discovers nothing at all and just sits there, so pin it before rclpy
# loads the middleware. An explicit setting from the caller still wins.
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray

MAX_SPEED = 0.6          # m/s; refuse anything above this
HARD_TIMEOUT = 30.0      # s; ceiling on any single run

# jy_exe on the motion computer. STAND is a TOGGLE: 6 -> 7 -> 1 lies down,
# 1 -> 4 -> 5 -> 6 stands up. Code confirmed by capturing the handheld
# (192.168.2.99 -> 192.168.2.1:43893) pressing [Stand].
MOTION_ADDR = ('192.168.1.120', 43893)
CMD_STAND_TOGGLE = 0x21010202
CMD_MODE_AUTO = 0x21010C03    # required, or /cmd_vel is silently ignored
CMD_MODE_MANUAL = 0x21010C02  # the handheld sends this whenever you move a joystick

# Both are lying/ready postures, reached different ways: 1 after a commanded
# lie-down, 8 after power-on. Either is safe to toggle up from. Transitional
# states (4, 5, 7) and zeroing (17) are not.
STATES_LYING = (1, 8)
STATE_STANDING = 6

# The robot silently refuses to stand on a flat battery: the stand toggle is
# ignored and basic_state just cycles 1/8/9 with nothing explaining why. It was
# observed refusing at 17%. Fail loudly instead of leaving that to be debugged.
BATTERY_REFUSE = 20      # %
BATTERY_WARN = 30        # %
BATTERY_INDEX = 7        # appended to /robot_state_debug after zero_flag


class WalkForward(Node):
    def __init__(self):
        super().__init__('walk_forward')
        # Match MotionSender's subscription: best effort, depth 1. Velocity is
        # a stream of latest-value-wins commands, so a reliable queue would
        # only retransmit stale ones.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.pose = None
        self.basic_state = None
        self.battery = None
        self.create_subscription(Odometry, 'leg_odom2', self._odom_cb, 10)
        self.create_subscription(
            Int32MultiArray, '/robot_state_debug', self._state_cb, 10)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _state_cb(self, msg):
        if msg.data:
            self.basic_state = msg.data[0]
        if len(msg.data) > BATTERY_INDEX:
            self.battery = msg.data[BATTERY_INDEX]

    def check_battery(self, force=False):
        """True if it is sane to proceed. None means an older transfer build
        that does not publish battery - do not block on that."""
        if self.battery is None:
            print('battery not published by this transfer build - not checking')
            return True
        if self.battery < BATTERY_REFUSE:
            print(f'battery {self.battery}% is below {BATTERY_REFUSE}%.')
            if not force:
                print('The robot will likely refuse to stand and give no reason.')
                return False
            print('--force given; proceeding anyway')
        elif self.battery < BATTERY_WARN:
            print(f'WARNING: battery {self.battery}% - charge soon')
        else:
            print(f'battery {self.battery}%')
        return True

    def set_auto_mode(self):
        """Put the robot in auto mode. Without this /cmd_vel does nothing at all,
        and there is no topic that reports the current mode, so we cannot check
        first - just set it. Moving a handheld joystick reverts it to manual."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(struct.pack('<III', CMD_MODE_AUTO, 0, 0), MOTION_ADDR)
        print(f'set auto mode (0x{CMD_MODE_AUTO:08X})')
        time.sleep(0.5)

    def stand(self, timeout=20.0):
        """Toggle the robot up if it isn't already standing. Returns True if standing."""
        end = time.time() + 5.0
        while self.basic_state is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.basic_state is None:
            print('no /robot_state_debug - cannot confirm posture')
            return False
        if self.basic_state == STATE_STANDING:
            print('already standing')
            return True
        if self.basic_state not in STATES_LYING:
            print(f'basic_state={self.basic_state}, expected one of '
                  f'{STATES_LYING} (lying) or {STATE_STANDING} (standing) '
                  f'- not toggling')
            return False

        print(f'basic_state={self.basic_state} (lying), sending stand toggle '
              f'0x{CMD_STAND_TOGGLE:08X}')
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(struct.pack('<III', CMD_STAND_TOGGLE, 0, 0), MOTION_ADDR)

        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.basic_state == STATE_STANDING:
                print('standing')
                return True
        print(f'timed out waiting to stand (basic_state={self.basic_state})')
        return False

    def pump_state(self, timeout=3.0):
        end = time.time() + timeout
        while self.basic_state is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_odom(self, timeout=3.0):
        end = time.time() + timeout
        while self.pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.pose is not None

    def publish(self, vx):
        t = Twist()
        t.linear.x = vx
        self.pub.publish(t)

    def stop(self):
        """Repeat the zero command; best-effort delivery can drop a single one."""
        for _ in range(20):
            self.publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


def travelled(start, pose):
    return math.hypot(pose[0] - start[0], pose[1] - start[1])


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speed', type=float, default=0.15,
                    help='forward velocity in m/s (default: 0.15)')
    limit = ap.add_mutually_exclusive_group()
    limit.add_argument('--distance', type=float,
                       help='stop after this many metres of odometry')
    limit.add_argument('--duration', type=float,
                       help='stop after this many seconds')
    ap.add_argument('--rate', type=float, default=20.0,
                    help='publish rate in Hz (default: 20)')
    ap.add_argument('--stand', action='store_true',
                    help='stand the robot first if it is lying in ready pose. '
                         'Off by default: this physically moves the robot.')
    ap.add_argument('--force', action='store_true',
                    help=f'run even if the battery is below {BATTERY_REFUSE}%%')
    ap.add_argument('--no-auto', dest='auto', action='store_false',
                    help='do not send the auto-mode command. Only useful if a '
                         'human is driving with the handheld and you do not '
                         'want to take control away from them.')
    args = ap.parse_args()

    if not 0 < args.speed <= MAX_SPEED:
        sys.exit(f'--speed must be in (0, {MAX_SPEED}] m/s, got {args.speed}')
    if args.distance is None and args.duration is None:
        args.distance = 0.5
    if args.distance is not None and args.distance <= 0:
        sys.exit('--distance must be positive')

    rclpy.init()
    node = WalkForward()
    start = None
    reason = 'interrupted'
    try:
        if not node.wait_for_odom():
            sys.exit('No /leg_odom2 within 3 s - is transfer_ros2.service running, '
                     'and is Lite_motion stopped? Refusing to move.')
        node.pump_state()
        if not node.check_battery(args.force):
            sys.exit('refusing to run on a flat battery (--force to override)')
        if args.auto:
            node.set_auto_mode()
        if args.stand and not node.stand():
            sys.exit('could not get the robot standing; refusing to send velocity')
        if node.basic_state is not None and node.basic_state != STATE_STANDING:
            sys.exit(f'basic_state={node.basic_state}, not standing ({STATE_STANDING}). '
                     f'Pass --stand, or stand it with the handheld. '
                     f'Velocity is ignored unless the robot is standing.')
        start = node.pose
        print(f'start:  x={start[0]:.3f}  y={start[1]:.3f}')
        print(f'walking at {args.speed} m/s '
              f'({"%.2f m" % args.distance if args.distance else "%.1f s" % args.duration})')

        period = 1.0 / args.rate
        deadline = time.time() + min(args.duration or HARD_TIMEOUT, HARD_TIMEOUT)
        while time.time() < deadline:
            node.publish(args.speed)
            rclpy.spin_once(node, timeout_sec=period)
            if args.distance is not None and node.pose is not None:
                if travelled(start, node.pose) >= args.distance:
                    reason = 'distance reached'
                    break
        else:
            reason = 'time limit reached'
    finally:
        node.stop()
        node.spin_for(1.0)
        if start is not None and node.pose is not None:
            print(f'end:    x={node.pose[0]:.3f}  y={node.pose[1]:.3f}')
            print(f'moved:  {travelled(start, node.pose):.3f} m  ({reason})')
        rclpy.shutdown()


if __name__ == '__main__':
    main()

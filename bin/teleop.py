#!/usr/bin/env python3
"""Drive the robot from the keyboard over SSH.

    python3 ~/robot/bin/teleop.py                 # w/s drive, a/d turn, space stop, q quit
    python3 ~/robot/bin/teleop.py --speed 0.4
    python3 ~/robot/bin/teleop.py --handheld      # real controller sends the heartbeat
    python3 ~/robot/bin/teleop.py --selftest      # key mapping only, no robot

Keys are momentary: he keeps going while you hold one and stops HOLD seconds
after the last keypress, so a dropped SSH session stops him too. +/- change
speed. He stands at the start and sits at the end; Ctrl-C is the e-stop.

There is NO obstacle guard here - walk() has one, steer() does not. You watch
him. Every 29 s the steer() call is renewed, which shows up as a ~0.5 s pause.
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.lite3 import Lite3, MAX_SPEED, MAX_YAW_RATE  # noqa: E402

HOLD = 0.5          # s without a keypress before he stops
STEER_LIMIT = 29.0  # s, under lite3's 30 s ceiling


def step(key, speed, yaw):
    """(vx, wz) for a driving key, or None if the key does not drive."""
    return {'w': (speed, 0.0),
            's': (-speed, 0.0),
            'a': (0.0, yaw),
            'd': (0.0, -yaw)}.get(key)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--speed', type=float, default=0.3, help='m/s, max %.1f' % MAX_SPEED)
    p.add_argument('--yaw', type=float, default=0.6, help='rad/s, max %.1f' % MAX_YAW_RATE)
    p.add_argument('--handheld', action='store_true', help='do not send our own heartbeat')
    p.add_argument('--force', action='store_true', help='ignore the battery floor')
    p.add_argument('--selftest', action='store_true')
    args = p.parse_args()

    if args.selftest:
        assert step('w', 0.3, 0.6) == (0.3, 0.0)
        assert step('s', 0.3, 0.6) == (-0.3, 0.0)
        assert step('a', 0.3, 0.6) == (0.0, 0.6)
        assert step('d', 0.3, 0.6) == (0.0, -0.6)
        assert step('k', 0.3, 0.6) is None
        print('selftest ok')
        return

    state = {'v': (0.0, 0.0), 'last': 0.0, 'quit': False,
             'speed': min(args.speed, MAX_SPEED), 'yaw': min(args.yaw, MAX_YAW_RATE)}

    def control():
        now = time.time()
        while select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            if k in ('q', '\x03', ''):
                state['quit'] = True
                return 'quit'
            if k == ' ':
                state['v'] = (0.0, 0.0)
            elif k in '+=':
                state['speed'] = min(MAX_SPEED, state['speed'] + 0.05)
            elif k == '-':
                state['speed'] = max(0.05, state['speed'] - 0.05)
            else:
                move = step(k, state['speed'], state['yaw'])
                if move:
                    state['v'] = move
                    state['last'] = now
        if now - state['last'] > HOLD:
            state['v'] = (0.0, 0.0)
        return state['v']

    bot = Lite3()
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        if not args.handheld:
            bot.heartbeat_start()
            bot.wait_ready()
        bot.stand()
        print('w/s drive  a/d turn  space stop  +/- speed  q quit')
        tty.setcbreak(fd)
        while not state['quit']:
            bot.steer(control, limit=STEER_LIMIT, force=args.force)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        bot.sit()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""check_motion.py - what the motion code promises, checked with NO robot.

    python3 ~/robot/demos/check_motion.py     # needs rclpy to import, nothing else

A fake node, a fake clock and a pretend Nav2 stand in for the robot, and no
packet or ROS message leaves this process, so it is safe to run anywhere -
next to a live robot too. Run it after touching Lite3._loop, _run, steer,
walk, turn, goto or nav_stop.
"""
import math
import os
import subprocess
import sys
import time
import types

import rclpy.action
from builtin_interfaces.msg import Time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import lite3 as L, nav, protocol as P  # noqa: E402

P.send = lambda code, value=0: None         # mode packets stay here


class World:
    """Stands in for Lite3._node: a pose that follows the last Twist exactly,
    on a clock that only sleep() moves. at=(seconds, fn(world)) is something
    that happens on the way."""
    state = {'basic': L.STANDING, 'battery': 80}
    cloud, tilt, stick_time, at = None, (0.0, 0.0), -1.0, None

    def __init__(self, **setup):
        self.t, self.odom, self.cmd, self.sent, self.pub = 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), [], self
        vars(self).update(setup)

    def publish(self, twist):
        self.cmd = (twist.linear.x, twist.linear.y, twist.angular.z)
        self.sent.append(self.cmd)

    def sleep(self, dt):
        (x, y, yaw), (vx, vy, wz) = self.odom, self.cmd
        self.odom = (x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt,
                     y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt,
                     math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt)))
        self.t += dt
        if self.at and self.t >= self.at[0]:
            happen, self.at = self.at[1], None
            happen(self)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=Time))


class Bot(L.Lite3):
    nav_running = True                      # no pgrep: the pretend Nav2 below is always up


def run(move, **setup):
    """move(bot) on a fresh fake robot -> (its result or exception, was he left
    halted, every Twist sent)."""
    w, bot = World(**setup), Bot.__new__(Bot)
    w.bot, bot._node, bot._nav_client, bot._goal_handle = bot, w, None, None
    real = time.time, time.sleep
    time.time, time.sleep = (lambda: w.t), w.sleep
    try:
        r = move(bot)
    except Exception as e:
        r = e
    finally:
        time.time, time.sleep = real
    return r, set(w.sent[-20:]) == {(0.0, 0.0, 0.0)}, w.sent


def pretend_nav2(arrives):
    """rclpy's ActionClient, pretended. A goal ends when it is cancelled, or
    `arrives` seconds after it was sent, or (None) never. Returns the list of
    clients made."""
    made = []

    def client(node, action, name):
        goal = {}
        result = types.SimpleNamespace(
            done=lambda: goal['cancelled'] or (arrives is not None and node.t >= goal['sent'] + arrives),
            result=lambda: types.SimpleNamespace(status=nav.CANCELED if goal['cancelled'] else nav.SUCCEEDED))
        handle = types.SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                       cancel_goal_async=lambda: goal.update(cancelled=True))
        made.append(name)
        return types.SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda msg: goal.update(sent=node.t, cancelled=False) or types.SimpleNamespace(
                done=lambda: True, result=lambda: handle))

    rclpy.action.ActionClient = client
    sys.modules['nav2_msgs'] = sys.modules['nav2_msgs.action'] = types.SimpleNamespace(
        NavigateToPose=types.SimpleNamespace(Goal=types.SimpleNamespace))
    return made


def boom():
    raise ZeroDivisionError


def main():
    # every way out of a motion call leaves him stopped, and says why
    r, halted, _ = run(lambda b: b.walk(0.5, stop_distance=None))
    assert r['reason'] == 'target reached' and 0.5 <= r['moved'] < 0.55 and halted, r
    r, halted, _ = run(lambda b: b.turn_deg(270, guard=False))   # past 180: counted, not wrapped
    assert r['reason'] == 'target reached' and 260 < r['turned_deg'] < 275 and halted, r
    r, halted, sent = run(lambda b: b.steer(lambda: (9.0, -9.0), limit=1.0))
    assert r['reason'] == 'time limit' and halted, r
    assert sent[0] == (L.MAX_SPEED, 0.0, -L.MAX_YAW_RATE), sent[0]
    r, halted, _ = run(lambda b: b.steer(lambda: (0.1, 0.0), limit=999))    # HARD_TIMEOUT caps it
    assert r['reason'] == 'time limit' and r['moved'] < 0.1 * (L.HARD_TIMEOUT + 1) and halted, r
    r, halted, _ = run(lambda b: b.steer(lambda: 'seen enough'))
    assert r['reason'] == 'seen enough' and halted, r
    r, halted, _ = run(lambda b: b.steer(boom))                  # a crash still ends stopped
    assert isinstance(r, ZeroDivisionError) and halted, r
    r, halted, _ = run(lambda b: b.steer(lambda: (0.2, 0.0)), at=(2.0, lambda w: setattr(w, 'stick_time', w.t)))
    assert r['reason'].startswith('handheld took over') and halted, r
    r, _, sent = run(lambda b: b.walk(0.5, stop_distance=None), state={'basic': 1, 'battery': 80})
    assert isinstance(r, L.Lite3Error) and not sent, r           # lying: refused, nothing sent
    r, _, sent = run(lambda b: b.steer(lambda: (float('nan'), float('nan')), limit=1.0))
    assert set(sent) == {(0.0, 0.0, 0.0)}, sent[0]               # a NaN is a stop, not full speed
    for bad in (lambda b: b.strafe(0.5, speed=0), lambda b: b.strafe(0.5, speed=9), lambda b: b.turn(1.0, rate=0)):
        r, _, sent = run(bad)
        assert isinstance(r, L.Lite3Error) and not sent, r       # refused, not a ZeroDivisionError

    # a depth guard reads the cloud at 4 Hz however often the loop asks, and still stops him
    reads = []
    r, halted, _ = run(lambda b: setattr(b, 'clearance', lambda: reads.append(b._node.t) or 3.0 - b._node.odom[0])
                       or b.walk(5.0, speed=0.5, stop_distance=0.6))
    assert r['reason'].startswith('obstacle at') and 2.2 < r['moved'] < 2.6 and halted, r
    assert all(b - a >= 0.25 - 1e-9 for a, b in zip(reads[1:], reads[2:])), reads    # reads[0] is the pre-flight

    # upright(): armed BEFORE waiting for ready, and sat down on the way out even after a crash
    calls = []
    for step in ('heartbeat_start', 'stand', 'sit'):
        setattr(Bot, step, lambda self, step=step: calls.append(step))
    Bot.wait_ready = lambda self: calls.append('wait_ready') or True

    def crash(b):
        with b.upright():
            boom()
    r, _, _ = run(crash)
    assert isinstance(r, ZeroDivisionError) and calls == ['heartbeat_start', 'wait_ready', 'stand', 'sit'], calls

    # a Go: one action client however many goals, and three ways it ends
    made = pretend_nav2(arrives=2.0)
    r, _, _ = run(lambda b: [b.goto(1.0, check=False) for _ in range(3)])
    assert r == [nav.SUCCEEDED] * 3 and len(made) == 1, (r, made)
    pretend_nav2(arrives=None)                                   # a Nav2 that never gets there
    r, halted, _ = run(lambda b: b.goto(1.0, check=False), at=(1.0, lambda w: w.bot.goto_cancel()))
    assert r == nav.CANCELED and halted, r                       # cancelled from another thread
    r, halted, _ = run(lambda b: b.goto(1.0, check=False), at=(1.0, lambda w: setattr(w.bot, '_goal_handle', None)))
    assert r == nav.CANCELED and halted, r                       # estop() dropped it: no waiting on a dead Nav2
    r, halted, _ = run(lambda b: b.goto(1.0, check=False, timeout=5.0))
    assert isinstance(r, L.Lite3Error) and 'timed out' in str(r) and halted, r

    # stopping a stack this process launched does not wait on its zombie (real time, real process)
    stack = subprocess.Popen(['setsid', 'sleep', '60'])
    assert L.Lite3._wait(lambda: os.getpgid(stack.pid) == stack.pid, 2.0), 'setsid did not take'
    began = time.time()
    nav._kill_group(stack.pid, 6.0, L.Lite3._wait)
    quick, gone = time.time() - began < 1.5, not nav._alive(stack.pid)
    stack.kill()                                                 # only matters if that failed
    assert quick and gone, 'nav_stop() would stall'
    print('check_motion ok')


if __name__ == '__main__':
    main()

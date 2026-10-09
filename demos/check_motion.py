#!/usr/bin/env python3
"""check_motion.py - what the motion code promises, checked with NO robot.

    python3 ~/robot/demos/check_motion.py     # needs rclpy to import, nothing else

A fake node, a fake clock and a pretend Nav2 stand in for the robot, and no
packet or ROS message leaves this process, so it is safe to run anywhere -
next to a live robot too. Run it after touching Lite3._loop, _run, steer,
walk, turn, goto, go_to or nav_stop.
"""
import math
import os
import subprocess
import sys
import tempfile
import time
import types

import rclpy.action
from builtin_interfaces.msg import Time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import lite3 as L, locate, nav, places, protocol as P  # noqa: E402

P.send = lambda code, value=0: None         # mode packets stay here


class World:
    """Stands in for Lite3._node: a pose that follows the last Twist exactly,
    on a clock that only sleep() moves. at=(seconds, fn(world)) is something
    that happens on the way."""
    state = {'basic': L.STANDING, 'battery': 80}
    cloud, tilt, stick_time, at = None, (0.0, 0.0), -1.0, None
    odom_time = property(lambda self: self.odom_stopped or self.t)     # fresh until it stops
    odom_stopped = None
    # Map mode: the global costmap, and what robot/locate.py last said (with the
    # odometry of that moment). None of it in mapless mode.
    grid = loc = loc_stopped = None
    loc_odom = (0.0, 0.0, 0.0)
    loc_time = property(lambda self: self.t if self.loc_stopped is None else self.loc_stopped)

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
    floor = 'lab'                           # nor for the floor that is loaded


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


GOALS, CANCELS = [], []      # every goal pose the pretend Nav2 was sent, and when one was cancelled


def pretend_nav2(arrives):
    """rclpy's ActionClient, pretended. A goal ends when it is cancelled, or
    `arrives` seconds after it was sent, or (None) never. Returns the list of
    clients made."""
    made = []
    del GOALS[:], CANCELS[:]

    def client(node, action, name):
        goal = {}
        result = types.SimpleNamespace(
            done=lambda: goal['cancelled'] or (arrives is not None and node.t >= goal['sent'] + arrives),
            result=lambda: types.SimpleNamespace(status=nav.CANCELED if goal['cancelled'] else nav.SUCCEEDED))
        handle = types.SimpleNamespace(accepted=True, get_result_async=lambda: result,
                                       cancel_goal_async=lambda: CANCELS.append(node.t) or goal.update(cancelled=True))
        made.append(name)
        return types.SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda msg: GOALS.append(msg.pose) or goal.update(sent=node.t, cancelled=False) or types.SimpleNamespace(
                done=lambda: True, result=lambda: handle))

    rclpy.action.ActionClient = client
    sys.modules['nav2_msgs'] = sys.modules['nav2_msgs.action'] = types.SimpleNamespace(
        NavigateToPose=types.SimpleNamespace(Goal=types.SimpleNamespace))
    return made


def boom():
    raise ZeroDivisionError


def floor_costmap():
    """Nav2's global costmap in map mode: 10 x 10 m in the map frame, free
    but for a wall along x = 8 m."""
    cells = [0] * 10000
    for row in range(100):
        cells[row * 100 + 80] = 100
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id='map'), data=cells,
        info=types.SimpleNamespace(resolution=0.1, width=100, height=100, origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0))))


def on_floor(state=locate.LOCALIZED):
    """World settings for map mode: odometry reads (0, 0, 0) and locate.py has
    him at (5, 5) on the map, facing +y."""
    return {'grid': floor_costmap(), 'loc': locate.Reading(state, 5.0, 5.0, math.pi / 2, 0.1, 0.05, 0.95)}


def sent_goal():
    """(frame, x, y, yaw in degrees) of the goal the pretend Nav2 got last."""
    g = GOALS[-1]
    return (g.header.frame_id, round(g.pose.position.x, 3), round(g.pose.position.y, 3),
            round(math.degrees(2 * math.atan2(g.pose.orientation.z, g.pose.orientation.w))))


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

    # the odometry relay dies mid-walk: he stops, he does not walk on blind
    r, halted, _ = run(lambda b: b.walk(5.0, speed=0.5, stop_distance=None),
                       at=(1.0, lambda w: setattr(w, 'odom_stopped', w.t)))
    assert r['reason'].startswith('odometry stopped') and halted and r['moved'] < 1.5, r
    r, _, sent = run(lambda b: b.walk(1.0, stop_distance=None), odom_stopped=-10.0)
    assert isinstance(r, L.Lite3Error) and not sent, r           # and does not set off without it

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
    assert sent_goal() == ('odom', 1.0, 0.0, 0), sent_goal()     # mapless: goals are in odom, as ever
    pretend_nav2(arrives=None)                                   # a Nav2 that never gets there
    r, halted, _ = run(lambda b: b.goto(1.0, check=False), at=(1.0, lambda w: w.bot.goto_cancel()))
    assert r == nav.CANCELED and halted, r                       # cancelled from another thread
    r, halted, _ = run(lambda b: b.goto(1.0, check=False), at=(1.0, lambda w: setattr(w.bot, '_goal_handle', None)))
    assert r == nav.CANCELED and halted, r                       # estop() dropped it: no waiting on a dead Nav2
    r, halted, _ = run(lambda b: b.goto(1.0, check=False, timeout=5.0))
    assert isinstance(r, L.Lite3Error) and 'timed out' in str(r) and halted, r

    # --- map mode: Nav2 plans in the map frame, and so every goal must be in it ---
    pretend_nav2(arrives=2.0)
    r, _, _ = run(lambda b: b.goto(1.0, check=False), **on_floor())
    assert r == nav.SUCCEEDED and sent_goal() == ('map', 5.0, 6.0, 90), sent_goal()     # 1 m ahead of where he is on the map
    r, _, _ = run(lambda b: (b.cost_at(0.0, -3.05), b.cost_at(0.0, 0.0), b.cost_at(0.0, 9.0)), **on_floor())
    assert r == (100, 0, None), r           # odom points, looked up on the map: the wall at (8.05, 5), him, and (-4, 5) off it
    r, _, _ = run(lambda b: (b.cost_at(8.05, 5.0, frame='map'), b.cost_at(2.0, 5.0, frame='map')), **on_floor())
    assert r == (100, 0), r
    r, _, _ = run(lambda b: (b.localized, b.map_pose), **on_floor())
    assert r[0] is True and all(abs(a - b) < 1e-9 for a, b in zip(r[1], (5.0, 5.0, math.pi / 2))), r
    # he has walked 1 m since locate.py last spoke: the map pose has gone with him
    r, _, _ = run(lambda b: b.map_pose, odom=(1.0, 0.0, 0.0), **on_floor())
    assert all(abs(a - b) < 1e-9 for a, b in zip(r, (5.0, 6.0, math.pi / 2))), r
    r, _, _ = run(lambda b: (b.localized, b.map_pose))
    assert r == (False, None), r            # mapless: neither

    # go to a point on the map
    r, _, _ = run(lambda b: b.go_to_point(2.0, 5.0), **on_floor())
    assert r == nav.SUCCEEDED and sent_goal() == ('map', 2.0, 5.0, 180), sent_goal()    # facing the way he came
    r, _, _ = run(lambda b: (b.go_to_point(2.0, 5.0, yaw_deg=0), b._node.odom[2]), **on_floor())
    assert r[0] == nav.SUCCEEDED and abs(r[1] + math.pi / 2) < 0.1, r                   # and turned to the heading asked for
    r, _, _ = run(lambda b: b.go_to_point(5.05, 5.0), **on_floor())
    assert r == nav.SUCCEEDED and len(GOALS) == 3, (r, len(GOALS))                      # already there: no goal sent
    pretend_nav2(arrives=None)
    r, halted, _ = run(lambda b: b.go_to_point(2.0, 5.0), at=(1.0, lambda w: w.bot.goto_cancel()), **on_floor())
    assert r == nav.CANCELED and halted, r

    def refused(move, **setup):
        before = len(GOALS)
        r, _, sent = run(move, **setup)
        assert isinstance(r, L.Lite3Error) and not sent and len(GOALS) == before, r     # no goal, no step
        return str(r)

    Bot.floor = None
    assert 'no floor' in refused(lambda b: b.go_to_point(2.0, 5.0))
    Bot.floor = 'lab'
    assert 'fit the map' in refused(lambda b: b.go_to_point(2.0, 5.0), **on_floor(locate.NO_FIT))
    assert 'not been told' in refused(lambda b: b.go_to_point(2.0, 5.0), **on_floor(locate.NO_POSE))
    assert 'says nothing' in refused(lambda b: b.go_to_point(2.0, 5.0), loc_stopped=-10.0, **on_floor())
    assert 'outside the map' in refused(lambda b: b.go_to_point(20.0, 5.0), **on_floor())
    assert 'wall' in refused(lambda b: b.go_to_point(8.05, 5.0), **on_floor())
    assert 'numbers' in refused(lambda b: b.go_to_point(float('nan'), 5.0), **on_floor())
    assert 'not standing' in refused(lambda b: b.go_to_point(2.0, 5.0), state={'basic': 1, 'battery': 80}, **on_floor())

    # lost on the way: the goal is dropped and he stops, he does not walk on a wrong position
    r, halted, _ = run(lambda b: b.go_to_point(2.0, 5.0),
                       at=(1.0, lambda w: setattr(w, 'loc', w.loc._replace(state=locate.NO_FIT))), **on_floor())
    assert isinstance(r, L.Lite3Error) and 'lost' in str(r) and 'fit the map' in str(r) and halted and CANCELS, r
    del CANCELS[:]
    r, halted, _ = run(lambda b: b.go_to_point(2.0, 5.0), at=(1.0, lambda w: setattr(w, 'loc_stopped', w.t)), **on_floor())
    assert isinstance(r, L.Lite3Error) and 'lost' in str(r) and halted and CANCELS, r   # locate.py itself died

    # places: saved where he stands or at a point, gone to by name
    pretend_nav2(arrives=2.0)
    real_root = places.ROOT
    with tempfile.TemporaryDirectory() as places.ROOT:
        os.makedirs(os.path.join(places.ROOT, 'lab'))
        open(os.path.join(places.ROOT, 'lab', 'map.yaml'), 'w').close()

        def tour(b):
            here = b.save_place('Start')
            b.save_place('Window', 2.0, 5.0, 180)
            return here, b.go_to('window'), [p['name'] for p in b.places()], b.delete_place('START'), b.places()
        r, _, _ = run(tour, **on_floor())
        assert (r[0]['x'], r[0]['y'], r[0]['yaw_deg']) == (5.0, 5.0, 90.0), r
        assert r[1] == nav.SUCCEEDED and sent_goal() == ('map', 2.0, 5.0, 180) and r[2] == ['Start', 'Window'], r
        assert [p['name'] for p in r[4]] == ['Window'], r
        assert 'no place' in refused(lambda b: b.go_to('canteen'), **on_floor())
        assert 'fit the map' in refused(lambda b: b.save_place('Nowhere'), **on_floor(locate.NO_FIT))
        assert 'wall' in refused(lambda b: b.save_place('In the wall', 8.05, 5.0, 0), **on_floor())
    places.ROOT = real_root

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

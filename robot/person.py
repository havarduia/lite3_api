#!/usr/bin/env python3
"""Find a person with the robot's built-in tracker and walk up to them.

Only the tracker's detection is used; driving goes through lite3.steer()
with the depth-camera stop. README.md section 8.

    python3 -m robot.person      # print where the person is, no movement
    from robot.person import PersonDetector, approach
"""
import json
import math
import os
import select
import socket
import struct
import threading
import time

from .depth import sampled
from .nav import LETHAL, clamp, dist
from .protocol import (TRACKER_ADDR, TRK_DETECT, TRK_MODES, TRK_QUERY, TRK_TARGETS,
                       Lite3Error, tracker_packet)

WIDTH, HEIGHT = 1280.0, 720.0       # bbox pixel space
FRESH = 0.7                 # s: a detection older than this is not trusted
GIVE_UP = 5.0               # s without the person before approach() stops, and a follow stops looking round
FOLLOW_GIVE_UP = 15.0       # s before a follow ends: close up the tracker loses them (it sees legs only), and he should wait
CENTRED = 0.25              # |x - 0.5| under which he may walk forward: keeping up, not pointing
# Turn rates are what is commanded; he delivers about 0.7 of it.
K_TURN = 5.0                # rad/s per unit of x offset
MAX_TURN = 1.6              # rad/s, the fastest tried (lite3.MAX_YAW_RATE)
SEARCH_TURN = 1.0           # rad/s toward the side a person left the picture on: half a turn in GIVE_UP
AIM = 0.10                  # |x - 0.5| he calls facing them: no turning for less
RESUME = 0.5                # m past the stop distance before he walks again, once stopped there
SLOW_ZONE = 0.8             # m before the stop distance over which he slows down
MIN_SPEED = 0.12            # m/s at the end of that ramp
# follow_nav(): where the person is, to give Nav2 a goal beside them
CAM_FOV = math.radians(130)  # the front camera's picture, edge to edge: its rated angle, not measured here
PERSON_HALF_DEG = 7.5       # the depth bins this far either side of their bearing are them
REGOAL_M = 0.5              # a new goal once they have moved this far from the last one ...
REGOAL_S = 2.0              # ... or after this long, in case Nav2 gave the last one up
REGOAL_MIN_S = 1.0          # ... but never sooner than this: every new goal restarts Nav2's planning
SMOOTH = 0.4                # share of each new fix in where he takes them to be: single fixes jump 0.5 m
NAV_K_TURN = 3.5            # rad/s per unit of x offset, turning to face them
NAV_MAX_TURN = 1.2          # rad/s commanded for that (0.84 delivered). 1.6 overshot and lost them
DEPTH_VIEW_DEG = 40         # beyond this bearing the depth camera, and so Nav2's costmap, cannot see them
FACE_DEG = 30               # standing with no goal, he turns to them when they are further round than this ...
FACED_DEG = 15              # ... and until they are within this
PERSON_HEIGHT = 1.87        # m: the height of whoever he follows most. Their box's height gives a second estimate of the range
CAM_VFOV = CAM_FOV * 9 / 16  # the picture top to bottom
BOX_MATCH = 0.35            # a depth return within this share of the box's range is them, not furniture
# follow_nav() walks straight at them, as follow() does, while the depth camera shows the way free
LINE_HALF = 0.3             # m either side of the line to them that has to be empty
BLOCK_MARGIN = 0.5          # m: something this much nearer than they are is in the way, not them
BLOCKED_S = 0.3             # the way has to be blocked this long before Nav2 takes over
CLEAR_S = 1.0               # and free this long before he walks straight again
FOLLOW_LOG = '/tmp/follow_nav.log'   # every change of state in each follow_nav(), newest last
NAV_NEAR = 0.3              # m: a goal nearer than this is "there" to Nav2 (xy_goal_tolerance 0.25)


class PersonDetector(threading.Thread):
    """Switches the built-in detection on and keeps .latest up to date, locked
    onto the largest person's track id. stop() switches detection off.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.latest = None      # dict(t, x, left, right, top, bottom, id), all in 0..1
        self.ready = threading.Event()
        self.error = None
        self._halt = threading.Event()
        self._target = None
        self._target_seen = 0.0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('0.0.0.0', 0))

    def run(self):
        try:
            self._sock.sendto(tracker_packet(TRK_DETECT, enabled=1), TRACKER_ADDR)
            # With nobody in view he sends no targets at all: the answer to
            # this is what shows the tracker is there.
            self._sock.sendto(tracker_packet(TRK_QUERY), TRACKER_ADDR)
            deadline = time.time() + 5
            while not self._halt.is_set():
                r, _, _ = select.select([self._sock], [], [], 0.5)
                if not r:
                    if not self.ready.is_set() and time.time() > deadline:
                        raise RuntimeError('no reply from the tracker at %s:%d in 5 s'
                                           % TRACKER_ADDR)
                    continue
                d, _ = self._sock.recvfrom(8192)
                code = struct.unpack('<i', d[:4])[0] if len(d) > 12 else None
                if code == TRK_TARGETS:
                    self._update(json.loads(d[12:]).get('targets', []))
                if code in (TRK_TARGETS, TRK_MODES):
                    self.ready.set()
        except Exception as e:
            self.error = e
            self.ready.set()
        finally:
            self._sock.sendto(tracker_packet(TRK_DETECT, enabled=0), TRACKER_ADDR)

    def _update(self, targets):
        now = time.time()
        people = []
        for tg in targets:
            x1, y1, x2, y2 = tg['bbox']
            people.append(dict(t=now, x=(x1 + x2) / 2 / WIDTH, left=x1 / WIDTH,
                               right=x2 / WIDTH, top=y1 / HEIGHT,
                               bottom=y2 / HEIGHT, area=(x2 - x1) * (y2 - y1),
                               id=tg.get('id')))
        pick = next((p for p in people if p['id'] == self._target), None)
        if pick is None and people and now - self._target_seen > 1.0:
            pick = max(people, key=lambda p: p['area'])
            self._target = pick['id']
        if pick is not None:
            self._target_seen = now
            self.latest = pick

    def person(self):
        """The target if seen within FRESH seconds, else None."""
        p = self.latest
        return p if p and time.time() - p['t'] < FRESH else None

    def stop(self):
        self._halt.set()


def _swing(wz, sides, sweep):
    """wz, unless it would swing the body into something on that side."""
    left, right = sides
    return 0.0 if (wz > 0 and left < sweep) or (wz < 0 and right < sweep) else wz


def _controller(bot, det, stop_distance, speed, hold, near=None, near_distance=1.0,
                abort=None, view=None):
    """The per-cycle control behind approach() and follow(): turn to keep the
    person centred (never into something inside TURN_SWEEP), walk while they
    are centred, slow over the last SLOW_ZONE metres, and turn after a person
    who left the picture to one side. hold=True stays at the
    stop distance instead of ending there. abort() returning a reason ends it.
    view() -> (clearance, side_clear), for a caller that scans already.
    """
    from .lite3 import TURN_SWEEP         # here, not on top: lite3 needs ROS
    state = {'lost_since': None, 'near_done': near is None, 'off': 0.0,
             'arrived': False, 'aimed': False}

    def read():
        bins = bot.scan()                       # one scan for both checks
        return bot.clearance(bins=bins), bot.side_clear(bins)

    view = view or sampled(read)

    def control():
        now = time.time()
        why = abort and abort()
        if why:
            return why
        if det.error:
            return 'detector failed: %s' % det.error
        try:
            clear, sides = view()
        except Lite3Error:
            return 'lost the depth stream - stopped rather than walking blind'

        def swing(wz):
            return _swing(wz, sides, TURN_SWEEP)

        p = det.person()
        if p is None:
            state['lost_since'] = state['lost_since'] or now
            gone = now - state['lost_since']
            if gone > (FOLLOW_GIVE_UP if hold else GIVE_UP):
                return 'lost the person for %.0f s' % gone
            # Last seen off to one side: they walked out of the picture that
            # way, so look that way, for GIVE_UP. Lost near the middle, wait.
            last = state['off']
            return 0.0, swing(0.0 if abs(last) < CENTRED or gone > GIVE_UP
                              else -SEARCH_TURN if last > 0 else SEARCH_TURN)
        state['lost_since'] = None
        if not state['near_done'] and clear <= stop_distance + near_distance:
            state['near_done'] = True
            near()
        off = state['off'] = p['x'] - 0.5         # +ve = person to the right
        wz = swing(clamp(-K_TURN * off, MAX_TURN)) if abs(off) >= AIM else 0.0
        if clear <= stop_distance + (RESUME if state['arrived'] else 0.0):
            if not hold:
                return 'reached: %.2f m from body centre' % clear
            # In front of them he stands still: any velocity at all keeps him
            # stepping, so both leaving again and re-aiming need a real error.
            state['arrived'] = True
            state['aimed'] = abs(off) < (CENTRED if state['aimed'] else AIM)
            return 0.0, 0.0 if state['aimed'] else wz
        state['arrived'] = state['aimed'] = False
        ramp = (clear - stop_distance) / SLOW_ZONE
        # The ramp's floor never lifts him above the speed that was asked for.
        vx = max(min(MIN_SPEED, speed), speed * min(1.0, ramp)) if abs(off) < CENTRED else 0.0
        return vx, wz

    return control


def approach(bot, det, stop_distance=0.6, speed=0.3, limit=30.0,
             near=None, near_distance=1.0, abort=None):
    """Turn to and walk up to the detected person, stopping stop_distance (from
    the body centre) short. near(), if given, is called once from the control
    loop when he is within near_distance of that. Returns steer()'s result.
    """
    return bot.steer(_controller(bot, det, stop_distance, speed, False,
                                 near, near_distance, abort), limit=limit)


def follow(bot, det, seconds, stop_distance=0.6, speed=0.3, abort=None):
    """Follow the person for `seconds` (at most lite3's 30 s HARD_TIMEOUT),
    keeping stop_distance; ends early if they are lost for FOLLOW_GIVE_UP seconds.
    Returns lite3.steer()'s result dict ('time limit' = followed the full time).
    """
    return bot.steer(_controller(bot, det, stop_distance, speed, True, abort=abort),
                     limit=seconds)


def box_range(p):
    """Metres to a person from how tall their box is in the picture, None if
    the box runs off the top or bottom (they are close) so its height is not
    theirs. Good to about PERSON_HEIGHT's error: 10-15 %."""
    h = p['bottom'] - p['top']
    if p['top'] < 0.02 or p['bottom'] > 0.98 or h < 0.05:
        return None
    return PERSON_HEIGHT / 2 / math.tan(h * CAM_VFOV / 2)


def person_range(bins, bearing_deg, guess=None):
    """Metres to the person at that bearing. The depth scan `bins` is exact
    but shows the nearest thing, which is often a chair; `guess` (box_range)
    is rough but is them. So: the depth return nearest the guess if one is
    within BOX_MATCH of it, else the guess. With no guess, the nearest depth
    return, or None."""
    near = [r for b, r in bins if r is not None and abs(b - bearing_deg) <= PERSON_HALF_DEG]
    if guess is None:
        return min(near) if near else None
    best = min(near, key=lambda r: abs(r - guess), default=None)
    return best if best is not None and abs(best - guess) <= BOX_MATCH * guess else guess


def follow_nav(bot, det, seconds, stop_distance=0.6, speed=0.3, abort=None):
    """follow() while the straight line to the person is free, and Nav2 goals
    round whatever is on it while it is not. Needs Nav2 running. speed is for
    the straight part; Nav2 sets its own pace. Returns a result like
    steer()'s, plus 'spent': seconds in each state, which is also traced to
    FOLLOW_LOG. README.md section 8.
    """
    from .lite3 import HARD_TIMEOUT, MAX_SPEED, MAX_YAW_RATE, TURN_SWEEP
    if not bot.nav_running:
        raise Lite3Error('nav2 is not running - call nav_start() first')
    # His own time limit, a second inside _loop's: _loop ends with a halt, and
    # the goal has to be gone before that.
    seconds = min(seconds, HARD_TIMEOUT - 1.0)
    view = sampled(bot.scan)
    state = {'lost_since': None, 'off': 0.0, 'arrived': False,
             'goal': None, 'label': None, 'since': time.time(), 'sent': 0, 'began': None,
             'them': None, 'facing': False, 'via_nav': False, 'changed': None, 'bins': None}
    # the straight part: follow()'s own controller, on the scan taken here
    straight = _controller(bot, det, stop_distance, speed, True, view=lambda: (
        bot.clearance(bins=state['bins']), bot.side_clear(state['bins'])))
    spent = {}
    # Appended, so an earlier run is still there to read; started afresh past 1 MB.
    big = os.path.exists(FOLLOW_LOG) and os.path.getsize(FOLLOW_LOG) > 1000000
    trace = open(FOLLOW_LOG, 'w' if big else 'a')
    trace.write('--- follow_nav %s\n' % time.strftime('%H:%M:%S'))

    def note(label, detail=''):
        """Count the time in each state and write every change of it down."""
        now = time.time()
        spent[state['label']] = spent.get(state['label'], 0.0) + now - state['since']
        state['since'] = now
        if label != state['label']:
            state['label'] = label
            trace.write('%.2f %s %s\n' % (now, label, detail))
            trace.flush()

    def drop():
        bot.goto_cancel()
        bot._goal_handle = None             # as goto() leaves it: nothing of ours to cancel later
        state['goal'] = None

    def cycle(start):
        why = step()
        if why:
            drop()                          # before _loop's halt: Nav2 must not drive against it
        return why

    def free_goal(x, y, heading, d):
        """The point d metres out on that heading, pulled back toward him until
        Nav2 can plan into it: the person's own trail lingers in the costmap."""
        while d >= NAV_NEAR:
            gx, gy = x + d * math.cos(heading), y + d * math.sin(heading)
            c = bot.cost_at(gx, gy)
            if c is not None and c < LETHAL:
                return gx, gy
            d -= 0.1
        return None

    def way_clear(bins, deg):
        """Metres the depth scan shows free toward them: the nearer of what is
        straight ahead of him (where the straight controller walks) and what
        is on the line to them. The camera, not the costmap: the costmap
        keeps their own trail and called a free line blocked."""
        on_line = [r * math.cos(math.radians(b - deg)) for b, r in bins if r is not None
                   and abs(r * math.sin(math.radians(b - deg))) <= LINE_HALF and abs(b - deg) < 90]
        return min([bot.clearance(bins=bins)] + on_line)

    def step():
        now = time.time()
        state['began'] = state['began'] or now
        why = ((abort and abort()) or (det.error and 'detector failed: %s' % det.error)
               or (now - state['began'] >= seconds and 'time limit'))
        if why:
            return why
        try:
            bins = state['bins'] = view()
        except Lite3Error:
            return 'lost the depth stream - stopped rather than walking blind'
        wz = 0.0
        if state['goal'] and not bot.goal_active():
            state['goal'] = None                      # Nav2 has finished with it, one way or the other
        p = det.person()
        if p is not None:
            # Straight or by Nav2? Only judged while he sees them: a goal
            # under way when they vanish leads to where they went.
            off = p['x'] - 0.5
            bearing = -off * CAM_FOV
            r = person_range(bins, math.degrees(bearing), box_range(p))
            deg = math.degrees(bearing)
            free = way_clear(bins, deg)
            blocked = r is not None and abs(deg) <= DEPTH_VIEW_DEG and free < r - BLOCK_MARGIN
            if blocked != state['via_nav'] and state['changed'] is None:
                trace.write('%.2f   way %s: %.2f m free, them %.2f m at %+.0f deg\n' % (
                    now, 'blocked' if blocked else 'clear', free, r or -1, deg))
            if blocked == state['via_nav']:
                state['changed'] = None
            else:
                state['changed'] = state['changed'] or now
                if now - state['changed'] >= (BLOCKED_S if blocked else CLEAR_S):
                    state['via_nav'], state['changed'] = blocked, None
                    state['arrived'], state['them'] = False, None
                    if not blocked and state['goal']:
                        drop()
        if not state['via_nav']:
            out = straight()
            if isinstance(out, str):
                return out
            # what the straight controller is doing, and on what it sees
            if p is None:
                note('straight: not seen')
            else:
                note('straight: ' + ('walking' if out[0] else 'turning' if out[1] else 'standing'),
                     'ahead %.2f m, sides %.2f / %.2f m, them %s at %+.0f deg' % (
                         (bot.clearance(bins=bins),) + bot.side_clear(bins)
                         + ('-' if r is None else '%.2f m' % r, deg)))
            bot._drive(clamp(out[0], MAX_SPEED), 0.0, clamp(out[1], MAX_YAW_RATE))
            time.sleep(0.05)
            return None
        if p is None:
            state['lost_since'] = state['lost_since'] or now
            gone = now - state['lost_since']
            if gone > FOLLOW_GIVE_UP:
                return 'lost the person for %.0f s' % gone
            note('not seen')
            # A goal under way stays: it leads to where they were last seen.
            if not state['goal'] and abs(state['off']) >= CENTRED and gone <= GIVE_UP:
                wz = -SEARCH_TURN if state['off'] > 0 else SEARCH_TURN
        else:
            state['lost_since'] = None
            off = state['off'] = p['x'] - 0.5         # +ve = person to the right
            bearing = -off * CAM_FOV                  # +ve = left, as the depth bins
            deg = math.degrees(bearing)
            guess = box_range(p)
            r = person_range(bins, deg, guess)
            said = 'them %s (box %s) at %+.0f deg' % (
                '-' if r is None else '%.2f m' % r, '-' if guess is None else '%.2f m' % guess, deg)
            if abs(deg) > DEPTH_VIEW_DEG:
                # Beside him: the depth camera cannot see them, so neither can
                # Nav2, and a goal there could walk him into them. The goal he
                # had leads to where they no longer are: drop it and turn.
                note('beside him', said)
                if state['goal']:
                    drop()
            elif r is not None and r <= stop_distance + (RESUME if state['arrived'] else NAV_NEAR):
                # With them he stands. He need not point at them exactly, only
                # keep them where the depth camera sees them (FACE_DEG).
                note('with them', said)
                if state['goal']:
                    drop()
                state['arrived'], state['them'] = True, None
            elif r is None:
                note('no range on them', said)
            else:
                state['arrived'] = False
                x, y, yaw = bot.pose
                fix = (x + r * math.cos(yaw + bearing), y + r * math.sin(yaw + bearing))
                was = state['them']
                them = state['them'] = fix if was is None else (
                    was[0] + SMOOTH * (fix[0] - was[0]), was[1] + SMOOTH * (fix[1] - was[1]))
                heading = math.atan2(them[1] - y, them[0] - x)
                at = free_goal(x, y, heading, dist((x, y), them) - stop_distance)
                if at:
                    note('goal', said)
                    g = state['goal']
                    if g is None or now - g[2] > REGOAL_S or (
                            dist(g, at) > REGOAL_M and now - g[2] > REGOAL_MIN_S):
                        bot.goal_send(at[0], at[1], heading)
                        state['goal'] = (at[0], at[1], now)
                        state['sent'] += 1
                        trace.write('%.2f   sent (%.2f, %.2f), he is at (%.2f, %.2f)\n' % (now, at[0], at[1], x, y))
                else:
                    note('goal blocked', said)
            if not state['goal']:
                # Standing with no goal: turn to them once they are well round
                # to one side, and stop well before they are dead ahead.
                state['facing'] = abs(deg) > (FACED_DEG if state['facing'] else FACE_DEG)
                if state['facing']:
                    wz = clamp(-NAV_K_TURN * off, NAV_MAX_TURN)
        if not state['goal']:
            # Only with no goal: Nav2's controller owns cmd_vel while it has one.
            bot._drive(0.0, 0.0, _swing(wz, bot.side_clear(bins), TURN_SWEEP))
        time.sleep(0.05)

    try:
        r = bot._loop(cycle, seconds + 1.0, False)
    finally:
        drop()
        bot.halt()
        note(None)
        trace.write('--- stopped %s\n' % time.strftime('%H:%M:%S'))
        trace.close()
    spent.pop(None, None)
    r['spent'] = spent
    r['reason'] += ' (%d goals; %s)' % (state['sent'], ', '.join(
        '%.0f s %s' % (t, k) for k, t in sorted(spent.items(), key=lambda kv: -kv[1])))
    with open(FOLLOW_LOG, 'a') as trace:
        trace.write('--- ended: %s\n' % r['reason'])
    r['moved'] = dist(r['start'], r['end'])
    return r


if __name__ == '__main__':
    det = PersonDetector()
    det.start()
    det.ready.wait(6)
    if det.error:
        raise SystemExit('detector failed: %s' % det.error)
    try:
        while True:
            p = det.person()
            if p:
                print('person id %s: x %.2f (%s), top %.2f, bottom %.2f'
                      % (p['id'], p['x'], 'right' if p['x'] > 0.5 else 'left',
                         p['top'], p['bottom']), flush=True)
            else:
                print('no person', flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        det.stop()
        det.join(2)

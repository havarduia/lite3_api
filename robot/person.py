#!/usr/bin/env python3
"""Find a person with the robot's built-in tracker and walk up to them.

Only the tracker's detection is used; driving goes through lite3.steer()
with the depth-camera stop. README.md section 8.

    python3 -m robot.person      # print where the person is, no movement
    from robot.person import PersonDetector, approach
"""
import json
import select
import socket
import struct
import threading
import time

from .depth import sampled
from .nav import clamp
from .protocol import (TRACKER_ADDR, TRK_DETECT, TRK_MODES, TRK_QUERY, TRK_TARGETS,
                       Lite3Error, tracker_packet)

WIDTH, HEIGHT = 1280.0, 720.0       # bbox pixel space
FRESH = 0.7                 # s: a detection older than this is not trusted
GIVE_UP = 5.0               # s without the person before approach() stops
CENTRED = 0.25              # |x - 0.5| under which he may walk forward: keeping up, not pointing
# Turn rates are what is commanded; he delivers about 0.7 of it.
K_TURN = 5.0                # rad/s per unit of x offset
MAX_TURN = 1.6              # rad/s, the fastest tried (lite3.MAX_YAW_RATE)
SEARCH_TURN = 1.0           # rad/s toward the side a person left the picture on: half a turn in GIVE_UP
AIM = 0.10                  # |x - 0.5| he calls facing them: no turning for less
RESUME = 0.5                # m past the stop distance before he walks again, once stopped there
SLOW_ZONE = 0.8             # m before the stop distance over which he slows down
MIN_SPEED = 0.12            # m/s at the end of that ramp


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


def _controller(bot, det, stop_distance, speed, hold, near=None, near_distance=1.0,
                abort=None):
    """The per-cycle control behind approach() and follow(): turn to keep the
    person centred (never into something inside TURN_SWEEP), walk while they
    are centred, slow over the last SLOW_ZONE metres, and turn after a person
    who left the picture to one side. hold=True stays at the
    stop distance instead of ending there. abort() returning a reason ends it.
    """
    from .lite3 import TURN_SWEEP         # here, not on top: lite3 needs ROS
    state = {'lost_since': None, 'near_done': near is None, 'off': 0.0,
             'arrived': False, 'aimed': False}

    def read():
        bins = bot.scan()                       # one scan for both checks
        return bot.clearance(bins=bins), bot.side_clear(bins)

    view = sampled(read)

    def control():
        now = time.time()
        why = abort and abort()
        if why:
            return why
        if det.error:
            return 'detector failed: %s' % det.error
        try:
            clear, (left, right) = view()
        except Lite3Error:
            return 'lost the depth stream - stopped rather than walking blind'

        def swing(wz):
            """wz, unless it would swing the body into something on that side."""
            return 0.0 if (wz > 0 and left < TURN_SWEEP) or (wz < 0 and right < TURN_SWEEP) else wz

        p = det.person()
        if p is None:
            state['lost_since'] = state['lost_since'] or now
            if now - state['lost_since'] > GIVE_UP:
                return 'lost the person for %.0f s' % GIVE_UP
            # Last seen off to one side: they walked out of the picture that
            # way, so look that way. Lost near the middle, wait where he is.
            last = state['off']
            return 0.0, swing(0.0 if abs(last) < CENTRED else -SEARCH_TURN if last > 0 else SEARCH_TURN)
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
    keeping stop_distance; ends early if they are lost for GIVE_UP seconds.
    Returns lite3.steer()'s result dict ('time limit' = followed the full time).
    """
    return bot.steer(_controller(bot, det, stop_distance, speed, True, abort=abort),
                     limit=seconds)


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

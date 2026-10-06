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
from .protocol import TRACKER_ADDR, TRK_DETECT, TRK_TARGETS, Lite3Error, tracker_packet

WIDTH, HEIGHT = 1280.0, 720.0       # bbox pixel space
FRESH = 0.7                 # s: a detection older than this is not trusted
GIVE_UP = 5.0               # s without the person before approach() stops
CENTRED = 0.12              # |x - 0.5| under which he may walk forward
K_TURN = 2.4                # rad/s per unit of x offset
MAX_TURN = 0.6              # rad/s
SLOW_ZONE = 0.8             # m before the stop distance over which he slows down
MIN_SPEED = 0.12            # m/s at the end of that ramp


class PersonDetector(threading.Thread):
    """Switches the built-in detection on and keeps .latest up to date, locked
    onto the largest person's track id. stop() switches detection off.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.latest = None      # dict(t, x, top, bottom, id) - x, top, bottom in 0..1
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
            deadline = time.time() + 5
            while not self._halt.is_set():
                r, _, _ = select.select([self._sock], [], [], 0.5)
                if not r:
                    if not self.ready.is_set() and time.time() > deadline:
                        raise RuntimeError('no reply from the tracker at %s:%d in 5 s'
                                           % TRACKER_ADDR)
                    continue
                d, _ = self._sock.recvfrom(8192)
                if len(d) > 12 and struct.unpack('<i', d[:4])[0] == TRK_TARGETS:
                    self._update(json.loads(d[12:]).get('targets', []))
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
            people.append(dict(t=now, x=(x1 + x2) / 2 / WIDTH, top=y1 / HEIGHT,
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


def _controller(bot, det, stop_distance, speed, hold, near=None, near_distance=1.0):
    """The per-cycle control behind approach() and follow(): turn to keep the
    person centred (never into something inside TURN_SWEEP), walk while they
    are centred, slow over the last SLOW_ZONE metres. hold=True stays at the
    stop distance instead of ending there.
    """
    from .lite3 import TURN_SWEEP         # here, not on top: lite3 needs ROS
    state = {'lost_since': None, 'near_done': near is None}

    def read():
        bins = bot.scan()                       # one scan for both checks
        return bot.clearance(bins=bins), bot.side_clear(bins)

    view = sampled(read)

    def control():
        now = time.time()
        if det.error:
            return 'detector failed: %s' % det.error
        p = det.person()
        if p is None:
            state['lost_since'] = state['lost_since'] or now
            if now - state['lost_since'] > GIVE_UP:
                return 'lost the person for %.0f s' % GIVE_UP
            return 0.0, 0.0
        state['lost_since'] = None
        try:
            clear, (left, right) = view()
        except Lite3Error:
            return 'lost the depth stream - stopped rather than walking blind'
        if not state['near_done'] and clear <= stop_distance + near_distance:
            state['near_done'] = True
            near()
        off = p['x'] - 0.5                        # +ve = person to the right
        wz = clamp(-K_TURN * off, MAX_TURN)
        # Don't swing the body into something on the side he'd turn toward.
        if (wz > 0 and left < TURN_SWEEP) or (wz < 0 and right < TURN_SWEEP):
            wz = 0.0
        if clear <= stop_distance:
            if not hold:
                return 'reached: %.2f m from body centre' % clear
            return 0.0, wz
        ramp = (clear - stop_distance) / SLOW_ZONE
        vx = max(MIN_SPEED, speed * min(1.0, ramp)) if abs(off) < CENTRED else 0.0
        return vx, wz

    return control


def approach(bot, det, stop_distance=0.6, speed=0.3, limit=30.0,
             near=None, near_distance=1.0):
    """Turn to and walk up to the detected person, stopping stop_distance (from
    the body centre) short. near(), if given, is called once from the control
    loop when he is within near_distance of that. Returns steer()'s result.
    """
    return bot.steer(_controller(bot, det, stop_distance, speed, False,
                                 near, near_distance), limit=limit)


def follow(bot, det, seconds, stop_distance=0.6, speed=0.3):
    """Follow the person for `seconds` (at most lite3's 30 s HARD_TIMEOUT),
    keeping stop_distance; ends early if they are lost for GIVE_UP seconds.
    Returns lite3.steer()'s result dict ('time limit' = followed the full time).
    """
    return bot.steer(_controller(bot, det, stop_distance, speed, True),
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

#!/usr/bin/env python3
"""Find a person with the robot's built-in tracker and walk up to them.

The detector is the `track` service on the MOTION computer (RK3588 NPU,
YOLOv5s + tracker, always running, the one behind the app's follow button).
It speaks UDP on 192.168.1.120:43901: a 12-byte header <code, json_len, 1>
then JSON. Decoded from a capture of the phone app (2026-09-18):

    0x21013301  {"enabled":0|1}                  video streaming
    0x21013302  {"enabled":0|1}                  person detection
    0x21013303  {"targetID":n,"enabled":0|1}     built-in follow - NOT used here
    0x21013304  <- {"targets":[{"id","following","bbox":[x1,y1,x2,y2]}]}
                   ~27/s while detection is on, bbox in 1280x720 pixels
    0x21013305  {}  state query  ->  0x21013306 {"modes":{...}}

Replies go to the sender's own address and port. Only detection is used:
the built-in follow drives at up to 1.0 m/s with no depth check, so driving
goes through lite3.steer() instead, with the depth-camera stop.

    python3 -m robot.person      # print where the person is, no movement
    from robot.person import PersonDetector, approach
"""
import json
import select
import socket
import struct
import threading
import time

from .protocol import TRACKER_ADDR as TRACKER, TRK_DETECT as DETECT, \
    TRK_TARGETS as TARGETS, tracker_packet as _msg
WIDTH, HEIGHT = 1280.0, 720.0       # bbox pixel space
FRESH = 0.7                 # s: a detection older than this is not trusted
GIVE_UP = 5.0               # s without the person before approach() stops
CENTRED = 0.12              # |x - 0.5| under which he may walk forward
K_TURN = 2.4                # rad/s per unit of x offset
MAX_TURN = 0.6              # rad/s
SLOW_ZONE = 0.8             # m before the stop distance over which he slows down
MIN_SPEED = 0.12            # m/s at the end of that ramp


class PersonDetector(threading.Thread):
    """Switches the built-in detection on and keeps .latest up to date.

    Locks onto the largest (usually nearest) person's track id and follows
    that id; if it vanishes for over a second, re-locks on the largest.
    stop() switches detection off again.
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
            self._sock.sendto(_msg(DETECT, enabled=1), TRACKER)
            deadline = time.time() + 5
            while not self._halt.is_set():
                r, _, _ = select.select([self._sock], [], [], 0.5)
                if not r:
                    if not self.ready.is_set() and time.time() > deadline:
                        raise RuntimeError('no reply from the tracker at %s:%d in 5 s'
                                           % TRACKER)
                    continue
                d, _ = self._sock.recvfrom(8192)
                if len(d) > 12 and struct.unpack('<i', d[:4])[0] == TARGETS:
                    self._update(json.loads(d[12:]).get('targets', []))
                    self.ready.set()
        except Exception as e:
            self.error = e
            self.ready.set()
        finally:
            self._sock.sendto(_msg(DETECT, enabled=0), TRACKER)

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
    """The shared per-cycle control for approach() and follow().

    Turns to keep the person centred and walks only while they are roughly
    centred - but never turns toward a side with something inside the
    turning sweep (lite3.TURN_SWEEP). Slows linearly over the last SLOW_ZONE metres: clearance is only
    sampled at 4 Hz, so at full speed he would coast past the stop distance.
    At the stop distance, approach() ends; follow() (hold=True) stays put,
    still turning to face them, and walks again when they move away.
    """
    from .lite3 import Lite3Error, TURN_SWEEP
    state = {'lost_since': None, 'last_clear': 0.0, 'clear': float('inf'),
             'sides': (float('inf'), float('inf')), 'near_done': near is None}

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
        if now - state['last_clear'] > 0.25:     # scan() walks the cloud
            state['last_clear'] = now
            try:
                bins = bot.scan()
                state['clear'] = bot.clearance(bins=bins)
                state['sides'] = bot.side_clear(bins)
            except Lite3Error:
                return 'lost the depth stream - stopped rather than walking blind'
        if not state['near_done'] and state['clear'] <= stop_distance + near_distance:
            state['near_done'] = True
            near()
        off = p['x'] - 0.5                        # +ve = person to the right
        wz = max(-MAX_TURN, min(MAX_TURN, -K_TURN * off))
        # Don't swing the body into something on the side he'd turn toward.
        left, right = state['sides']
        if (wz > 0 and left < TURN_SWEEP) or (wz < 0 and right < TURN_SWEEP):
            wz = 0.0
        if state['clear'] <= stop_distance:
            if not hold:
                return 'reached: %.2f m from body centre' % state['clear']
            return 0.0, wz
        ramp = (state['clear'] - stop_distance) / SLOW_ZONE
        vx = max(MIN_SPEED, speed * min(1.0, ramp)) if abs(off) < CENTRED else 0.0
        return vx, wz

    return control


def approach(bot, det, stop_distance=0.6, speed=0.3, limit=30.0,
             near=None, near_distance=1.0):
    """Turn to and walk up to the detected person; stop at the depth guard.

    stop_distance is from the BODY CENTRE like walk()'s (nose ~0.33 m ahead).
    near(), if given, is called once (from the control loop - keep it quick,
    e.g. start a thread) when he gets within near_distance metres of the stop
    point, so a reaction can be prepared before he arrives.
    Returns lite3.steer()'s result dict.
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

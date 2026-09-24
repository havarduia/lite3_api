#!/usr/bin/env python3
"""The robot walks a route and says what he sees along the way (--persona).

    python3 ~/robot/bin/tour.py                          # walk 2 m forward, commenting
    python3 ~/robot/bin/tour.py "walk 1.5, turn 90, walk 1, turn -90"
    python3 ~/robot/bin/tour.py "goto 2.5, goto 1.5 90"  # Nav2: routes around obstacles
    python3 ~/robot/bin/tour.py --interactive            # type steps one at a time
    python3 ~/robot/bin/tour.py --speed 0.2 "walk 3"
    python3 ~/robot/bin/tour.py --dry-run                # comments only, no movement
    python3 ~/robot/bin/tour.py --finale "roast the person in front of you" "goto 2.8"

Steps (comma separated, or one per line with --interactive):
    walk <m>            straight line, blind to routing, stops for obstacles
    turn <deg>          in place, +ve = left
    goto <m> [<deg>]    Nav2 goal <m> ahead of where he faces now, then turn
                        <deg> (+ve = left). Plans around obstacles. If that
                        spot is inside an obstacle, goes to the free distance
                        CLOSEST to <m> on that line instead, so he still moves.
    approach <m>        Nav2 toward <m> ahead, stopping short of the FIRST
                        obstacle on that line.
    person              turn to and walk up to the nearest PERSON the camera
                        sees (the robot's built-in tracker, see person.py),
                        stopping at --stop.
    follow <s>          follow that person for <s> seconds, holding --stop
                        from them; every --talk-every seconds he stops and
                        says what he sees (line prepared in the background just
                        before, so he speaks at once). Ends if he loses them.

Movement is lite3.py's: the robot stands first, walk() stops for obstacles
within --stop metres using the depth camera (so realsense_ros2.service must be
running; --blind walks without that guard), Ctrl-C is an e-stop (it also kills
Nav2), and it sits again at the end. If any step is a goto - or with
--interactive - mapless Nav2 is started after standing (it needs the pose
settled first) and stopped before sitting. It sends its own controller
heartbeat by default, so no handheld is needed - which also means Ctrl-C and
the power switch are the only e-stops. --handheld uses the real controller.

A step that fails (goal in an obstacle, planner abort, obstacle at the start)
is reported and skipped; the API has already halted the robot by then.

He only talks while STANDING STILL - the motors are too loud to hear him
while walking: after each step he stops, looks (front camera -> Gemini Live ->
Piper) and comments, then the next step starts. With --finale the last pause
is the finale instead. A failed look is just printed and skipped.
"""
import argparse
import contextlib
import math
import sys
import threading
import time

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.talk import Talker

PAUSED = ('You just stopped for a moment during your walk. In one or two short '
          'sentences, say what you see ahead of you and what you think of it. Do '
          'not repeat what you already said unless something changed. Only '
          'describe what you see; do not announce moves, the walking is not yours '
          'to choose.')
FOLLOWING = ('You are following your human friend around. You just stopped '
             'for a moment. In one or two short sentences, say what you see or '
             'think right now. Do not repeat yourself.')
PREP_LEAD = 5.0     # s before a talking stop to start writing the line
PREP_CONTEXT = ('(You are still walking up to them and will say this the moment '
                'you arrive, looking up at them. Say it as if you are already '
                'there.) ')
DONE = ('You just finished your walk; you covered %.1f metres in total. '
        'Say one short sentence about it.')
NAV_STATUS = {4: 'arrived', 5: 'cancelled', 6: 'aborted - no path'}
# Plannable costmap cost: below 99 (inscribed/LETHAL), as goto() itself
# checks. 50 was tried first and wrongly refused open floor between two tables,
# whose inflation spreads 50-90 over the whole gap.
FREE_ENOUGH = 99
APPROACH_MARGIN = 0.2  # stop this far short of the last free cell
MIN_GOAL = 0.5         # xy_goal_tolerance is 0.25: a nearer goal "arrives" at once and he only turns


def settled_profile(bot, out_to, tries=10):
    """cost_ahead() once it stops changing. Same call as before; the waiting
    now lives in Lite3.cost_ahead(settle=...) so shake.py shares it."""
    return bot.cost_ahead(out_to=out_to, step=0.1, settle=tries)


def parse_step(step):
    """'walk 1.5' -> ('walk', [1.5]); 'goto 2 90' -> ('goto', [2.0, 90.0])"""
    verb, *args = step.split()
    nargs = {'walk': (1,), 'turn': (1,), 'goto': (1, 2), 'approach': (1,),
             'person': (0,), 'follow': (1,)}
    if verb not in nargs or len(args) not in nargs[verb]:
        raise ValueError('bad step %r: use "walk <m>", "turn <deg>", '
                         '"goto <m> [<deg>]", "approach <m>", "person" or '
                         '"follow <s>"' % step)
    return verb, [float(x) for x in args]


def parse_route(text):
    try:
        return [parse_step(s) for s in filter(None, (s.strip() for s in text.split(',')))]
    except ValueError as e:
        raise SystemExit(e)


class Odometer(threading.Thread):
    """Path length walked, sampled at 2 Hz, so the closing line is true.

    Summing start-to-end per step undercounted a 14 m follow as 7.2 m (and
    before any counting he announced "five meters" after moving 0.09 m).
    Steps under 2 cm are ignored, so standing-still odometry noise doesn't add up.
    """

    def __init__(self, bot):
        super().__init__(daemon=True)
        self.bot, self.total, self.last = bot, 0.0, None

    def reset(self):
        self.last = self.bot.wait_pose()[:2]
        if not self.is_alive():
            self.start()

    def run(self):
        while True:
            time.sleep(0.5)
            self.sample()

    def sample(self):
        if self.last is None:
            return
        p = self.bot.pose
        if p is None:
            return
        d = math.hypot(p[0] - self.last[0], p[1] - self.last[1])
        if d >= 0.02:
            self.total += d
            self.last = p[:2]


def comment(talker, prompt):
    """Look and speak, blocking - call it only while the robot stands still."""
    try:
        print('\n%s:' % talker.name, end=' ', flush=True)
        talker.look(prompt, on_sentence=lambda s: print(s, end=' ', flush=True))
        print(flush=True)
    except Exception as e:          # never let a look take down the walk
        print('\n[look failed: %s]' % e, flush=True)


class Prepared(threading.Thread):
    """The finale's line, written in the background while he is still walking.

    Frame grab + Gemini take ~3-4 s; done during the last metre of a person
    approach, he can speak the moment he arrives. result is the text, or None
    if it failed (the finale is then done live, the slow way).
    """

    def __init__(self, talker, prompt):
        super().__init__(daemon=True)
        self.talker, self.prompt, self.result = talker, prompt, None

    def run(self):
        try:
            self.result = self.talker.look(self.prompt, speak=False)
        except Exception as e:
            print('\n[preparing the finale failed: %s]' % e, flush=True)


def follow_and_talk(bot, a, det, talker, seconds):
    """Follow the person in segments of --talk-every seconds, stopping after
    each to say what he sees. The line is written during the last PREP_LEAD
    seconds of the segment so he can speak as soon as he halts."""
    from robot import person
    end = time.time() + seconds
    while time.time() < end - 1.0:
        seg = min(a.talk_every, end - time.time(), 29.0)   # steer() caps at 30 s
        prep = Prepared(talker, FOLLOWING)
        timer = threading.Timer(max(0.0, seg - PREP_LEAD), prep.start)
        timer.start()
        try:
            r = person.follow(bot, det, seg, stop_distance=a.stop, speed=a.speed)
        finally:
            timer.cancel()
        print('\n[followed: moved %.2f m, %s]' % (r['moved'], r['reason']), flush=True)
        if prep.ident is not None:            # the timer did start it
            prep.join(timeout=15)
        if prep.result:
            print('%s:' % talker.name, end=' ', flush=True)
            talker.say(prep.result, on_sentence=lambda s: print(s, end=' ', flush=True))
            print(flush=True)
        else:
            comment(talker, FOLLOWING)
        if r['reason'] != 'time limit':       # lost them, or no depth: stop
            break


def run_step(bot, a, verb, args, det=None, near=None, talker=None):
    from robot.lite3 import Lite3Error
    try:
        if verb == 'person':
            from robot import person
            if not det.ready.is_set():
                det.ready.wait(6)
            r = person.approach(bot, det, stop_distance=a.stop, speed=a.speed,
                                near=near)
            print('\n[person: moved %.2f m, %s]' % (r['moved'], r['reason']))
        elif verb == 'follow':
            follow_and_talk(bot, a, det, talker, args[0])
        elif verb == 'walk':
            # walk() samples the depth camera at 4 Hz and has no slow-down
            # ramp, so give it half a second of travel as extra margin.
            r = bot.walk(args[0], speed=a.speed,
                         stop_distance=None if a.blind else a.stop + 0.5 * a.speed)
            print('\n[walked %.2f m, %s]' % (r['moved'], r['reason']))
        elif verb == 'turn':
            bot.turn_deg(args[0])
            print('\n[turned %+.0f deg]' % args[0])
        elif verb == 'approach':
            prof = settled_profile(bot, args[0])
            print('\n[costmap ahead: %s]' % ' '.join(
                '%.1f:%s' % (d, '?' if c is None else c) for d, c in prof[::3]))
            # Walk the line outwards and stop at the first cell that is not
            # free enough: going to a free cell BEHIND an obstacle would
            # route around it, i.e. past the thing being approached.
            goal = None
            for d, c in prof:
                if c is None or c >= FREE_ENOUGH:
                    break
                goal = d
            if goal is None or goal - APPROACH_MARGIN < MIN_GOAL:
                raise Lite3Error('obstacle within 0.5 m ahead, nowhere to approach to')
            goal -= APPROACH_MARGIN
            print('[approach: goal %.1f m ahead]' % goal)
            status = bot.goto(goal)
            print('\n[approach %.1f m: %s]' % (goal, NAV_STATUS.get(status, 'status %s' % status)))
        else:
            want = args[0]
            prof = settled_profile(bot, want + 1.5)
            print('\n[costmap ahead: %s]' % ' '.join(
                '%.1f:%s' % (d, '?' if c is None else c) for d, c in prof[::3]))
            if not any(abs(d - want) < 0.05 and c is not None and c < FREE_ENOUGH
                       for d, c in prof):
                # Nearest free distance on the line, ties to the shorter one.
                # Beyond an obstacle is allowed: Nav2 routes around it.
                free = [d for d, c in prof
                        if d >= MIN_GOAL and c is not None and c < FREE_ENOUGH]
                if not free:
                    raise Lite3Error('nothing free between %.1f and %.1f m ahead'
                                     % (MIN_GOAL, want + 1.5))
                near = min(free, key=lambda d: (abs(d - want), d))
                print('\n[goto %.1f m is inside an obstacle; using %.1f m, the '
                      'closest free distance]' % (want, near))
                want = near
            x0, y0, _ = bot.wait_pose()
            status = bot.goto(want, *args[1:])
            x1, y1, _ = bot.wait_pose()
            time.sleep(2.0)
            x2, y2, _ = bot.wait_pose()
            print('\n[goto %.1f m: %s - moved %.2f m, then drifted %.2f m in 2 s]' % (
                want, NAV_STATUS.get(status, 'status %s' % status),
                math.hypot(x1 - x0, y1 - y0), math.hypot(x2 - x1, y2 - y1)))
    except Lite3Error as e:
        print('\n[%s %s skipped: %s]' % (verb, ' '.join('%g' % x for x in args), e))


def steps_from_stdin():
    print('Steps: walk <m> | turn <deg> | goto <m> [<deg>] | approach <m> | '
          'person | follow <s>. Ctrl-D to finish.')
    while True:
        try:
            line = input('step> ').strip()
        except EOFError:
            return
        if line:
            try:
                yield parse_step(line)
            except ValueError as e:
                print(e)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('route', nargs='?', default='walk 2')
    ap.add_argument('--interactive', action='store_true',
                    help='read steps from the keyboard instead of the route')
    ap.add_argument('--talk-every', type=float, default=20.0, metavar='S',
                    help='follow: seconds of following between talking stops')
    ap.add_argument('--speed', type=float, default=0.3,
                    help='m/s, max 0.6. Below ~0.2 the gait shuffles and drifts '
                         'sideways. walk steps add speed * 0.5 s to --stop as '
                         'reaction margin; person slows down near the target')
    ap.add_argument('--stop', type=float, default=1.0,
                    help='obstacle stop distance in metres, measured from the '
                         'BODY CENTRE: the nose is ~0.33 m ahead of it, so '
                         '0.45 stops ~12 cm short of the nose')
    ap.add_argument('--blind', action='store_true',
                    help='walk without the depth-camera obstacle stop')
    ap.add_argument('--handheld', dest='heartbeat', action='store_false',
                    help='use the powered-on handheld instead of our own heartbeat')
    ap.add_argument('--dry-run', action='store_true', help='one comment, no movement')
    ap.add_argument('--voice', default=None)
    ap.add_argument('--persona', default=None, help='deadpan (default), sarcastic or rocky')
    ap.add_argument('--finale', default=None,
                    help='at the end, still standing, look and answer this '
                         'instead of the usual closing line')
    ap.add_argument('--look-up', type=float, default=12.0, metavar='DEG',
                    help='tilt the body nose-up this much for the finale, so '
                         'the camera sees a face, not knees (max ~14, 0 = off)')
    a = ap.parse_args()
    route = None if a.interactive else parse_route(a.route)
    use_nav = a.interactive or any(v in ('goto', 'approach') for v, _ in route)
    if not a.blind and a.stop < 0.4:
        raise SystemExit('--stop %.2f is inside or at the robot\'s nose (~0.33 m from '
                         'the body centre the distance is measured from); the '
                         'camera cannot see that close either. Use >= 0.4.' % a.stop)

    det = None
    if a.interactive or any(v in ('person', 'follow') for v, _ in route):
        from robot import person
        det = person.PersonDetector()
        det.start()

    talker = Talker(**{k: v for k, v in (('voice', a.voice), ('persona', a.persona)) if v})
    if talker.live:
        # Connect to Gemini while he stands up and walks, not when he has
        # to speak.
        threading.Thread(target=talker.live.warm, daemon=True).start()
    try:
        if a.dry_run:
            comment(talker, PAUSED)
            return 0
        from robot.lite3 import Lite3
        with Lite3() as bot:
            track = Odometer(bot)
            if a.heartbeat:
                bot.heartbeat_start()
            try:
                # No sleep needed here. From 8 the heartbeat auto-arms and
                # wait_ready() blocks out the ~2 s handover; from a cold-boot
                # 98 nothing arms until something commands him, so it returns
                # at once - and stand() re-sends the toggle that arming eats.
                if a.heartbeat and not bot.wait_ready():
                    raise SystemExit('interlock never came up')
                bot.stand()
                if use_nav:
                    # Standing makes leg odometry jump; nav_start() refuses
                    # until the pose has settled, so give it a moment first.
                    deadline = time.time() + 15
                    while not bot.pose_settled() and time.time() < deadline:
                        time.sleep(0.5)
                    print('[starting Nav2...]', flush=True)
                    bot.nav_start()
                    print('[Nav2 up]', flush=True)
                track.reset()
                prep = None
                # Every step ends halted, so after it is a quiet moment to
                # talk. On a fixed route the last pause is the finale's.
                if a.interactive:
                    for verb, args in steps_from_stdin():
                        run_step(bot, a, verb, args, det, talker=talker)
                        track.sample()
                        if verb != 'follow':          # follow talks as it goes
                            comment(talker, PAUSED)
                else:
                    for i, (verb, args) in enumerate(route):
                        near = None
                        if (a.finale and i == len(route) - 1 and verb == 'person'
                                and not a.look_up):
                            # Write the finale during the last metre so it can
                            # be spoken on arrival. Only when he is NOT tilting
                            # up: that frame is grabbed mid-walk, before the
                            # tilt, so he would describe knees and shoes. With
                            # --look-up the finale is done live after tilting,
                            # which costs ~3 s of silence but sees the person.
                            prep = Prepared(talker, PREP_CONTEXT + a.finale)
                            near = prep.start
                        run_step(bot, a, verb, args, det, near, talker)
                        track.sample()
                        if verb != 'follow' and (i < len(route) - 1 or not a.finale):
                            comment(talker, PAUSED)
                if a.finale:
                    if prep and prep.is_alive():
                        prep.join(timeout=15)
                    ready = prep.result if prep else None
                    with (bot.tilt(a.look_up, settle=0.3) if a.look_up
                          else contextlib.nullcontext()):
                        if ready:
                            print('\n%s:' % talker.name, end=' ', flush=True)
                            talker.say(ready, on_sentence=lambda s: print(s, end=' ', flush=True))
                            print(flush=True)
                        else:
                            # Short settle above: look() spends ~2 s grabbing
                            # a frame anyway, and by then he is tilted.
                            comment(talker, a.finale)
            finally:
                bot.halt()
                try:
                    if use_nav:
                        bot.nav_stop()
                    bot.sit()               # sit FIRST ...
                finally:
                    if a.heartbeat:
                        bot.heartbeat_stop()    # ... disarm SECOND
        if a.finale:
            return 0
        print('\n%s:' % talker.name, end=' ', flush=True)
        talker.ask(DONE % track.total, on_sentence=lambda s: print(s, end=' ', flush=True))
        print()
        return 0
    finally:
        if det:
            det.stop()
        talker.close()


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""demo.py - a guided tour of the lite3 API. Run the lessons in order.

    source ~/robot/env/lite3_env.sh   # ALWAYS do this first
    python3 ~/robot/demos/demo.py     # list the lessons
    python3 ~/robot/demos/demo.py 1   # run one lesson
    python3 ~/robot/demos/demo.py read  # every lesson that cannot move the robot

Lessons 5 and up move the robot, so they refuse to run unless you add --move:

    python3 ~/robot/demos/demo.py 5 --move

Before any moving lesson: clear a few metres in front of him, and have a stop
ready - either the handheld powered on, or the terminal running the heartbeat.
"""
import math
import sys
import time

import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.lite3 import Lite3, Lite3Error


# ---------------------------------------------------------------- lesson 1
def lesson1_connect():
    """Connecting. The object is the robot; the context manager is the safety net."""
    # Lite3() spins a ROS node in a background thread and, by default, puts the
    # robot in auto mode (without which /cmd_vel is silently ignored).
    # Using it as a context manager guarantees a zero-Twist stop on the way out,
    # whether you return normally, raise, or press Ctrl-C.
    with Lite3() as bot:
        print('connected.')
        print('  one-line summary :', bot.status())

    # Pass auto_mode=False if a human is driving with the handheld and you only
    # want to observe - taking auto mode would steal control from them.
    with Lite3(auto_mode=False) as bot:
        print('  observer mode, battery:', bot.battery, '%')


# ---------------------------------------------------------------- lesson 2
def lesson2_state():
    """State is live attributes, not callbacks. No spin loop in your code."""
    with Lite3(auto_mode=False) as bot:
        # These update continuously in the background. Just read them.
        print('  battery  :', bot.battery, '%')
        print('  standing :', bot.standing)
        print('  pose     :', bot.pose)          # (x, y, yaw) in the odom frame
        print('  full state dict:', bot.state)

        # Watch the pose for a moment to prove it is live.
        print('  sampling pose for 3 s:')
        for _ in range(3):
            x, y, yaw = bot.pose
            print('    x=%+.3f y=%+.3f yaw=%+.1f deg' % (x, y, math.degrees(yaw)))
            time.sleep(1)

        # basic_state is the one that gates everything:
        #   1 or 8 or 98 = lying/ready   6 = standing   8 = no controller
        print('  basic_state:', bot.state.get('basic'))


# ---------------------------------------------------------------- lesson 3
def lesson3_perception():
    """Seeing. The depth camera is pitched 20 degrees down - the API handles it."""
    with Lite3(auto_mode=False) as bot:
        # scan() returns [(bearing_deg, range_m or None), ...], +ve bearing = LEFT.
        rows = bot.scan()
        print('  %d bearing bins' % len(rows))

        # scan_text() is the same thing as a bar chart for humans.
        print(bot.scan_text())

        # The last line of that chart is clearance(): the nearest obstacle in
        # the path, measured by LATERAL offset, so a wall off to one side is
        # not counted as being in the way. inf means nothing ahead.
        print('  clearance() on its own: %.2f m' % bot.clearance())

        # Use scan() when you care WHERE the gap is; clearance() only for
        # "is the path ahead blocked".

        # NOTE: if he is lying down, most of what the camera sees is floor at
        # close range, so expect a small clearance number. Readings are only
        # meaningful about the world once he is standing.


# ---------------------------------------------------------------- lesson 4
def lesson4_guards():
    """The API refuses unsafe things. Learn the refusals - they are the docs."""
    with Lite3(auto_mode=False) as bot:
        attempts = [
            ('walk while lying down', lambda: bot.walk(0.3)),
            ('navigate while lying down', lambda: bot.goto(1.0)),
            ('read costmap with no nav2', lambda: bot.cost_at(0, 0)),
        ]
        for label, fn in attempts:
            try:
                fn()
                print('  %-28s -> allowed (unexpected!)' % label)
            except Lite3Error as e:
                print('  %-28s -> refused:' % label)
                print('      %s' % e)

        # Note the second one: goto() checks posture BEFORE it checks whether
        # nav2 is running, so lying down you get the posture complaint. Guards
        # fire in the order that matters, not the order you might expect.

        # Every refusal is a bug that actually happened once. Catch Lite3Error
        # in your own code and you get a readable reason, not a silent no-op -
        # which is what the robot itself does.


# ---------------------------------------------------------------- lesson 5
def lesson5_posture():
    """Standing and sitting. Both are idempotent, unlike the raw toggle."""
    with Lite3() as bot:
        print('  before:', bot.status())
        bot.stand()          # no-op if already standing
        print('  standing:', bot.standing)
        time.sleep(2)
        bot.sit()            # no-op if already down
        print('  after :', bot.status())

        # The underlying command (0x21010202) is a TOGGLE, so calling it blindly
        # on a standing robot lies him down. stand()/sit() read basic_state
        # first, which is why they are safe to call from a loop.


# ---------------------------------------------------------------- lesson 6
def lesson6_motion():
    """Walking and turning. Blocking calls that always leave a zero Twist."""
    with Lite3() as bot:
        bot.stand()
        if bot.clearance() < 1.2:
            print('  not enough room ahead (%.2f m) - skipping' % bot.clearance())
            return

        r = bot.walk(0.4)                    # metres; negative walks backward
        print('  walked %.3f m (%s)' % (r['moved'], r['reason']))

        r = bot.turn_deg(45)                 # +ve is LEFT; turn() takes radians
        print('  turned %.1f deg' % r['turned_deg'])
        bot.turn_deg(-45)

        # Expect a few degrees of overshoot: termination is checked at 20 Hz
        # against odometry with no deceleration ramp. Fine for navigation,
        # not for precise heading.
        bot.sit()


# ---------------------------------------------------------------- lesson 7
def lesson7_navigation():
    """Nav2: goal-directed movement that routes AROUND obstacles."""
    with Lite3() as bot:
        bot.stand()
        time.sleep(1.5)              # let the pose settle before the costmap builds

        # nav_start() refuses unless he is standing AND the pose has settled.
        # The stand-up transition jumps odometry by over a metre, and a costmap
        # built across that jump shows a clear path into a real obstacle.
        bot.nav_start()
        try:
            # ALWAYS pick the goal from the costmap, never from scan().
            # The costmap remembers obstacles the camera cannot currently see.
            profile = bot.cost_ahead(out_to=3.0)
            for d, c in profile:
                mark = ' LETHAL' if c is not None and c >= 99 else ''
                print('    +%4.2f m  cost %s%s' % (d, c, mark))

            free = [d for d, c in profile if c is not None and c < 50 and d >= 0.75]
            if not free:
                print('  no free cell beyond 0.75 m - not sending a goal')
                return
            goal = min(1.75, max(free) - 0.25)

            status = bot.goto(goal)          # 4 = SUCCEEDED, 6 = ABORTED
            print('  goto(%.2f) -> status %s' % (goal, status))
        finally:
            bot.nav_stop()                   # always tear the stack down
            bot.sit()


# ---------------------------------------------------------------- lesson 8
def lesson8_estop():
    """Stopping. Ctrl-C is wired to a real emergency stop."""
    with Lite3(auto_mode=False) as bot:
        print("""
  Ctrl-C during any API call runs estop(), which:
    1. ignores further SIGINT, so a second Ctrl-C cannot abort the stop
    2. cancels the active Nav2 goal
    3. publishes a burst of zero Twists
    4. KILLS the Nav2 stack  <- the important one: controller_server is a
       separate process and keeps publishing cmd_vel otherwise
    5. publishes zero Twists again

  Measured mid-goal on the real robot: 0.000 m of coast.

  You can also call it directly, e.g. from your own error handler:""")
        print('  bot.estop() ->', bot.estop())

        # estop(disarm=True) additionally drops the controller keepalive, after
        # which the robot ignores every network command. Strongest stop there
        # is, but untested while standing - so it is opt-in, not the default.

        # Opt out of the signal handler with Lite3(estop_on_sigint=False) if
        # your program needs its own Ctrl-C behaviour.


# ---------------------------------------------------------------- lesson 9
def lesson9_behaviour():
    """Putting it together: a tiny behaviour - creep forward while it is clear."""
    with Lite3() as bot:
        bot.stand()
        travelled = 0.0
        while travelled < 1.2:
            clear = bot.clearance()
            print('  clearance %.2f m, travelled %.2f m' % (clear, travelled))
            if clear < 0.8:
                print('  blocked - stopping')
                break
            step = min(0.3, clear - 0.5)
            if step < 0.15:
                break
            travelled += bot.walk(step)['moved']
        bot.sit()
        print('  done, travelled %.2f m' % travelled)

        # Note what you did NOT have to write: no rclpy.init, no spin loop, no
        # callbacks, no zero-Twist cleanup, no battery check, no mode command.


# --------------------------------------------------------------- lesson 10
def lesson10_no_controller():
    """Running with no handheld: hold the interlock from your own script."""
    with Lite3() as bot:
        # jy_exe only accepts commands while a controller keepalive keeps
        # arriving. Normally that is the handheld. heartbeat_start() makes this
        # process the controller instead, so the robot obeys with no handheld
        # powered on at all.
        #
        # Read that twice: while this runs, the handheld is NOT your stop.
        # This terminal is. Ctrl-C stops the keepalive and the robot goes deaf
        # to every network command.
            # NOT instant: basic_state runs 8 -> 9 -> 1 over about two seconds,
            # and a command sent inside that window is ignored, then times out
            # complaining about the handheld. So wait for it.
        if not bot.wait_ready():
            print('  interlock never came up - is transfer_ros2 running?')
            return
        print('  interlock now held by us, basic_state =',
                bot.state.get('basic'))

        try:
            bot.heartbeat_start()
            time.sleep(2)
            bot.stand()
            bot.turn_deg(180)
            
            
        finally:
            bot.heartbeat_stop()

LESSONS = [
    (1, 'connect          ', lesson1_connect,     False),
    (2, 'live state       ', lesson2_state,       False),
    (3, 'perception       ', lesson3_perception,  False),
    (4, 'guard rails      ', lesson4_guards,      False),
    (5, 'stand and sit    ', lesson5_posture,     True),
    (6, 'walk and turn    ', lesson6_motion,      True),
    (7, 'nav2 goals       ', lesson7_navigation,  True),
    (8, 'emergency stop   ', lesson8_estop,       False),
    (9, 'a real behaviour ', lesson9_behaviour,   True),
    (10, 'no handheld     ', lesson10_no_controller, True),
]


def main():
    args = sys.argv[1:]
    move_ok = '--move' in args
    args = [a for a in args if a != '--move']

    if not args:
        print(__doc__)
        print('lessons:')
        for n, name, fn, moves in LESSONS:
            print('  %d  %s %s' % (n, name, '[MOVES THE ROBOT]' if moves else ''))
        return

    if args[0] == 'read':
        chosen = [l for l in LESSONS if not l[3]]
    elif args[0] == 'all':
        chosen = LESSONS
    else:
        want = {int(a) for a in args}
        chosen = [l for l in LESSONS if l[0] in want]

    for n, name, fn, moves in chosen:
        if moves and not move_ok:
            print('\n=== lesson %d: %s SKIPPED (needs --move) ===' % (n, name.strip()))
            continue
        print('\n=== lesson %d: %s ===' % (n, name.strip()))
        print('    %s' % (fn.__doc__ or '').strip().splitlines()[0])
        try:
            fn()
        except Lite3Error as e:
            print('  refused: %s' % e)
        except KeyboardInterrupt:
            print('  interrupted - e-stop ran, robot stopped')
            break


if __name__ == '__main__':
    main()

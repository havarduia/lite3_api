import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.lite3 import Lite3
from robot.nav import goal_ahead, status_text

results = []
def step(name, fn):
    try:
        r = fn()
        results.append((name, 'OK'))
        print('[OK]   %-16s %s' % (name, r))
        return r
    except Exception as e:
        results.append((name, 'FAIL'))
        print('[FAIL] %-16s %s' % (name, e))
        return None

with Lite3() as bot:
    print('start:', bot.status())
    try:
        # No handheld: this process holds the interlock instead. SAFETY - that
        # makes THIS TERMINAL the only stop. Ctrl-C runs estop() (Lite3 installs
        # the handler by default) and close() drops the keepalive on the way out.
        bot.heartbeat_start()
        if not bot.wait_ready():
            raise SystemExit('interlock never came up - is transfer_ros2 running?')
        step('stand', bot.stand)
        print('       status:', bot.status())

        clear = bot.clearance()
        print('       clearance ahead: %.2f m' % clear)
        dist = 0.5 if clear > 1.1 else max(0.0, clear - 0.6)
        if dist >= 0.2:
            r = step('walk %.2fm' % dist, lambda: bot.walk(dist))
            if r: print('       moved %.3f m (%s)' % (r['moved'], r['reason']))
        else:
            print('[SKIP] walk - only %.2f m of clearance' % clear)

        r = step('turn +90', lambda: bot.turn_deg(90))
        if r: print('       turned %.1f deg' % r['turned_deg'])
        r = step('turn -90', lambda: bot.turn_deg(-90))
        if r: print('       turned %.1f deg' % r['turned_deg'])

        step('nav_start', bot.nav_start)
        if bot.nav_running:
            profile = bot.cost_ahead(out_to=3.0, settle=10)
            print('       cost ahead:', [(round(d,2), c) for d, c in profile])
            goal = goal_ahead(profile)
            if goal is not None:
                step('goto %.2fm' % goal, lambda: status_text(bot.goto(goal, timeout=90)))
            else:
                print('[SKIP] goto - under 0.75 m of free floor ahead')
            step('nav_stop', bot.nav_stop)
    finally:
        step('sit', bot.sit)
        print('end:', bot.status())

print('\n==== SUMMARY ====')
for n, s in results:
    print('  %-14s %s' % (n, s))
